import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from mars_quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import SetBool


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _quat_normalize(q: Quaternion) -> Quaternion:
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-9:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def _quat_conjugate(q: Quaternion) -> Quaternion:
    x, y, z, w = q
    return -x, -y, -z, w


def _quat_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return _quat_normalize(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def _quat_rotate(q: Quaternion, v: Vector3) -> Vector3:
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _quat_angle(a: Quaternion, b: Quaternion) -> float:
    delta = _quat_multiply(b, _quat_conjugate(a))
    x, y, z, w = delta
    return 2.0 * math.atan2(math.sqrt(x * x + y * y + z * z), abs(w))


def _yaw_from_quat(q: Quaternion) -> float:
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _position_target_mask(*names: str) -> int:
    mask = 0
    for name in names:
        mask |= int(getattr(PositionTarget, name))
    return mask


class SuperPx4MavrosOffboardBridge(Node):
    def __init__(self) -> None:
        super().__init__("super_px4_mavros_offboard_bridge")

        self.odom_topic = self.declare_parameter("odom_topic", "/Odometry").value
        self.pos_cmd_topic = self.declare_parameter("pos_cmd_topic", "/planning/pos_cmd").value
        self.vision_pose_topic = self.declare_parameter("vision_pose_topic", "/mavros/vision_pose/pose").value
        self.setpoint_topic = self.declare_parameter("setpoint_topic", "/mavros/setpoint_raw/local").value
        self.state_topic = self.declare_parameter("state_topic", "/mavros/state").value
        self.arming_service_name = self.declare_parameter("arming_service", "/mavros/cmd/arming").value
        self.set_mode_service_name = self.declare_parameter("set_mode_service", "/mavros/set_mode").value

        self.setpoint_frame_id = self.declare_parameter("setpoint_frame_id", "camera_init").value
        self.timer_rate_hz = float(self.declare_parameter("timer_rate_hz", 50.0).value)
        self.stale_command_timeout_s = float(self.declare_parameter("stale_command_timeout_s", 0.5).value)
        self.mode_request_interval_s = float(self.declare_parameter("mode_request_interval_s", 1.0).value)
        self.takeoff_altitude = float(self.declare_parameter("takeoff_altitude", 1.5).value)
        self.jump_position_threshold_m = float(
            self.declare_parameter("jump_position_threshold_m", 1.0).value
        )
        self.jump_angle_threshold_rad = float(
            self.declare_parameter("jump_angle_threshold_rad", 0.75).value
        )

        self.arm = bool(self.declare_parameter("arm", False).value)
        self.offboard_mode = bool(self.declare_parameter("offboard_mode", False).value)
        self.hold_position = bool(self.declare_parameter("hold_position", False).value)
        self.planner_enabled = bool(self.declare_parameter("planner_enabled", False).value)
        self.publish_vision_pose = bool(self.declare_parameter("publish_vision_pose", True).value)
        self.publish_setpoints = bool(self.declare_parameter("publish_setpoints", True).value)

        self.state: Optional[State] = None
        self.last_mode_request_time: Optional[Time] = None
        self.last_arm_request: Optional[bool] = None
        self.last_arm_request_time: Optional[Time] = None
        self.suppress_arm_request = False

        self.offset_t: Vector3 = (0.0, 0.0, 0.0)
        self.offset_q: Quaternion = (0.0, 0.0, 0.0, 1.0)
        self.last_output_position: Optional[Vector3] = None
        self.last_output_orientation: Optional[Quaternion] = None
        self.latest_continuous_pose: Optional[PoseStamped] = None

        self.latest_planner_setpoint: Optional[PositionTarget] = None
        self.latest_planner_time: Optional[Time] = None
        self.hold_setpoint: Optional[PositionTarget] = None
        self.have_published_setpoint = False
        self.was_hold_position = self.hold_position
        self.was_takeoff_hold_active = False

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, sensor_qos)
        self.state_sub = self.create_subscription(State, self.state_topic, self.state_callback, 10)
        self.pos_cmd_sub = self.create_subscription(
            PositionCommand, self.pos_cmd_topic, self.pos_cmd_callback, sensor_qos
        )

        self.vision_pose_pub = self.create_publisher(PoseStamped, self.vision_pose_topic, 10)
        self.setpoint_pub = self.create_publisher(PositionTarget, self.setpoint_topic, 10)
        self.arming_client = self.create_client(CommandBool, self.arming_service_name)
        self.set_mode_client = self.create_client(SetMode, self.set_mode_service_name)

        self.create_service(SetBool, "~/set_arm", self.set_arm_service_callback)
        self.create_service(SetBool, "~/set_offboard_mode", self.set_offboard_mode_service_callback)
        self.create_service(SetBool, "~/set_hold_position", self.set_hold_position_service_callback)
        self.create_service(SetBool, "~/set_planner_enabled", self.set_planner_enabled_service_callback)
        self.create_service(SetBool, "~/set_publish_vision_pose", self.set_publish_vision_pose_service_callback)
        self.create_service(SetBool, "~/set_publish_setpoints", self.set_publish_setpoints_service_callback)

        period = 1.0 / max(1.0, self.timer_rate_hz)
        self.timer = self.create_timer(period, self.timer_callback)
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info(
            "Bridge ready: %s -> %s, %s -> %s"
            % (self.odom_topic, self.vision_pose_topic, self.pos_cmd_topic, self.setpoint_topic)
        )

    def parameters_callback(self, params):
        old_arm = self.arm
        old_hold = self.hold_position
        old_takeoff_altitude = self.takeoff_altitude
        for param in params:
            if param.type_ == Parameter.Type.NOT_SET:
                continue
            if param.name in {
                "arm",
                "offboard_mode",
                "hold_position",
                "planner_enabled",
                "publish_vision_pose",
                "publish_setpoints",
            }:
                setattr(self, param.name, bool(param.value))
            elif param.name in {
                "timer_rate_hz",
                "stale_command_timeout_s",
                "mode_request_interval_s",
                "takeoff_altitude",
                "jump_position_threshold_m",
                "jump_angle_threshold_rad",
            }:
                setattr(self, param.name, float(param.value))
            elif param.name in {"setpoint_frame_id"}:
                setattr(self, param.name, str(param.value))

        if old_hold != self.hold_position:
            self.hold_setpoint = None
        if abs(old_takeoff_altitude - self.takeoff_altitude) > 1.0e-6:
            self.hold_setpoint = None
        if old_arm != self.arm and not self.suppress_arm_request:
            self.request_arm(self.arm)
        return SetParametersResult(successful=True)

    def set_bool_parameter(self, name: str, value: bool) -> bool:
        results = self.set_parameters([Parameter(name, Parameter.Type.BOOL, bool(value))])
        return bool(results and results[0].successful)

    def set_bool_service_response(self, response, name: str, value: bool):
        response.success = self.set_bool_parameter(name, value)
        state = "enabled" if value else "disabled"
        response.message = "%s %s" % (name, state) if response.success else "Failed to set %s" % name
        return response

    def set_arm_service_callback(self, request, response):
        return self.set_bool_service_response(response, "arm", request.data)

    def set_offboard_mode_service_callback(self, request, response):
        return self.set_bool_service_response(response, "offboard_mode", request.data)

    def set_hold_position_service_callback(self, request, response):
        return self.set_bool_service_response(response, "hold_position", request.data)

    def set_planner_enabled_service_callback(self, request, response):
        return self.set_bool_service_response(response, "planner_enabled", request.data)

    def set_publish_vision_pose_service_callback(self, request, response):
        return self.set_bool_service_response(response, "publish_vision_pose", request.data)

    def set_publish_setpoints_service_callback(self, request, response):
        return self.set_bool_service_response(response, "publish_setpoints", request.data)

    def state_callback(self, msg: State) -> None:
        was_armed = self.state is not None and self.state.armed
        was_offboard = self.state is not None and self.state.mode == "OFFBOARD"
        self.state = msg

        if was_armed and not msg.armed:
            if self.arm or self.offboard_mode or self.planner_enabled or self.hold_position:
                self.clear_control_intent("PX4 reported disarmed", clear_arm=True)
            return

        if not msg.armed:
            if self.offboard_mode or self.planner_enabled or self.hold_position:
                self.clear_control_intent("PX4 is disarmed", clear_arm=False)
            return

        if was_offboard and msg.mode != "OFFBOARD" and self.offboard_mode:
            self.clear_control_intent("PX4 left OFFBOARD mode", clear_arm=False)

    def clear_control_intent(self, reason: str, clear_arm: bool) -> None:
        params = [
            Parameter("offboard_mode", Parameter.Type.BOOL, False),
            Parameter("planner_enabled", Parameter.Type.BOOL, False),
            Parameter("hold_position", Parameter.Type.BOOL, False),
        ]
        if clear_arm:
            params.append(Parameter("arm", Parameter.Type.BOOL, False))

        self.suppress_arm_request = True
        try:
            self.set_parameters(params)
        finally:
            self.suppress_arm_request = False

        self.hold_setpoint = None
        self.have_published_setpoint = False
        self.last_mode_request_time = None
        self.get_logger().warn("%s; cleared bridge offboard control intent" % reason)

    def odom_callback(self, msg: Odometry) -> None:
        raw_p = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        raw_q = _quat_normalize(
            (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
        )

        out_p = self.apply_offset_position(raw_p)
        out_q = _quat_multiply(self.offset_q, raw_q)

        if self.last_output_position is not None and self.last_output_orientation is not None:
            dp = self.distance(out_p, self.last_output_position)
            da = _quat_angle(out_q, self.last_output_orientation)
            if dp > self.jump_position_threshold_m or da > self.jump_angle_threshold_rad:
                self.offset_q = _quat_multiply(self.last_output_orientation, _quat_conjugate(raw_q))
                rotated_raw = _quat_rotate(self.offset_q, raw_p)
                self.offset_t = (
                    self.last_output_position[0] - rotated_raw[0],
                    self.last_output_position[1] - rotated_raw[1],
                    self.last_output_position[2] - rotated_raw[2],
                )
                out_p = self.last_output_position
                out_q = self.last_output_orientation
                self.get_logger().warn(
                    "Detected SLAM pose discontinuity; adjusted external-vision continuity offset"
                )

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = out_p[0]
        pose.pose.position.y = out_p[1]
        pose.pose.position.z = out_p[2]
        pose.pose.orientation.x = out_q[0]
        pose.pose.orientation.y = out_q[1]
        pose.pose.orientation.z = out_q[2]
        pose.pose.orientation.w = out_q[3]

        self.latest_continuous_pose = pose
        self.last_output_position = out_p
        self.last_output_orientation = out_q

        if self.publish_vision_pose:
            self.vision_pose_pub.publish(pose)

    def pos_cmd_callback(self, msg: PositionCommand) -> None:
        setpoint = PositionTarget()
        setpoint.header = msg.header
        setpoint.header.stamp = self.get_clock().now().to_msg()
        if self.setpoint_frame_id:
            setpoint.header.frame_id = self.setpoint_frame_id
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.type_mask = 0
        setpoint.position.x = msg.position.x
        setpoint.position.y = msg.position.y
        setpoint.position.z = msg.position.z
        setpoint.velocity.x = msg.velocity.x
        setpoint.velocity.y = msg.velocity.y
        setpoint.velocity.z = msg.velocity.z
        setpoint.acceleration_or_force.x = msg.acceleration.x
        setpoint.acceleration_or_force.y = msg.acceleration.y
        setpoint.acceleration_or_force.z = msg.acceleration.z
        setpoint.yaw = float(msg.yaw)
        setpoint.yaw_rate = float(msg.yaw_dot)
        self.latest_planner_setpoint = setpoint
        self.latest_planner_time = self.get_clock().now()

    def timer_callback(self) -> None:
        now = self.get_clock().now()

        if self.arm and self.should_request_arm(now):
            self.request_arm(True)

        takeoff_hold_active = self.arm and self.offboard_mode and not self.planner_enabled
        if self.hold_position and not self.was_hold_position:
            self.hold_setpoint = None
        if takeoff_hold_active and not self.was_takeoff_hold_active:
            self.hold_setpoint = None
        self.was_hold_position = self.hold_position
        self.was_takeoff_hold_active = takeoff_hold_active

        if self.publish_setpoints:
            setpoint = self.select_setpoint(takeoff_hold_active)
            if setpoint is not None:
                setpoint.header.stamp = now.to_msg()
                self.setpoint_pub.publish(setpoint)
                self.have_published_setpoint = True

        if self.offboard_mode and self.have_published_setpoint and self.should_request_mode(now):
            self.request_offboard()

    def select_setpoint(self, takeoff_hold_active: bool) -> Optional[PositionTarget]:
        if self.hold_position or takeoff_hold_active:
            if self.hold_setpoint is None:
                self.hold_setpoint = self.make_hold_setpoint(use_takeoff_altitude=takeoff_hold_active)
            return self.hold_setpoint

        if (
            self.planner_enabled
            and self.latest_planner_setpoint is not None
            and self.latest_planner_time is not None
        ):
            age = (self.get_clock().now() - self.latest_planner_time).nanoseconds * 1.0e-9
            if age <= self.stale_command_timeout_s:
                return self.latest_planner_setpoint
        return None

    def make_hold_setpoint(self, use_takeoff_altitude: bool) -> Optional[PositionTarget]:
        if self.latest_continuous_pose is None:
            return None

        pose = self.latest_continuous_pose.pose
        q = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        setpoint = PositionTarget()
        setpoint.header.frame_id = self.setpoint_frame_id
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.type_mask = _position_target_mask(
            "IGNORE_VX",
            "IGNORE_VY",
            "IGNORE_VZ",
            "IGNORE_AFX",
            "IGNORE_AFY",
            "IGNORE_AFZ",
            "IGNORE_YAW_RATE",
        )
        setpoint.position.x = pose.position.x
        setpoint.position.y = pose.position.y
        setpoint.position.z = self.takeoff_altitude if use_takeoff_altitude else pose.position.z
        setpoint.yaw = _yaw_from_quat(q)
        return setpoint

    def request_arm(self, value: bool) -> None:
        if not self.arming_client.service_is_ready():
            self.arming_client.wait_for_service(timeout_sec=0.01)
        if not self.arming_client.service_is_ready():
            return
        request = CommandBool.Request()
        request.value = bool(value)
        future = self.arming_client.call_async(request)
        future.add_done_callback(lambda fut: self.log_arm_response(fut, value))
        self.last_arm_request = value
        self.last_arm_request_time = self.get_clock().now()

    def request_offboard(self) -> None:
        if not self.set_mode_client.service_is_ready():
            self.set_mode_client.wait_for_service(timeout_sec=0.01)
        if not self.set_mode_client.service_is_ready():
            return
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = "OFFBOARD"
        future = self.set_mode_client.call_async(request)
        future.add_done_callback(self.log_mode_response)
        self.last_mode_request_time = self.get_clock().now()

    def should_request_arm(self, now: Time) -> bool:
        if self.state is not None and self.state.armed:
            return False
        if self.last_arm_request is not True or self.last_arm_request_time is None:
            return True
        age = (now - self.last_arm_request_time).nanoseconds * 1.0e-9
        return age >= self.mode_request_interval_s

    def should_request_mode(self, now: Time) -> bool:
        if self.state is not None and self.state.mode == "OFFBOARD":
            return False
        if self.last_mode_request_time is None:
            return True
        age = (now - self.last_mode_request_time).nanoseconds * 1.0e-9
        return age >= self.mode_request_interval_s

    def log_arm_response(self, future, value: bool) -> None:
        try:
            result = future.result()
            if not result.success:
                self.get_logger().warn("Arm request failed" if value else "Disarm request failed")
        except Exception as exc:
            self.get_logger().warn("Arm service call failed: %s" % exc)

    def log_mode_response(self, future) -> None:
        try:
            result = future.result()
            if not result.mode_sent:
                self.get_logger().warn("OFFBOARD mode request was not accepted by MAVROS")
        except Exception as exc:
            self.get_logger().warn("SetMode service call failed: %s" % exc)

    def apply_offset_position(self, raw_p: Vector3) -> Vector3:
        rotated = _quat_rotate(self.offset_q, raw_p)
        return (
            rotated[0] + self.offset_t[0],
            rotated[1] + self.offset_t[1],
            rotated[2] + self.offset_t[2],
        )

    @staticmethod
    def distance(a: Vector3, b: Vector3) -> float:
        return math.sqrt(
            (a[0] - b[0]) * (a[0] - b[0])
            + (a[1] - b[1]) * (a[1] - b[1])
            + (a[2] - b[2]) * (a[2] - b[2])
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SuperPx4MavrosOffboardBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
