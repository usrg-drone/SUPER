from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="super_px4_mavros_offboard_bridge",
                executable="offboard_bridge_node",
                name="super_px4_mavros_offboard_bridge",
                output="screen",
                parameters=[
                    {
                        "odom_topic": "/Odometry",
                        "pos_cmd_topic": "/planning/pos_cmd",
                        "vision_pose_topic": "/mavros/vision_pose/pose",
                        "setpoint_topic": "/mavros/setpoint_raw/local",
                        "arming_service": "/mavros/cmd/arming",
                        "set_mode_service": "/mavros/set_mode",
                        "state_topic": "/mavros/state",
                    }
                ],
            )
        ]
    )
