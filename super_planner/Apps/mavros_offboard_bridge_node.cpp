#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mars_quadrotor_msgs/msg/position_command.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class MavrosOffboardBridgeNode final : public rclcpp::Node {
public:
    MavrosOffboardBridgeNode() : Node("mavros_offboard_bridge_node") {
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
        pos_cmd_topic_ = declare_parameter<std::string>("pos_cmd_topic", "/planning/pos_cmd");
        vision_pose_topic_ = declare_parameter<std::string>("vision_pose_topic", "/mavros/vision_pose/pose");
        setpoint_topic_ = declare_parameter<std::string>("setpoint_topic", "/mavros/setpoint_raw/local");
        setpoint_frame_id_ = declare_parameter<std::string>("setpoint_frame_id", "world");
        timer_rate_hz_ = declare_parameter<double>("timer_rate_hz", 50.0);
        stale_command_timeout_s_ = declare_parameter<double>("stale_command_timeout_s", 0.5);
        publish_vision_pose_ = declare_parameter<bool>("publish_vision_pose", true);
        publish_setpoints_ = declare_parameter<bool>("publish_setpoints", true);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                odom_topic_,
                rclcpp::SensorDataQoS(),
                std::bind(&MavrosOffboardBridgeNode::odomCallback, this, std::placeholders::_1));

        const auto cmd_qos = rclcpp::QoS(10).best_effort().keep_last(10).durability_volatile();
        pos_cmd_sub_ = create_subscription<mars_quadrotor_msgs::msg::PositionCommand>(
                pos_cmd_topic_,
                cmd_qos,
                std::bind(&MavrosOffboardBridgeNode::posCmdCallback, this, std::placeholders::_1));

        vision_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(vision_pose_topic_, 10);
        setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(setpoint_topic_, 10);

        const auto period_ms = static_cast<int>(1000.0 / std::max(1.0, timer_rate_hz_));
        timer_ = create_wall_timer(
                std::chrono::milliseconds(period_ms),
                std::bind(&MavrosOffboardBridgeNode::timerCallback, this));

        RCLCPP_INFO(
                get_logger(),
                "MAVROS offboard bridge: odom '%s' -> '%s', pos_cmd '%s' -> '%s'",
                odom_topic_.c_str(),
                vision_pose_topic_.c_str(),
                pos_cmd_topic_.c_str(),
                setpoint_topic_.c_str());
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!publish_vision_pose_) {
            return;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        vision_pose_pub_->publish(pose);
    }

    void posCmdCallback(const mars_quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
        mavros_msgs::msg::PositionTarget setpoint;
        setpoint.header = msg->header;
        setpoint.header.stamp = now();
        if (!setpoint_frame_id_.empty()) {
            setpoint.header.frame_id = setpoint_frame_id_;
        }

        // MAVROS ROS interfaces use ENU/FLU conventions and transform to PX4 NED internally.
        setpoint.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        setpoint.type_mask = 0;

        setpoint.position.x = msg->position.x;
        setpoint.position.y = msg->position.y;
        setpoint.position.z = msg->position.z;
        setpoint.velocity.x = msg->velocity.x;
        setpoint.velocity.y = msg->velocity.y;
        setpoint.velocity.z = msg->velocity.z;
        setpoint.acceleration_or_force.x = msg->acceleration.x;
        setpoint.acceleration_or_force.y = msg->acceleration.y;
        setpoint.acceleration_or_force.z = msg->acceleration.z;
        setpoint.yaw = static_cast<float>(msg->yaw);
        setpoint.yaw_rate = static_cast<float>(msg->yaw_dot);

        latest_setpoint_ = setpoint;
        latest_setpoint_time_ = now();
        have_setpoint_ = true;
    }

    void timerCallback() {
        if (!publish_setpoints_ || !have_setpoint_) {
            return;
        }

        const double age = (now() - latest_setpoint_time_).seconds();
        if (age > stale_command_timeout_s_) {
            return;
        }

        latest_setpoint_.header.stamp = now();
        setpoint_pub_->publish(latest_setpoint_);
    }

    std::string odom_topic_;
    std::string pos_cmd_topic_;
    std::string vision_pose_topic_;
    std::string setpoint_topic_;
    std::string setpoint_frame_id_;
    double timer_rate_hz_{50.0};
    double stale_command_timeout_s_{0.5};
    bool publish_vision_pose_{true};
    bool publish_setpoints_{true};

    bool have_setpoint_{false};
    rclcpp::Time latest_setpoint_time_{0, 0, RCL_ROS_TIME};
    mavros_msgs::msg::PositionTarget latest_setpoint_{};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<mars_quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_pub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MavrosOffboardBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
