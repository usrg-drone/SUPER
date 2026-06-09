#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

geometry_msgs::msg::Quaternion disabledYawQuaternion() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    geometry_msgs::msg::Quaternion q;
    q.x = nan;
    q.y = nan;
    q.z = nan;
    q.w = nan;
    return q;
}

}  // namespace

class GoalPoint3DNode final : public rclcpp::Node {
public:
    GoalPoint3DNode() : Node("goal_point_3d_node") {
        const auto input_topic = declare_parameter<std::string>("input_topic", "/goal_point_3d");
        const auto input_pose_topic = declare_parameter<std::string>("input_pose_topic", "/goal_pose_2d");
        const auto output_topic = declare_parameter<std::string>("output_topic", "/goal_pose");
        default_frame_id_ = declare_parameter<std::string>("default_frame_id", "world");
        use_yaw_ = declare_parameter<bool>("use_yaw", false);
        yaw_ = declare_parameter<double>("yaw", 0.0);
        pose_fixed_z_ = declare_parameter<double>("pose_fixed_z", std::numeric_limits<double>::quiet_NaN());

        const auto qos = rclcpp::QoS(1).reliable().keep_last(1).durability_volatile();
        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(output_topic, qos);
        goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
                input_topic,
                qos,
                std::bind(&GoalPoint3DNode::goalCallback, this, std::placeholders::_1));
        pose_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                input_pose_topic,
                qos,
                std::bind(&GoalPoint3DNode::poseGoalCallback, this, std::placeholders::_1));

        RCLCPP_INFO(
                get_logger(),
                "Forwarding 3D goal points from '%s' and 2D goal poses from '%s' to SUPER goal poses on '%s'",
                input_topic.c_str(),
                input_pose_topic.c_str(),
                output_topic.c_str());
    }

private:
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) const {
        geometry_msgs::msg::PoseStamped goal;
        goal.header = msg->header;
        if (goal.header.frame_id.empty()) {
            goal.header.frame_id = default_frame_id_;
        }
        if (goal.header.stamp.sec == 0 && goal.header.stamp.nanosec == 0) {
            goal.header.stamp = now();
        }

        goal.pose.position = msg->point;
        goal.pose.orientation = use_yaw_ ? yawToQuaternion(yaw_) : disabledYawQuaternion();
        goal_pub_->publish(goal);

        RCLCPP_INFO(
                get_logger(),
                "Published 3D goal [%.3f, %.3f, %.3f] in frame '%s'",
                goal.pose.position.x,
                goal.pose.position.y,
                goal.pose.position.z,
                goal.header.frame_id.c_str());
    }

    void poseGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const {
        geometry_msgs::msg::PoseStamped goal = *msg;
        if (goal.header.frame_id.empty()) {
            goal.header.frame_id = default_frame_id_;
        }
        if (goal.header.stamp.sec == 0 && goal.header.stamp.nanosec == 0) {
            goal.header.stamp = now();
        }

        if (std::isfinite(pose_fixed_z_)) {
            goal.pose.position.z = pose_fixed_z_;
        }
        goal_pub_->publish(goal);

        RCLCPP_INFO(
                get_logger(),
                "Published 2D goal as [%.3f, %.3f, %.3f] in frame '%s'",
                goal.pose.position.x,
                goal.pose.position.y,
                goal.pose.position.z,
                goal.header.frame_id.c_str());
    }

    std::string default_frame_id_;
    bool use_yaw_{false};
    double yaw_{0.0};
    double pose_fixed_z_{std::numeric_limits<double>::quiet_NaN()};
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_goal_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPoint3DNode>());
    rclcpp::shutdown();
    return 0;
}
