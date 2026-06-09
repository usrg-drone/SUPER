#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mars_quadrotor_msgs/msg/polynomial_trajectory.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace {

double evalPolynomial(const std::vector<double> &coeffs,
                      const size_t offset,
                      const size_t order,
                      const double t) {
    double value = 0.0;
    for (size_t i = 0; i <= order; ++i) {
        value = value * t + coeffs[offset + i];
    }
    return value;
}

}  // namespace

class PolyTrajVizNode final : public rclcpp::Node {
public:
    PolyTrajVizNode() : Node("poly_traj_viz_node") {
        input_topic_ = declare_parameter<std::string>("input_topic", "/planning_cmd/poly_traj");
        path_topic_ = declare_parameter<std::string>("path_topic", "/planning_cmd/poly_traj_path");
        marker_topic_ = declare_parameter<std::string>("marker_topic", "/planning_cmd/poly_traj_marker");
        output_frame_id_ = declare_parameter<std::string>("output_frame_id", "camera_init");
        sample_dt_ = declare_parameter<double>("sample_dt", 0.05);
        marker_width_ = declare_parameter<double>("marker_width", 0.08);

        const auto pub_qos = rclcpp::QoS(1).reliable().transient_local();
        path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, pub_qos);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(marker_topic_, pub_qos);

        const auto sub_qos = rclcpp::QoS(10).best_effort().keep_last(10).durability_volatile();
        traj_sub_ = create_subscription<mars_quadrotor_msgs::msg::PolynomialTrajectory>(
                input_topic_,
                sub_qos,
                std::bind(&PolyTrajVizNode::trajectoryCallback, this, std::placeholders::_1));

        RCLCPP_INFO(
                get_logger(),
                "Visualizing PolynomialTrajectory '%s' as Path '%s' and Marker '%s'",
                input_topic_.c_str(),
                path_topic_.c_str(),
                marker_topic_.c_str());
    }

private:
    void trajectoryCallback(const mars_quadrotor_msgs::msg::PolynomialTrajectory::SharedPtr msg) {
        if ((msg->type & mars_quadrotor_msgs::msg::PolynomialTrajectory::POSITION_TRAJ) == 0 ||
            msg->piece_num_pos == 0) {
            publishDeleteMarker(msg->header);
            return;
        }

        const size_t order = static_cast<size_t>(msg->order_pos);
        const size_t coeffs_per_piece = order + 1;
        const size_t piece_count = static_cast<size_t>(msg->piece_num_pos);
        const size_t required_coeffs = piece_count * coeffs_per_piece;
        if (msg->time_pos.size() < piece_count ||
            msg->coef_pos_x.size() < required_coeffs ||
            msg->coef_pos_y.size() < required_coeffs ||
            msg->coef_pos_z.size() < required_coeffs) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Skipping malformed PolynomialTrajectory: pieces=%zu order=%zu time=%zu coef=(%zu,%zu,%zu)",
                    piece_count,
                    order,
                    msg->time_pos.size(),
                    msg->coef_pos_x.size(),
                    msg->coef_pos_y.size(),
                    msg->coef_pos_z.size());
            return;
        }

        nav_msgs::msg::Path path;
        path.header = makeHeader(msg->header);

        visualization_msgs::msg::Marker marker;
        marker.header = path.header;
        marker.ns = "polynomial_trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker_width_;
        marker.color.r = 0.0f;
        marker.color.g = 0.45f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;

        const double dt = std::max(0.005, sample_dt_);
        for (size_t piece = 0; piece < piece_count; ++piece) {
            const double duration = std::max(0.0, msg->time_pos[piece]);
            const size_t offset = piece * coeffs_per_piece;
            const int samples = std::max(1, static_cast<int>(std::ceil(duration / dt)));

            for (int sample = 0; sample <= samples; ++sample) {
                if (piece > 0 && sample == 0) {
                    continue;
                }

                const double t = duration * static_cast<double>(sample) / static_cast<double>(samples);
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = evalPolynomial(msg->coef_pos_x, offset, order, t);
                pose.pose.position.y = evalPolynomial(msg->coef_pos_y, offset, order, t);
                pose.pose.position.z = evalPolynomial(msg->coef_pos_z, offset, order, t);
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);

                geometry_msgs::msg::Point point;
                point.x = pose.pose.position.x;
                point.y = pose.pose.position.y;
                point.z = pose.pose.position.z;
                marker.points.push_back(point);
            }
        }

        path_pub_->publish(path);
        marker_pub_->publish(marker);
    }

    std_msgs::msg::Header makeHeader(const std_msgs::msg::Header &input_header) {
        std_msgs::msg::Header header = input_header;
        header.stamp = now();
        if (!output_frame_id_.empty()) {
            header.frame_id = output_frame_id_;
        } else if (header.frame_id.empty()) {
            header.frame_id = "camera_init";
        }
        return header;
    }

    void publishDeleteMarker(const std_msgs::msg::Header &input_header) {
        visualization_msgs::msg::Marker marker;
        marker.header = makeHeader(input_header);
        marker.ns = "polynomial_trajectory";
        marker.id = 0;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_pub_->publish(marker);
    }

    std::string input_topic_;
    std::string path_topic_;
    std::string marker_topic_;
    std::string output_frame_id_;
    double sample_dt_{0.05};
    double marker_width_{0.08};
    rclcpp::Subscription<mars_quadrotor_msgs::msg::PolynomialTrajectory>::SharedPtr traj_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PolyTrajVizNode>());
    rclcpp::shutdown();
    return 0;
}
