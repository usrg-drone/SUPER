#ifndef MISSION_PLANNER_WAYPOINT_PLANNER
#define MISSION_PLANNER_WAYPOINT_PLANNER

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "vector"
#include "string"
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <thread>
#include "config.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "Eigen/Core"
#include "utils/eigen_alias.hpp"
#include "utils/color_msg_utils.hpp"

namespace mission_planner {
    using namespace std;
    using namespace super_utils;

    class WaypointPlanner {

    private:
        MissionConfig cfg_;

        rclcpp::Node::SharedPtr nh_;

        Eigen::Vector3d cur_position;
        int waypoint_counter{0};
        bool had_odom{false};
        bool triggered{false};
        bool new_goal{true};
        bool mission_completed{false};
        bool suppress_next_goal_echo{false};
        double odom_rcv_time{0};
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mkr_pub_;

        geometry_msgs::msg::PoseStamped last_mission_goal_;
        mavros_msgs::msg::State mavros_state_;
        bool had_mavros_state_{false};
        double mavros_state_rcv_time_{0};

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr click_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
        rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr mavros_sub_;
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        rclcpp::TimerBase::SharedPtr goal_pub_timer_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restart_srv_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_active_srv_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr hold_position_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr publish_setpoints_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr offboard_mode_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr planner_enabled_client_;

        rclcpp::CallbackGroup::SharedPtr click_cb_group_, mavros_cb_group_, mavros_state_cb_group_;
        rclcpp::CallbackGroup::SharedPtr odom_cb_group_, goal_pose_cb_group_, goal_pub_timer_cb_group_, service_cb_group_;

        double system_start_time{0};
        bool trigger_once{false};

        static string ResolvePackageFile(const string &subdir, const string &name) {
            if (!name.empty() && name.front() == '/') {
                return name;
            }
            const auto share_dir = ament_index_cpp::get_package_share_directory("mission_planner");
            return share_dir + "/" + subdir + "/" + name;
        }

        void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            had_odom = true;
            odom_rcv_time = nh_->get_clock()->now().seconds();
            cur_position = Eigen::Vector3d(msg->pose.pose.position.x,
                                           msg->pose.pose.position.y,
                                           msg->pose.pose.position.z);
        }

        void MavrosStateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
            mavros_state_ = *msg;
            had_mavros_state_ = true;
            mavros_state_rcv_time_ = nh_->get_clock()->now().seconds();
        }

        bool CloseToPoint(Vec3f &position) {
            return (position - cur_position).norm() < cfg_.switch_dis;
        }

        bool CloseToPoint(Vec3f &position, int id) {
            return (position - cur_position).norm() < cfg_.switch_dis_vec[id];
        }

        void RvizClickCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            if (!had_odom) {
                return;
            }
            StartMission(false, false);
            cout << YELLOW <<
                 " -- [MISSION] Rviz triggered." << RESET << endl;
        }

        bool SameGoalAsLastMissionGoal(const geometry_msgs::msg::PoseStamped &msg) const {
            const auto &a = msg.pose.position;
            const auto &b = last_mission_goal_.pose.position;
            return msg.header.frame_id == last_mission_goal_.header.frame_id
                   && std::abs(a.x - b.x) < 1.0e-4
                   && std::abs(a.y - b.y) < 1.0e-4
                   && std::abs(a.z - b.z) < 1.0e-4;
        }

        void GoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            if (suppress_next_goal_echo && SameGoalAsLastMissionGoal(*msg)) {
                suppress_next_goal_echo = false;
                return;
            }
            if (!triggered) {
                return;
            }
            triggered = false;
            new_goal = true;
            cout << YELLOW << " -- [MISSION] External goal_pose detected; mission paused at waypoint "
                 << waypoint_counter << "." << RESET << endl;
        }

        void MavrosRcCallback(const mavros_msgs::msg::RCIn::SharedPtr msg) {
            static int last_ch_10 = 1000;
            if (!had_odom) {
                return;
            }
            if (msg->channels.size() <= 9) {
                RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                                     " -- [MISSION] RC message has fewer than 10 channels.");
                return;
            }
            int ch_10 = msg->channels[9];
            if (last_ch_10 > 1500 && ch_10 < 1500) {
                StartMission(false, false);
                cout << YELLOW << " -- [MISSION] Mavros triggered." << RESET << endl;
            }
            last_ch_10 = ch_10;
        }

        void GoalPubTimerCallback() {
            static int last_mkr_sub_num = mkr_pub_->get_subscription_count();
            int cur_mkr_sub_num = mkr_pub_->get_subscription_count();
            if (cur_mkr_sub_num != last_mkr_sub_num && cur_mkr_sub_num > 0) {
                visualizeMission();
            }
            last_mkr_sub_num = cur_mkr_sub_num;
            if (cfg_.start_trigger_type == 2 && !trigger_once) {
                const auto cur_t = nh_->now().seconds();
                if (cur_t - system_start_time < cfg_.start_program_delay) {
                    cout << "Cur t = " << cur_t << " start t = " << system_start_time << endl;
                    return;
                } else {
                    cout << YELLOW << " -- [MISSION] System start delay passed." << RESET << endl;
                    StartMission(false, true);
                }
            }
            if (!triggered) {
                return;
            }

            double cur_t = nh_->now().seconds();
            if (cur_t - odom_rcv_time > cfg_.odom_timeout) {
                static double last_print_t = nh_->now().seconds();
                if (cur_t - last_print_t > 1.0) {
                    last_print_t = cur_t;
                    cout << YELLOW << " -- [MISSION] Odom Timeout!" << RESET << endl;
                }
                return;
            }


            if (CloseToPoint(cfg_.waypoints[waypoint_counter], waypoint_counter)) {
                cout << RED << " -- [MISSION] Close to goal {}, switch to next." << RESET << endl;
                waypoint_counter++;
                new_goal = true;
                if (waypoint_counter >= cfg_.waypoints.size()) {
                    // 结束，停止发布。
                    waypoint_counter = cfg_.waypoints.size() - 1;
                    triggered = false;
                    new_goal = false;
                    mission_completed = true;
                }
            }

            static double last_pub_time = 0;

            if (new_goal || cur_t - last_pub_time > cfg_.publish_dt) {
                new_goal = false;
                geometry_msgs::msg::PoseStamped goal;
                goal.pose.position.x = cfg_.waypoints[waypoint_counter].x();
                goal.pose.position.y = cfg_.waypoints[waypoint_counter].y();
                goal.pose.position.z = cfg_.waypoints[waypoint_counter].z();
                goal.pose.orientation.w = 1;
                goal.header.frame_id = cfg_.goal_frame_id;
                goal.header.stamp = nh_->now();
                last_mission_goal_ = goal;
                suppress_next_goal_echo = true;
                last_pub_time = cur_t;
                goal_pub_->publish(goal);
                cout << YELLOW << " -- [MISSION] Pub goal to " << cfg_.waypoints[waypoint_counter].transpose()
                     << RESET << endl;
                cout << YELLOW << "\t cur odom dis = "
                     << (cfg_.waypoints[waypoint_counter] - cur_position).norm() << endl;
            }


        }

        bool CheckStartSafety(std::string &message) const {
            if (cfg_.waypoints.empty()) {
                message = "No waypoints loaded.";
                return false;
            }
            if (!had_odom) {
                message = "Odometry has not been received.";
                return false;
            }
            const double cur_t = nh_->now().seconds();
            if (cur_t - odom_rcv_time > cfg_.odom_timeout) {
                message = "Odometry is stale.";
                return false;
            }
            if (cfg_.require_mavros_armed) {
                if (!had_mavros_state_) {
                    message = "MAVROS state has not been received.";
                    return false;
                }
                if (cur_t - mavros_state_rcv_time_ > 1.0) {
                    message = "MAVROS state is stale.";
                    return false;
                }
                if (!mavros_state_.connected) {
                    message = "MAVROS is not connected.";
                    return false;
                }
                if (!mavros_state_.armed) {
                    message = "Vehicle must be armed before mission start.";
                    return false;
                }
            }
            return true;
        }

        bool CallBoolService(
                const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr &client,
                bool value,
                const std::string &name,
                std::string &message) {
            if (!client->wait_for_service(std::chrono::milliseconds(500))) {
                message = name + " service is unavailable.";
                return false;
            }
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = value;
            auto future = client->async_send_request(request);
            const auto result = future.wait_for(std::chrono::seconds(2));
            if (result != std::future_status::ready) {
                message = name + " service timed out.";
                return false;
            }
            auto response = future.get();
            if (!response->success) {
                message = name + " rejected request: " + response->message;
                return false;
            }
            return true;
        }

        bool PrepareOffboardForMission(std::string &message) {
            if (!cfg_.prepare_offboard_on_start) {
                return true;
            }
            if (!CallBoolService(hold_position_client_, true, "hold_position", message)) {
                return false;
            }
            if (!CallBoolService(publish_setpoints_client_, true, "publish_setpoints", message)) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (!CallBoolService(offboard_mode_client_, true, "offboard_mode", message)) {
                return false;
            }
            if (!CallBoolService(planner_enabled_client_, true, "planner_enabled", message)) {
                return false;
            }
            if (!CallBoolService(hold_position_client_, false, "hold_position", message)) {
                return false;
            }
            return true;
        }

        bool StartMission(bool reset, bool from_auto_trigger, std::string *message_out = nullptr) {
            std::string message;
            if (!CheckStartSafety(message)) {
                if (message_out != nullptr) {
                    *message_out = message;
                }
                RCLCPP_WARN(nh_->get_logger(), " -- [MISSION] Start rejected: %s", message.c_str());
                return false;
            }
            if (!from_auto_trigger && !PrepareOffboardForMission(message)) {
                if (message_out != nullptr) {
                    *message_out = message;
                }
                RCLCPP_WARN(nh_->get_logger(), " -- [MISSION] Start rejected: %s", message.c_str());
                return false;
            }
            if (reset || mission_completed) {
                waypoint_counter = 0;
                mission_completed = false;
            }
            triggered = true;
            trigger_once = true;
            new_goal = true;
            if (message_out != nullptr) {
                *message_out = reset ? "Mission restarted." : "Mission started.";
            }
            return true;
        }

        bool StopMission(bool hold, std::string *message_out = nullptr) {
            triggered = false;
            new_goal = true;
            std::string message;
            if (hold && cfg_.prepare_offboard_on_start) {
                if (!CallBoolService(planner_enabled_client_, false, "planner_enabled", message)) {
                    if (message_out != nullptr) {
                        *message_out = "Mission stopped, but planner cleanup failed: " + message;
                    }
                    return false;
                }
                if (!CallBoolService(hold_position_client_, true, "hold_position", message)) {
                    if (message_out != nullptr) {
                        *message_out = "Mission stopped, but hold cleanup failed: " + message;
                    }
                    return false;
                }
                CallBoolService(publish_setpoints_client_, true, "publish_setpoints", message);
            }
            if (message_out != nullptr) {
                *message_out = "Mission stopped.";
            }
            return true;
        }

    public:
        WaypointPlanner() {};

        WaypointPlanner(rclcpp::Node::SharedPtr nh) {
            nh_ = nh;
            /* Load configuration parameters */
            std::string dft_cfg_name = "waypoint.yaml";
            std::string dft_data_name = "benchmark.txt";
            std::string cfg_path, cfg_name;
            std::string data_path, data_name;

            nh_->declare_parameter<std::string>("config_name", dft_cfg_name);
            nh_->declare_parameter<std::string>("data_name", dft_data_name);


           if(nh_->get_parameter("config_name", cfg_name)){
                cfg_path = ResolvePackageFile("config", cfg_name);
                RCLCPP_WARN(nh_->get_logger(), " -- [MissionPlanner] Load config by file name: %s", cfg_path.c_str());
            }

            if(nh_->get_parameter("data_name", data_name)){
                data_path = ResolvePackageFile("data", data_name);
                RCLCPP_WARN(nh_->get_logger(), " -- [MissionPlanner] Load data by file name: %s", data_path.c_str());
            }

            cfg_ = MissionConfig(cfg_path);

            cfg_.LoadWaypoint(data_path);
            if (cfg_.waypoints.empty()) {
                throw std::runtime_error("Mission has no waypoints: " + data_path);
            }

            const rclcpp::QoS qos(rclcpp::QoS(100)
                                          .best_effort()
                                          .keep_last(100)
                                          .durability_volatile());

            rclcpp::SubscriptionOptions so;
            odom_cb_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            so.callback_group = odom_cb_group_;
            odom_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(cfg_.odom_topic, qos,
                                                                          std::bind(&WaypointPlanner::OdomCallback,
                                                                                    this, std::placeholders::_1), so);

            mavros_state_cb_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            so.callback_group = mavros_state_cb_group_;
            mavros_state_sub_ = nh_->create_subscription<mavros_msgs::msg::State>(
                    "/mavros/state", qos,
                    std::bind(&WaypointPlanner::MavrosStateCallback, this, std::placeholders::_1), so);

            goal_pub_timer_cb_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            goal_pub_timer_ = nh_->create_wall_timer(
                    std::chrono::milliseconds(10), // 0.001秒，即1毫秒
                    std::bind(&WaypointPlanner::GoalPubTimerCallback, this),
                    goal_pub_timer_cb_group_
            );

            goal_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>(cfg_.goal_pub_topic, qos);

            goal_pose_cb_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            so.callback_group = goal_pose_cb_group_;
            goal_pose_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseStamped>(
                    cfg_.goal_pub_topic, qos,
                    std::bind(&WaypointPlanner::GoalPoseCallback, this, std::placeholders::_1), so);

            click_cb_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            so.callback_group = click_cb_group_;
            click_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseStamped>("/goal", qos, std::bind(
                    &WaypointPlanner::RvizClickCallback, this, std::placeholders::_1), so);
            mavros_cb_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            so.callback_group = mavros_cb_group_;
            mavros_sub_ = nh_->create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in", qos, std::bind(
                    &WaypointPlanner::MavrosRcCallback, this, std::placeholders::_1), so);

            mkr_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("/mission/mkr", qos);
            service_cb_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            start_srv_ = nh_->create_service<std_srvs::srv::Trigger>(
                    "/mission/start",
                    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                        response->success = StartMission(false, false, &response->message);
                    },
                    rmw_qos_profile_services_default,
                    service_cb_group_);
            restart_srv_ = nh_->create_service<std_srvs::srv::Trigger>(
                    "/mission/restart",
                    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                        response->success = StartMission(true, false, &response->message);
                    },
                    rmw_qos_profile_services_default,
                    service_cb_group_);
            stop_srv_ = nh_->create_service<std_srvs::srv::Trigger>(
                    "/mission/stop",
                    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                        response->success = StopMission(true, &response->message);
                    },
                    rmw_qos_profile_services_default,
                    service_cb_group_);
            set_active_srv_ = nh_->create_service<std_srvs::srv::SetBool>(
                    "/mission/set_active",
                    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                        response->success = request->data
                                            ? StartMission(false, false, &response->message)
                                            : StopMission(true, &response->message);
                    },
                    rmw_qos_profile_services_default,
                    service_cb_group_);
            hold_position_client_ = nh_->create_client<std_srvs::srv::SetBool>(
                    "/super_px4_mavros_offboard_bridge/set_hold_position", rmw_qos_profile_services_default,
                    service_cb_group_);
            publish_setpoints_client_ = nh_->create_client<std_srvs::srv::SetBool>(
                    "/super_px4_mavros_offboard_bridge/set_publish_setpoints", rmw_qos_profile_services_default,
                    service_cb_group_);
            offboard_mode_client_ = nh_->create_client<std_srvs::srv::SetBool>(
                    "/super_px4_mavros_offboard_bridge/set_offboard_mode", rmw_qos_profile_services_default,
                    service_cb_group_);
            planner_enabled_client_ = nh_->create_client<std_srvs::srv::SetBool>(
                    "/super_px4_mavros_offboard_bridge/set_planner_enabled", rmw_qos_profile_services_default,
                    service_cb_group_);
            system_start_time = nh_->now().seconds();
        }

        void visualizeMission() {
            visualization_msgs::msg::MarkerArray mkr_arr;
            addPathToMarkerArray(mkr_arr, cfg_.waypoints, Color::SteelBlue(), "waypoints", 0.5, 0.3);

            for (int i = 0; i < cfg_.switch_dis_vec.size(); i++) {
                Color c = Color::Orange();
                c.a = 0.2;
                visualizePoint(mkr_arr, cfg_.waypoints[i], c,
                               "switch_dis", cfg_.switch_dis_vec[i] * 2, i);
                visualizeText(mkr_arr, "id", to_string(1 + i), cfg_.waypoints[i],
                              Color::Black(), 3, i);
            }
            mkr_pub_->publish(mkr_arr);
        }

        static void visualizePoint(visualization_msgs::msg::MarkerArray &mkr_arr,
                                   const Vec3f &pt,
                                   Color color = Color::Pink(),
                                   std::string ns = "pt",
                                   double size = 0.1, int id = -1,
                                   const bool &print_ns = true) {
            visualization_msgs::msg::Marker marker_ball;
            static int cnt = 0;
            Vec3f cur_pos = pt;
            if (isnan(pt.x()) || isnan(pt.y()) || isnan(pt.z())) {
                return;
            }
            marker_ball.header.frame_id = "world";
            auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
            marker_ball.header.stamp = clock->now();
            marker_ball.ns = ns.c_str();
            marker_ball.id = id >= 0 ? id : cnt++;
            marker_ball.action = visualization_msgs::msg::Marker::ADD;
            marker_ball.pose.orientation.w = 1.0;
            marker_ball.type = visualization_msgs::msg::Marker::SPHERE;
            marker_ball.scale.x = size;
            marker_ball.scale.y = size;
            marker_ball.scale.z = size;
            marker_ball.color = color;

            geometry_msgs::msg::Point p;
            p.x = cur_pos.x();
            p.y = cur_pos.y();
            p.z = cur_pos.z();

            marker_ball.pose.position = p;
            mkr_arr.markers.push_back(marker_ball);

            // add test
            if (print_ns) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "world";
                auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
                marker.header.stamp = clock->now();
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.orientation.w = 1.0;
                marker.ns = ns + "_text";
                if (id >= 0) {
                    marker.id = id;
                } else {
                    static int id = 0;
                    marker.id = id++;
                }
                marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                marker.scale.z = 0.6;
                marker.color = color;
                marker.text = ns;
                marker.pose.position.x = cur_pos.x();
                marker.pose.position.y = cur_pos.y();
                marker.pose.position.z = cur_pos.z() + 0.5;
                marker.pose.orientation.w = 1.0;
                mkr_arr.markers.push_back(marker);
            }
        }

        static void visualizeText(visualization_msgs::msg::MarkerArray &mkr_arr,
                                  const std::string &ns,
                                  const std::string &text,
                                  const Vec3f &position,
                                  const Color &c = Color::White(),
                                  const double &size = 0.6,
                                  const int &id = -1) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
            marker.header.stamp = clock->now();
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.ns = ns.c_str();
            if (id >= 0) {
                marker.id = id;
            } else {
                static int id = 0;
                marker.id = id++;
            }
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.scale.z = size;
            marker.color = c;
            marker.text = text;
            marker.pose.position.x = position.x();
            marker.pose.position.y = position.y();
            marker.pose.position.z = position.z();
            marker.pose.orientation.w = 1.0;
            mkr_arr.markers.push_back(marker);
        };

        static void addPathToMarkerArray(visualization_msgs::msg::MarkerArray &mkr_ary,
                                         const vec_E<Vec3f> &path,
                                         Color color,
                                         string ns,
                                         double pt_size,
                                         double line_size) {
            visualization_msgs::msg::Marker line_list;
            if (path.size() <= 0) {
                std::cout << YELLOW << "Try to publish empty path, return.\n" << RESET << std::endl;
                return;
            }
            Vec3f cur_pt = path[0], last_pt;
            static int point_id = 0;
            static int line_cnt = 0;
            auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
            for (size_t i = 0; i < path.size(); i++) {
                last_pt = cur_pt;
                cur_pt = path[i];

                /* Publish point */
                visualization_msgs::msg::Marker point;
                point.header.frame_id = "world";
                point.header.stamp = clock->now();
                point.ns = ns.c_str();
                point.id = point_id++;
                point.action = visualization_msgs::msg::Marker::ADD;
                point.pose.orientation.w = 1.0;
                point.type = visualization_msgs::msg::Marker::SPHERE;
                // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                point.scale.x = pt_size;
                point.scale.y = pt_size;
                point.scale.z = pt_size;
                // Line list is blue
                point.color = color;
                point.color.a = 1.0;
                // Create the vertices for the points and lines
                geometry_msgs::msg::Point p;
                p.x = cur_pt.x();
                p.y = cur_pt.y();
                p.z = cur_pt.z();
                point.pose.position = p;
                mkr_ary.markers.push_back(point);
                /* publish lines */
                if (i > 0) {
                    geometry_msgs::msg::Point p;
                    // publish lines
                    visualization_msgs::msg::Marker line_list;
                    line_list.header.frame_id = "world";
                    line_list.header.stamp = clock->now();
                    line_list.ns = ns + "_line";
                    line_list.id = line_cnt++;
                    line_list.action = visualization_msgs::msg::Marker::ADD;
                    line_list.pose.orientation.w = 1.0;
                    line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
                    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                    line_list.scale.x = line_size;
                    // Line list is blue
                    line_list.color = color;
                    // Create the vertices for the points and lines

                    p.x = last_pt.x();
                    p.y = last_pt.y();
                    p.z = last_pt.z();
                    // The line list needs two points for each line
                    line_list.points.push_back(p);
                    p.x = cur_pt.x();
                    p.y = cur_pt.y();
                    p.z = cur_pt.z();
                    // The line list needs
                    line_list.points.push_back(p);

                    mkr_ary.markers.push_back(line_list);
                }
            }
        }

    };
}
#endif //MISSION_PLANNER_WAYPOINT_PLANNER
