from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_name = LaunchConfiguration("config_name")
    use_goal_point_3d = LaunchConfiguration("use_goal_point_3d")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_name",
            default_value="click_smooth_ros2.yaml",
            description="Planner config file under super_planner/config.",
        ),
        DeclareLaunchArgument(
            "use_goal_point_3d",
            default_value="true",
            description="Start a PointStamped-to-PoseStamped adapter for 3D goals.",
        ),
        Node(
            package="super_planner",
            executable="fsm_node",
            name="fsm_node",
            output="screen",
            parameters=[
                {"use_sim_time": False},
                {"config_name": config_name},
            ],
        ),
        Node(
            package="super_planner",
            executable="goal_point_3d_node",
            name="goal_point_3d_node",
            output="screen",
            condition=IfCondition(use_goal_point_3d),
            parameters=[
                {"input_topic": "/goal_point_3d"},
                {"output_topic": "/goal_pose"},
                {"default_frame_id": "world"},
                {"use_yaw": False},
            ],
        ),
    ])
