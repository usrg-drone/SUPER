from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_name = LaunchConfiguration("config_name")
    use_goal_point_3d = LaunchConfiguration("use_goal_point_3d")
    start_fast_lio = LaunchConfiguration("start_fast_lio")
    fast_lio_package = LaunchConfiguration("fast_lio_package")
    fast_lio_launch = LaunchConfiguration("fast_lio_launch")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_name",
            default_value="fast_lio_mid360_ros2.yaml",
            description="SUPER config file under super_planner/config.",
        ),
        DeclareLaunchArgument(
            "use_goal_point_3d",
            default_value="true",
            description="Start the /goal_point_3d to /goal_pose adapter.",
        ),
        DeclareLaunchArgument(
            "start_fast_lio",
            default_value="false",
            description=(
                "Also include a ROS 2 FAST-LIO launch file. Keep false when "
                "FAST-LIO is already running or when using the upstream ROS 1 package."
            ),
        ),
        DeclareLaunchArgument(
            "fast_lio_package",
            default_value="fast_lio",
            description="ROS 2 FAST-LIO package name, if start_fast_lio is true.",
        ),
        DeclareLaunchArgument(
            "fast_lio_launch",
            default_value="mapping_mid360.launch.py",
            description="ROS 2 FAST-LIO launch filename, if start_fast_lio is true.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare(fast_lio_package),
                    "launch",
                    fast_lio_launch,
                ])
            ),
            condition=IfCondition(start_fast_lio),
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
                {"default_frame_id": "camera_init"},
                {"use_yaw": False},
            ],
        ),
    ])
