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
    start_livox_driver = LaunchConfiguration("start_livox_driver")
    livox_driver_package = LaunchConfiguration("livox_driver_package")
    livox_driver_launch = LaunchConfiguration("livox_driver_launch")
    start_fast_lio = LaunchConfiguration("start_fast_lio")
    fast_lio_package = LaunchConfiguration("fast_lio_package")
    fast_lio_launch = LaunchConfiguration("fast_lio_launch")
    fast_lio_config_file = LaunchConfiguration("fast_lio_config_file")
    fast_lio_rviz = LaunchConfiguration("fast_lio_rviz")

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
            "start_livox_driver",
            default_value="false",
            description="Also start livox_ros_driver2 in MID360 custom-message mode.",
        ),
        DeclareLaunchArgument(
            "livox_driver_package",
            default_value="livox_ros_driver2",
            description="Livox ROS 2 driver package name.",
        ),
        DeclareLaunchArgument(
            "livox_driver_launch",
            default_value="msg_MID360_launch.py",
            description="Livox ROS 2 MID360 custom-message launch file.",
        ),
        DeclareLaunchArgument(
            "start_fast_lio",
            default_value="true",
            description="Start the ROS 2 FAST-LIO GPU mapping launch.",
        ),
        DeclareLaunchArgument(
            "fast_lio_package",
            default_value="fast_lio",
            description="ROS 2 FAST-LIO package name.",
        ),
        DeclareLaunchArgument(
            "fast_lio_launch",
            default_value="mapping.launch.py",
            description="FAST-LIO GPU launch filename.",
        ),
        DeclareLaunchArgument(
            "fast_lio_config_file",
            default_value="mid360.yaml",
            description="FAST-LIO GPU config file under fast_lio/config.",
        ),
        DeclareLaunchArgument(
            "fast_lio_rviz",
            default_value="false",
            description="Start FAST-LIO's RViz view.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare(livox_driver_package),
                    "launch",
                    livox_driver_launch,
                ])
            ),
            condition=IfCondition(start_livox_driver),
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
            launch_arguments={
                "config_file": fast_lio_config_file,
                "rviz": fast_lio_rviz,
                "use_sim_time": "false",
            }.items(),
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
