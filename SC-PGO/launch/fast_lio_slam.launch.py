from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    gtsam_lib_dir = LaunchConfiguration("gtsam_lib_dir")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("aloam_velodyne"),
                "config",
                "fast_lio_slam.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "gtsam_lib_dir",
            default_value=PathJoinSubstitution([
                FindPackageShare("aloam_velodyne"),
                "..",
                "..",
                "..",
                "gtsam",
                "lib",
            ]),
        ),
        SetEnvironmentVariable(
            name="LD_LIBRARY_PATH",
            value=[gtsam_lib_dir, ":", EnvironmentVariable("LD_LIBRARY_PATH", default_value="")],
        ),
        Node(
            package="aloam_velodyne",
            executable="alaserPGO",
            name="fast_lio_slam_pgo",
            output="screen",
            parameters=[params_file],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
