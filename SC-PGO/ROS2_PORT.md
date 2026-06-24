# ROS2 Port Notes

This workspace now treats FAST-LIO as the ROS2 local odometry frontend and
SC-PGO as the ROS2 Scan Context pose-graph backend.

## Build

Source ROS2 and the workspace that provides `livox_ros_driver2`, then build:

```bash
source /opt/ros/humble/setup.bash
source /home/john/super_ws/install/setup.bash
colcon build --packages-select fast_lio aloam_velodyne --allow-overriding fast_lio
```

`aloam_velodyne` requires GTSAM. If CMake cannot find it, install or source a
GTSAM build that exports `GTSAMConfig.cmake` or `gtsam-config.cmake`.

## Launch

FAST-LIO:

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

SC-PGO backend:

```bash
ros2 launch aloam_velodyne fast_lio_slam.launch.py
```

## Topic Contract

FAST-LIO publishes the local, control-safe outputs:

```text
/Odometry
/cloud_registered
/path
```

SC-PGO subscribes to:

```text
/Odometry
/cloud_registered
```

and publishes global, loop-closed outputs:

```text
/slam/optimized_odom
/slam/optimized_path
/slam/optimized_map
```

Keep PX4, ROG-Map, and SUPER on the smooth local FAST-LIO outputs. Use the
SC-PGO outputs for visualization, reporting, global mapping, or a separate
`map -> camera_init` correction node.
