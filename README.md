# SUPER MID360 ROS 2 Stack

This branch packages a ROS 2 Humble runtime stack for a Jetson Orin NX 16 GB using a Livox MID360 lidar:

- `super_planner`: SUPER planner, ROS 2 build, no simulation packages.
- `mars_quadrotor_msgs`: extracted message definitions required by SUPER.
- `rog_map`: local map backend used by SUPER.
- `FAST_LIO_GPU`: CUDA-enabled FAST-LIO for lidar-inertial odometry.
- `SC-PGO`: GTSAM/Scan Context backend for FAST-LIO SLAM loop closure and optimized maps.
- `super_px4_mavros_offboard_bridge`: PX4/MAVROS offboard bridge for external vision, arming/mode requests, takeoff hover, hold, and SUPER setpoints.
- `Livox-SDK2`: local SDK source used by `livox_ros_driver2`.
- `livox_ros_driver2`: ROS 2 Livox driver for MID360 custom messages.
- `scripts/`: build/setup helpers and a `smug` tmux runtime configuration.

The intended runtime data flow is:

```text
MID360 -> livox_ros_driver2 -> FAST_LIO_GPU -> /cloud_registered + /Odometry -> SUPER
                                                        |
                                                        +-> PX4/MAVROS bridge -> /mavros/vision_pose/pose
                                                        |
                                                        v
                                                     /cloud_registered_body + /Odometry
                                                        -> SC-PGO -> /slam/optimized_path + /slam/optimized_map

SUPER -> /planning/pos_cmd -> PX4/MAVROS bridge -> /mavros/setpoint_raw/local
```

SUPER's planner goal topic is `/goal_pose`. The `/goal_point_3d` topic is only an optional convenience input that is converted into `/goal_pose` by `goal_point_3d_node`.

## Platform

Tested target:

- Jetson Orin NX 16 GB
- ROS 2 Humble
- CUDA installed at `/usr/local/cuda`
- Livox MID360

Required system tools:

```bash
sudo apt install -y git cmake python3-colcon-common-extensions tmux
```

ROS 2 and CUDA must already be installed. The setup scripts intentionally do not install ROS/CUDA for you.

## Fresh Clone On Another Orin

Create a workspace and clone this branch into `src`:

```bash
mkdir -p ~/super_ws
cd ~/super_ws
git clone -b jkp https://github.com/usrg-drone/SUPER.git src
```

Run the fresh setup script:

```bash
cd ~/super_ws/src
./scripts/setup_fresh_orin.sh
```

The script will:

- build `Livox-SDK2` locally
- prepare `livox_ros_driver2` for ROS 2
- configure `livox_ros_driver2` to use the local `Livox-SDK2` at a portable relative path (no hardcoded workspace path required)
- install `ros-humble-rmw-zenoh-cpp` if not already present (controlled by `USE_ZENOH`, default `1`)
- install `ros-humble-foxglove-bridge` for Lichtblick/Foxglove WebSocket visualization
- install GTSAM source-build dependencies: Eigen, Boost, and TBB development packages
- build GTSAM 4.2.0 from official source into `install/gtsam` for the SC-PGO pose-graph backend
- build `mars_quadrotor_msgs`, `rog_map`, `super_planner`, `super_px4_mavros_offboard_bridge`, and `livox_ros_driver2`
- build `SC-PGO` as the `aloam_velodyne` package
- build `FAST_LIO_GPU` with CUDA enabled for Orin architecture `87`
- verify the installed ROS executables

If your workspace is not `~/super_ws`, pass `WORKSPACE`:

```bash
WORKSPACE=/home/jetson/super_ws ./scripts/setup_fresh_orin.sh
```

To skip the Zenoh install:

```bash
USE_ZENOH=0 ./scripts/setup_fresh_orin.sh
```

## MID360 Network Config

Before running live hardware, edit the MID360 config:

```bash
nano ~/super_ws/src/livox_ros_driver2/config/MID360_config.json
```

Check these fields:

- `host_net_info`: IP address of the Orin network interface connected to the MID360.
- `lidar_configs[0].ip`: IP address of the MID360.

The checked-in default uses:

```text
host:  192.168.1.5
lidar: 192.168.1.12
```

MID360 units commonly use a default IP address derived from the last two digits of the serial number:

```text
192.168.1.1XX
```

Here `XX` is the last two digits of the MID360 serial number. For example, if the serial number ends in `12`, try:

```text
192.168.1.112
```

Set `lidar_configs[0].ip` to that address if your unit is still using its default network configuration.

Make sure your Ethernet interface is on the same subnet.

## Running The Stack

Source the workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/super_ws/install/setup.bash
source ~/super_ws/install/fast_lio/share/fast_lio/local_setup.bash
```

`fast_lio` is built via cmake rather than colcon, so it is not automatically chained by `install/setup.bash`. The third line registers it with the ROS 2 package index.

Start the tmux/smug runtime:

```bash
smug start superplanner
```

To start the same MID360 stack with the waypoint mission pane enabled:

```bash
smug start superplanner_mission
```

Tab-completion for `smug` project names is installed automatically by `setup_fresh_orin.sh`. Restart your shell (or `source /etc/bash_completion.d/smug`) if completion is not active yet.

Stop it:

```bash
smug stop superplanner
```

The setup script installs `smug` to `/usr/local/bin/` and writes rendered configs to `~/.config/smug/superplanner.yml` and `~/.config/smug/superplanner_mission.yml`. If you move the workspace, re-run `setup_fresh_orin.sh` (or edit those files directly) to update the path.

The wrapper scripts remain available if you need to override RViz options at start time:

```bash
ENABLE_RVIZ=1 RVIZ_CONFIG=~/super_ws/src/super_planner/rviz/mid360_goal_path.rviz \
  ./scripts/start_super_mid360_tmux.sh
```

The tmux session starts separate windows/panes for:

- Zenoh RMW daemon, if configured in `scripts/super_mid360.smug.yml`
- `livox_ros_driver2_node`
- `fastlio_mapping`
- FAST-LIO path publisher
- SC-PGO FAST_LIO_SLAM backend
- Foxglove bridge on `ws://<robot-ip>:8765` for Lichtblick/Foxglove
- PX4/MAVROS offboard bridge
- Optional waypoint mission node in the `superplanner_mission` config
- SUPER `fsm_node`
- 3D goal adapter
- trajectory visualization helper
- topic monitors
- RViz, if enabled

The `smug` binary is vendored at:

```bash
scripts/bin/smug
```

No system-wide `smug` install is required.

## Frames And TF

The stack is configured around FAST-LIO's default world frame:

```text
camera_init
```

Important frames/topics:

- `livox_frame`: raw Livox driver frame.
- `camera_init`: FAST-LIO odometry/map frame and SUPER visualization frame.
- `world`: convenience frame used by some visualization tools.

FAST-LIO publishes odometry on `/Odometry` in the `camera_init` frame. SUPER is configured to consume `/Odometry` and `/cloud_registered`, and its visualization frame is also `camera_init`.

The tmux config starts a static identity transform:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --qx 0 --qy 0 --qz 0 --qw 1 \
  --frame-id camera_init \
  --child-frame-id world
```

This makes `world` coincide with `camera_init` for RViz/tools that expect a `world` frame. If your downstream controller or visualization expects the opposite direction, adjust the static TF in `scripts/super_mid360.smug.yml`.

Useful TF checks:

```bash
ros2 run tf2_ros tf2_echo camera_init world
ros2 run tf2_tools view_frames
```

## RViz

RViz is enabled by default in the rendered smug config. To change the setting, edit `~/.config/smug/superplanner.yml` and set `ENABLE_RVIZ` in the `env` block, or re-render using the wrapper script:

```bash
ENABLE_RVIZ=1 RVIZ_CONFIG=~/super_ws/src/super_planner/rviz/mid360_goal_path.rviz \
  ./scripts/start_super_mid360_tmux.sh

ENABLE_RVIZ=0 ./scripts/start_super_mid360_tmux.sh
```

## Lichtblick / Foxglove

The smug runtime starts `foxglove_bridge` on port `8765`:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0 port:=8765
```

From Lichtblick, add a Foxglove WebSocket connection:

```text
ws://<robot-ip>:8765
```

Useful topics to add in Lichtblick include `/cloud_registered`, `/Odometry`, `/fastlio/path`, `/slam/optimized_map`, `/slam/optimized_path`, `/slam/optimized_odom`, and the SUPER `/planning_cmd/*` visualization topics.

SC-PGO consumes `/cloud_registered_body`, not `/cloud_registered`. FAST-LIO publishes `/cloud_registered` already transformed into `camera_init`; the pose-graph backend needs the local/body-frame scan so it can apply the optimized pose exactly once when building `/slam/optimized_map`.

## PX4 / MAVROS Offboard Bridge

The `super_px4_mavros_offboard_bridge` package is started by the smug session in the `px4-bridge` window. It uses standard MAVROS ROS 2 topics and services:

```text
/Odometry                    -> /mavros/vision_pose/pose
/planning/pos_cmd            -> /mavros/setpoint_raw/local
/mavros/state                -> bridge state monitor
/mavros/cmd/arming           <- bridge arm/disarm requests
/mavros/set_mode             <- bridge OFFBOARD requests
```

The external vision relay publishes FAST-LIO odometry as a pose stream for PX4. If the SLAM/backend pose has a large discontinuity, the bridge updates an internal offset so the pose sent to PX4 remains continuous instead of jumping with the corrected map.

The bridge exposes `std_srvs/srv/SetBool` services for field control:

```bash
ros2 service call /super_px4_mavros_offboard_bridge/set_arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /super_px4_mavros_offboard_bridge/set_offboard_mode std_srvs/srv/SetBool "{data: true}"
ros2 service call /super_px4_mavros_offboard_bridge/set_planner_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /super_px4_mavros_offboard_bridge/set_hold_position std_srvs/srv/SetBool "{data: true}"
```

It also keeps live ROS parameters for tuning and compatibility:

```bash
ros2 param set /super_px4_mavros_offboard_bridge arm true
ros2 param set /super_px4_mavros_offboard_bridge offboard_mode true
ros2 param set /super_px4_mavros_offboard_bridge planner_enabled true
ros2 param set /super_px4_mavros_offboard_bridge hold_position true
ros2 param set /super_px4_mavros_offboard_bridge takeoff_altitude 1.5
```

Typical activation flow:

1. Confirm `/Odometry` and MAVROS are healthy.
2. Set `arm:=true`.
3. Set `offboard_mode:=true`; the bridge streams a hover setpoint at the current ENU `x/y` and `takeoff_altitude`.
4. Set `planner_enabled:=true` when SUPER should take over with `/planning/pos_cmd`.
5. Set `hold_position:=true` any time you want to latch the current continuous pose and ignore planner setpoints.

`/planning_cmd/poly_traj` is for visualization or downstream controllers that consume full polynomial segments. For MAVROS offboard setpoints, use `/planning/pos_cmd`.

## SLAM Backend

SC-PGO is launched as the `aloam_velodyne` executable `alaserPGO`:

```bash
ros2 launch aloam_velodyne fast_lio_slam.launch.py \
  params_file:=~/super_ws/install/aloam_velodyne/share/aloam_velodyne/config/fast_lio_slam.yaml
```

The launch file prepends `install/gtsam/lib` to `LD_LIBRARY_PATH` and respawns the backend if it exits. The executable also has an install RPATH pointing at the workspace GTSAM install, so `libgtsam.so` and `libmetis-gtsam.so` should resolve without setting global linker paths.

Topic roles:

```text
/cloud_registered       FAST-LIO world-frame registered scan, for SUPER/RViz
/cloud_registered_body  FAST-LIO body-frame local scan, for SC-PGO
/Odometry               FAST-LIO odometry in camera_init
/Laser_map              FAST-LIO frontend accumulated map
/slam/optimized_odom    SC-PGO latest optimized pose
/slam/optimized_path    SC-PGO optimized trajectory
/slam/optimized_map     SC-PGO optimized keyframe map
```

If `/slam/optimized_map` appears tilted, doubled, or badly overlapping, first confirm `cloud_topic` in `fast_lio_slam.yaml` is `/cloud_registered_body`. Feeding `/cloud_registered` into SC-PGO double-applies the FAST-LIO pose because that topic is already in `camera_init`.

## Sending Goals

SUPER consumes goals from `/goal_pose` as `geometry_msgs/msg/PoseStamped`. If your autonomy code can publish `PoseStamped`, publish directly to `/goal_pose`; you do not need the `/goal_point_3d` adapter.

Direct 3D goal example:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'camera_init'}, pose: {position: {x: 5.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}"
```

Use `frame_id: camera_init` unless you intentionally transform goals from another frame.

The `/goal_point_3d` topic is optional. It exists for simple tools/scripts that only want to send a point and let `goal_point_3d_node` fill in the pose message.

Optional point-input example:

```bash
ros2 topic pub --once /goal_point_3d geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'camera_init'}, point: {x: 5.0, y: 0.0, z: 1.5}}"
```

The adapter republishes this as `/goal_pose`, which SUPER consumes.

A 2D pose-style input is also wired in the tmux config through `/goal_pose_2d`, depending on the current node parameters in `scripts/super_mid360.smug.yml`.

The MID360 config preserves incoming 3D goal height:

```text
super_planner/config/fast_lio_mid360_ros2.yaml
```

The parameter is set below `-5`:

```yaml
fsm:
  click_height: -10.0
```

Set `click_height` to a positive flight height, such as `1.5`, only if you want every incoming goal to be flattened to that fixed z value.

## Rebuilding FAST_LIO_GPU Only

If you changed FAST_LIO_GPU or CUDA-related files:

```bash
cd ~/super_ws/src
./scripts/build_fast_lio_gpu_orin.sh
```

Useful overrides:

```bash
CUDA_ARCH=87 JOBS=1 WORKSPACE=~/super_ws ./scripts/build_fast_lio_gpu_orin.sh
```

`JOBS=1` is intentional for Orin stability. The FAST-LIO GPU build is memory-heavy.

## Launch File Alternative

A ROS launch file is still available:

```bash
ros2 launch super_planner fast_lio_mid360.launch.py start_livox_driver:=true
```

The tmux/smug workflow is preferred for field work because each component gets its own pane and can be restarted/debugged independently.

## Important Topics

Inputs:

```text
/livox/lidar
/livox/imu
/goal_pose
/goal_point_3d
```

`/goal_pose` is the planner input. `/goal_point_3d` is optional and only needed if you want the adapter node to convert a point into `/goal_pose`.

FAST-LIO outputs:

```text
/cloud_registered
/cloud_registered_body
/Odometry
/Laser_map
```

SC-PGO outputs:

```text
/slam/optimized_odom
/slam/optimized_path
/slam/optimized_map
/slam/loop_scan_local
/slam/loop_submap_local
```

SUPER outputs:

```text
/planning/pos_cmd
/planning_cmd/poly_traj
/planning_cmd/poly_traj_path
/planning_cmd/poly_traj_marker
```

PX4/MAVROS bridge outputs:

```text
/mavros/vision_pose/pose
/mavros/setpoint_raw/local
```

## Troubleshooting

If `livox_ros_driver2` does not receive data:

- verify `MID360_config.json`
- verify the Orin Ethernet IP
- verify the lidar IP
- check firewall/network isolation
- check the `livox` tmux pane logs

If FAST-LIO does not start:

```bash
source /opt/ros/humble/setup.bash
source ~/super_ws/install/setup.bash
ros2 pkg executables fast_lio
```

You should see:

```text
fast_lio fastlio_mapping
fast_lio fastlio_path_publisher.py
```

If SUPER does not plan:

- confirm `/cloud_registered` is publishing
- confirm `/Odometry` is publishing
- confirm `/goal_pose` is published; if using `/goal_point_3d`, confirm the adapter republishes it to `/goal_pose`
- check the `super` tmux pane logs

If PX4 does not enter OFFBOARD:

- confirm MAVROS is running and `/mavros/state` is publishing
- confirm `/mavros/vision_pose/pose` is publishing from the `px4-bridge` pane
- confirm `/mavros/setpoint_raw/local` is publishing before requesting OFFBOARD
- confirm `arm`, `offboard_mode`, and `planner_enabled`/`hold_position` parameters are set as intended

If the SLAM backend dies:

- check the `slam` tmux pane and the newest `~/super_ws/log/ros2/*/launch.log`
- an exit code `127` usually means a shared library path problem; confirm `install/gtsam/lib/libmetis-gtsam.so` exists
- confirm `/cloud_registered_body` and `/Odometry` are publishing before expecting optimized map output
- restart with `smug stop superplanner && smug start superplanner` after rebuilding `aloam_velodyne`
