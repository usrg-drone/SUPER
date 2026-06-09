# SUPER MID360 ROS 2 Stack

This branch packages a ROS 2 Humble runtime stack for a Jetson Orin NX 16 GB using a Livox MID360 lidar:

- `super_planner`: SUPER planner, ROS 2 build, no simulation packages.
- `mars_quadrotor_msgs`: extracted message definitions required by SUPER.
- `rog_map`: local map backend used by SUPER.
- `FAST_LIO_GPU`: CUDA-enabled FAST-LIO for lidar-inertial odometry.
- `Livox-SDK2`: local SDK source used by `livox_ros_driver2`.
- `livox_ros_driver2`: ROS 2 Livox driver for MID360 custom messages.
- `scripts/`: build/setup helpers and a `smug` tmux runtime configuration.

The intended runtime data flow is:

```text
MID360 -> livox_ros_driver2 -> FAST_LIO_GPU -> /cloud_registered + /Odometry -> SUPER
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
- patch `livox_ros_driver2` to use the local SDK path
- build `mars_quadrotor_msgs`, `rog_map`, `super_planner`, and `livox_ros_driver2`
- build `FAST_LIO_GPU` with CUDA enabled for Orin architecture `87`
- verify the installed ROS executables

If your workspace is not `~/super_ws`, pass `WORKSPACE`:

```bash
WORKSPACE=/home/jetson/super_ws ./scripts/setup_fresh_orin.sh
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
```

Start the tmux/smug runtime:

```bash
cd ~/super_ws/src
./scripts/start_super_mid360_tmux.sh
```

Stop it:

```bash
cd ~/super_ws/src
./scripts/stop_super_mid360_tmux.sh
```

The tmux session starts separate windows/panes for:

- Zenoh RMW daemon, if configured in `scripts/super_mid360.smug.yml`
- `livox_ros_driver2_node`
- `fastlio_mapping`
- FAST-LIO path publisher
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

The tmux config has RViz support. You can override it when starting:

```bash
ENABLE_RVIZ=1 RVIZ_CONFIG=~/super_ws/src/super_planner/rviz/mid360_goal_path.rviz \
  ./scripts/start_super_mid360_tmux.sh
```

To disable RViz:

```bash
ENABLE_RVIZ=0 ./scripts/start_super_mid360_tmux.sh
```

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

For 2D goals, SUPER uses the fixed height parameter in:

```text
super_planner/config/fast_lio_mid360_ros2.yaml
```

The parameter is:

```yaml
fsm:
  click_height: 1.5
```

Change `click_height` to set the flight/planning height used for RViz-style 2D goals. For true 3D goals, publish the desired `z` directly in `/goal_pose`, or publish it in `/goal_point_3d` if you are using the optional adapter.

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
/Odometry
```

SUPER outputs:

```text
/planning/pos_cmd
/planning_cmd/poly_traj
/planning_cmd/poly_traj_path
/planning_cmd/poly_traj_marker
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
