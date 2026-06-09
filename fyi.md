# FYI: SUPER + MID360 Setup Notes

## Frames and TF

- FAST-LIO publishes `/Odometry`, `/cloud_registered`, and `/path` in `camera_init`.
- FAST-LIO also publishes an identity static TF from `map` to `camera_init`.
- SUPER and ROG-Map visualization messages are still mostly stamped as `world`.
- We did not edit ROG-Map source. Instead, we added an identity static TF:

```text
camera_init -> world
translation: 0 0 0
rotation: 0 0 0 1
```

This lets RViz display `world` messages and `camera_init` messages together.

Locations:

- `super_planner/launch/fast_lio_mid360.launch.py`
- `scripts/super_mid360.smug.yml`

## RViz

- Main config: `super_planner/rviz/mid360_goal_path.rviz`
- Fixed frame is set to `world`.
- Displays include:
  - FAST-LIO pointcloud: `/cloud_registered`
  - FAST-LIO odom: `/Odometry`
  - FAST-LIO path: `/fastlio/path`
  - SUPER goal: `/goal_pose`
  - SUPER visualization markers under `/visualization/*`
  - Polynomial trajectory visualization:
    - `/planning_cmd/poly_traj_path`
    - `/planning_cmd/poly_traj_marker`
  - ROG-Map visualization:
    - `/rog_map/occ`
    - `/rog_map/inf_occ`
    - `/rog_map/map_bound`

## Goal Pose

- RViz 2D Goal is republished through `goal_point_3d_node`.
- Input pose topic: `/goal_pose_2d`
- Output topic: `/goal_pose`
- Default frame for generated goals in the MID360 launch/smug path: `camera_init`
- The adapter no longer forces z for 2D goals by default.
- Planner goal height is controlled by `fsm.click_height` in `super_planner/config/fast_lio_mid360_ros2.yaml`.
- With `click_height > -5`, the FSM overwrites incoming goal z with `click_height`.

## Trajectory Visualization

- Added `poly_traj_viz_node`.
- Input: `/planning_cmd/poly_traj`
- Outputs:
  - `/planning_cmd/poly_traj_path`
  - `/planning_cmd/poly_traj_marker`
- Visualization output frame is set to `camera_init`.
- RViz can show it in fixed frame `world` because of the identity TF.

## MAVROS Offboard Bridge

- Added source for `mavros_offboard_bridge_node`.
- It is built only when `mavros_msgs` is available in the workspace/environment.
- Launch hook is disabled by default:

```bash
ros2 launch super_planner fast_lio_mid360.launch.py mavros_offboard:=true
```

- FAST-LIO odometry should feed MAVROS external vision:
  - input: `/Odometry`
  - output: `/mavros/vision_pose/pose`
- SUPER real-time command should feed MAVROS offboard setpoints:
  - input: `/planning/pos_cmd`
  - output: `/mavros/setpoint_raw/local`
- MAVROS handles the ROS ENU to PX4 NED transform internally.
- The node does not arm or change modes. Use MAVROS services or a separate supervisor for:
  - `/mavros/set_mode`
  - `/mavros/cmd/arming`

Use `/planning_cmd/poly_traj` only if a downstream MPC/controller is designed to consume full polynomial segments. For MAVROS offboard setpoints, `/planning/pos_cmd` is the right input.

## ROG-Map

- ROG-Map source was left untouched.
- ROG-Map visualization topics are displayed in RViz through the identity TF.
- Unknown-map visualization is disabled by config:

```yaml
rog_map:
  visualization:
    pub_unknown_map_en: false
```

## Planning Failure Note

The earlier `GeneratePolytopeFromLine failed` with seed z near `0.050` was caused by the start/odom point being below the planner corridor lower bound, not by the goal z. The 2D goal z override affects only the clicked goal, not the current odometry/start state.
