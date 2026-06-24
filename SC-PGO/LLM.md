# FAST_LIO_SLAM Backend Integration for `usrg-drone/SUPER/tree/jkp`

This document describes how to add `gisbi-kim/FAST_LIO_SLAM` as a SLAM backend to the current FAST-LIO + ROG-Map + SUPER + PX4 stack.

Main rule:

```text
Use FAST-LIO local odometry for flight control.
Use FAST_LIO_SLAM pose-graph output for global map correction/reporting.
Do NOT feed loop-closure jumps directly into PX4 or SUPER.
```

---

## 0. Target Architecture

Current stack:

```text
Livox LiDAR + Livox IMU
        ↓
FAST-LIO / FAST-LIO-GPU
        ↓
/Odometry + /cloud_registered
        ↓
ROG-Map + SUPER
        ↓
/planning/pos_cmd
        ↓
PX4 Offboard / Position Control
```

Add FAST_LIO_SLAM as a parallel backend:

```text
FAST-LIO /Odometry
FAST-LIO /cloud_registered
        ↓
FAST_LIO_SLAM / SC-PGO
        ↓
loop closure + pose graph optimization
        ↓
global corrected map / map→odom correction
```

Final architecture:

```text
                         ┌─────────────────────────────┐
                         │ Livox LiDAR + Livox IMU     │
                         └──────────────┬──────────────┘
                                        ↓
                              FAST-LIO / FAST-LIO-GPU
                                        ↓
                  ┌─────────────────────┴─────────────────────┐
                  ↓                                           ↓
          local odom + cloud                         local odom + cloud
                  ↓                                           ↓
          ROG-Map + SUPER                         FAST_LIO_SLAM / SC-PGO
                  ↓                                           ↓
          /planning/pos_cmd                  optimized global map / map→odom
                  ↓                                           ↓
              PX4 bridge                              map/report/global planning
                  ↓
                PX4 EKF2
```

---

## 1. What FAST_LIO_SLAM Is

`FAST_LIO_SLAM` is not a replacement for FAST-LIO.

It is a backend that combines:

```text
FAST-LIO2 odometry
        +
Scan Context loop detection
        +
GTSAM pose graph optimization
```

In the original repository, FAST-LIO and SC-PGO run separately. SC-PGO subscribes to FAST-LIO odometry and point cloud topics, detects loop closures, adds constraints, and generates an optimized map.

For the drone stack, treat it as:

```text
FAST-LIO = real-time local odometry
FAST_LIO_SLAM = global consistency backend
```

---

## 2. Important Control Rule

Do not send pose-graph-optimized pose directly to PX4 while flying.

Bad:

```text
FAST_LIO_SLAM optimized pose
        ↓
PX4 external vision
```

Reason: loop closure can create discontinuous jumps.

Good:

```text
FAST-LIO local odom
        ↓
PX4 external vision
```

and separately:

```text
FAST_LIO_SLAM
        ↓
map→odom correction / global map / report
```

---

## 3. Recommended TF Structure

Use a standard local/global split:

```text
map
└── odom / camera_init
    └── base_link
        └── livox_frame
```

Meaning:

| TF | Type | Source | Behavior | Used by |
|---|---|---|---|---|
| `camera_init → base_link` | Dynamic | FAST-LIO local odom corrected to body frame | Smooth, no jumps | PX4, local debugging |
| `map → camera_init` | Dynamic | SLAM backend correction node | May change after loop closure | Global map/report |
| `base_link → livox_frame` | Static | Measured mount transform | Fixed sensor mount | odom correction |
| `map → base_link` | Derived | TF composition | Globally corrected pose | visualization/report/global planning |

Your current repo uses `camera_init` as the FAST-LIO local world frame, so treat:

```text
camera_init ≈ odom
```

Do not make PX4 depend on `map → camera_init`. PX4 should use the smooth local body odometry.

---

## 4. Existing Topics in Current Stack

| Publisher | Topic | Type | Meaning | Consumer |
|---|---|---|---|---|
| FAST-LIO | `/Odometry` | `nav_msgs/msg/Odometry` | Local FAST-LIO pose, approximately `camera_init → livox_frame` | SUPER, odom correction node, FAST_LIO_SLAM |
| FAST-LIO | `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Registered point cloud in `camera_init` | ROG-Map/SUPER, FAST_LIO_SLAM |
| SUPER | `/planning/pos_cmd` | `quadrotor_msgs/msg/PositionCommand` | Safe local trajectory command | PX4 Offboard bridge |
| Mission/exploration | `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Goal pose in `camera_init` | SUPER |

---

## 5. New Topics to Add

| Publisher | Topic | Type | Meaning | Consumer |
|---|---|---|---|---|
| `livox_to_base_odom_node` | `/visual_odometry_base_link` | `nav_msgs/msg/Odometry` | Corrected body pose, `camera_init → base_link` | MAVROS/PX4 external vision |
| `livox_to_base_odom_node` | `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic `camera_init → base_link` | RViz/debug |
| `static_transform_publisher` | `/tf_static` | `tf2_msgs/msg/TFMessage` | Static `base_link → livox_frame` | correction node/debug |
| FAST_LIO_SLAM / SC-PGO | `/slam/optimized_path` | `nav_msgs/msg/Path` | Optimized global trajectory | RViz/report |
| FAST_LIO_SLAM / SC-PGO | `/slam/optimized_map` | `sensor_msgs/msg/PointCloud2` | Loop-closed global point cloud map | RViz/report |
| `slam_map_odom_node` | `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic `map → camera_init` | RViz/global planner |
| `slam_map_odom_node` | `/global_pose` | `nav_msgs/msg/Odometry` | Globally corrected `map → base_link` | report/global planning only |

The exact output names from FAST_LIO_SLAM may differ depending on the ROS2 port/adaptation. If the package publishes different names, remap them to the names above.

---

## 6. ROS1 vs ROS2 Warning

The original `gisbi-kim/FAST_LIO_SLAM` repository is ROS1-style.

Your current stack is ROS2.

Choose one:

### Option A — Use/adapt a ROS2 port

Preferred:

```text
FAST-LIO ROS2
        ↓
FAST_LIO_SLAM_ros2 / adapted SC-PGO
```

Directly subscribe to ROS2 topics:

```text
/Odometry
/cloud_registered
```

### Option B — Run FAST_LIO_SLAM in ROS1 and bridge topics

Use `ros1_bridge`:

```text
ROS2 FAST-LIO topics
        ↓
ros1_bridge
        ↓
ROS1 FAST_LIO_SLAM
        ↓
ros1_bridge
        ↓
ROS2 optimized map/path
```

For a drone, Option A is cleaner. Option B is acceptable for logging/offboard mapping tests but adds latency and operational complexity.

---

## 7. FAST_LIO_SLAM Input Mapping

SC-PGO needs two main inputs:

```text
1. Odometry
2. Registered lidar cloud / keyframe cloud
```

Use:

| FAST_LIO_SLAM expected concept | Your topic |
|---|---|
| FAST-LIO odometry | `/Odometry` |
| Registered cloud | `/cloud_registered` |
| Local frame | `camera_init` |
| Sensor/body frame | raw FAST-LIO child frame, or normalized `livox_frame` |

Recommended config/remap:

```yaml
fast_lio_slam:
  odom_topic: "/Odometry"
  cloud_topic: "/cloud_registered"
  odom_frame: "camera_init"
  lidar_frame: "livox_frame"
  map_frame: "map"
```

If FAST-LIO `/Odometry.child_frame_id` is not `livox_frame`, either keep the original child frame and document it, or normalize it in a small relay node.

---

## 8. Keep SUPER Unchanged

SUPER should continue using raw local FAST-LIO data:

```text
FAST-LIO /Odometry
FAST-LIO /cloud_registered
        ↓
ROG-Map/SUPER
```

Do not feed loop-closed pose into SUPER unless you explicitly implement smooth correction.

Reason:

```text
ROG-Map + SUPER are local planning modules.
They need smooth local consistency more than global loop-closed accuracy.
```

---

## 9. PX4 External Vision Path

PX4 should use smooth local odometry, not loop-closed pose.

Use:

```text
FAST-LIO /Odometry
        ↓
livox_to_base_odom_node
        ↓
/visual_odometry_base_link
        ↓
MAVROS or px4_ros_com bridge
        ↓
PX4 EKF2
```

The correction node computes:

```text
T_camera_init_base = T_camera_init_livox × inverse(T_base_livox)
```

Then publishes:

```text
/visual_odometry_base_link
  header.frame_id: camera_init
  child_frame_id: base_link
```

For MAVROS, feed this corrected pose/odom to the MAVROS external-vision path, for example:

```text
/visual_odometry_base_link
        ↓
/mavros/vision_pose/pose
```

or:

```text
/visual_odometry_base_link
        ↓
/mavros/odometry/out
```

depending on your MAVROS setup.

For PX4 ROS2 native:

```text
/visual_odometry_base_link
        ↓
/fmu/in/vehicle_visual_odometry
```

or the corresponding `px4_msgs/msg/VehicleVisualOdometry` input topic for your PX4 version.

Important:

```text
ROS side: ENU + FLU
PX4 side: NED + FRD
```

MAVROS usually handles common ENU/NED conversion. If using `px4_msgs` directly, do the conversion explicitly.

---

## 10. SLAM Global Correction Output

The SLAM backend should output a global correction:

```text
map → camera_init
```

This keeps:

```text
camera_init → base_link
```

smooth while still giving globally corrected pose:

```text
map → base_link = map → camera_init × camera_init → base_link
```

Recommended `slam_map_odom_node` behavior:

```cpp
on_optimized_pose_or_path():
    T_map_base_optimized = latest optimized global base/lidar pose
    T_camera_init_base_local = latest local corrected odom

    T_map_camera_init =
        T_map_base_optimized * inverse(T_camera_init_base_local)

    publish TF:
        map -> camera_init
```

If FAST_LIO_SLAM gives optimized lidar pose instead of base pose, first convert:

```text
T_map_base_optimized = T_map_livox_optimized × inverse(T_base_livox)
```

Then compute `map → camera_init`.

---

## 11. Handling Loop-Closure Jumps

Do not directly change:

```text
camera_init → base_link
```

on loop closure.

Instead, change:

```text
map → camera_init
```

Optionally smooth it:

```text
map → camera_init old
        ↓
interpolate over 1–5 seconds
        ↓
map → camera_init new
```

Recommended consumer split:

| Consumer | Frame/Pose |
|---|---|
| PX4 EKF2 | `camera_init → base_link` local pose |
| SUPER/ROG-Map | raw FAST-LIO `/Odometry` + `/cloud_registered` |
| RViz global map | `map → camera_init → base_link` |
| report/geotagging | `map → base_link` |
| long-range return-home planner | `map` goal converted to `camera_init` goal before SUPER |

---

## 12. Exploration / Return-Home Integration

For exploration:

```text
FUEL/FALCON frontier logic
        ↓
global goal in map frame
        ↓
convert to local camera_init frame
        ↓
/goal_pose
        ↓
SUPER
```

Conversion:

```text
T_camera_init_goal = inverse(T_map_camera_init) × T_map_goal
```

Then publish to SUPER:

```text
/goal_pose
  header.frame_id: camera_init
```

Do not send a `map`-frame goal directly to SUPER unless SUPER is modified to use the `map → camera_init` correction.

---

## 13. Node Summary

### Existing nodes

| Node | Keep? | Notes |
|---|---|---|
| Livox driver | Yes | Publishes raw Livox data |
| FAST-LIO / FAST-LIO-GPU | Yes | Main local odometry source |
| ROG-Map | Yes | Local occupancy map |
| SUPER | Yes | Local planner |
| PX4 bridge | Yes | Offboard setpoint bridge |

### New nodes

| Node | Required? | Purpose |
|---|---|---|
| `livox_to_base_odom_node` | Yes | Convert sensor pose to body pose for PX4 |
| `static_transform_publisher` | Yes | Publish `base_link → livox_frame` |
| `fast_lio_slam_node` / `sc_pgo_node` | Yes | Loop detection and pose graph optimization |
| `slam_map_odom_node` | Recommended | Publish `map → camera_init` correction |
| `global_report_node` | Optional | Store globally corrected map/object detections |
| `global_to_local_goal_node` | Optional | Convert `map` goals to `camera_init` goals for SUPER |

---

## 14. Topic Summary Table

| Topic | Type | Producer | Consumer | Control-safe? |
|---|---|---|---|---|
| `/Odometry` | `nav_msgs/msg/Odometry` | FAST-LIO | SUPER, correction node, FAST_LIO_SLAM | Yes, local |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | FAST-LIO | ROG-Map/SUPER, FAST_LIO_SLAM | Yes, local |
| `/visual_odometry_base_link` | `nav_msgs/msg/Odometry` | `livox_to_base_odom_node` | PX4 EV bridge | Yes, local |
| `/planning/pos_cmd` | `quadrotor_msgs/msg/PositionCommand` | SUPER | PX4 Offboard bridge | Yes |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Mission/exploration | SUPER | Yes if in `camera_init` |
| `/slam/optimized_path` | `nav_msgs/msg/Path` | FAST_LIO_SLAM | RViz/report | No, global |
| `/slam/optimized_map` | `sensor_msgs/msg/PointCloud2` | FAST_LIO_SLAM | RViz/report | No, global |
| `/global_pose` | `nav_msgs/msg/Odometry` | `slam_map_odom_node` | report/global planner | No direct PX4 |
| `/tf` | `tf2_msgs/msg/TFMessage` | correction + SLAM correction nodes | RViz/all nodes | Depends on edge |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | static TF publisher | all nodes | Yes |

---

## 15. Launch Order

Recommended startup sequence:

```text
1. Start Livox driver.
2. Start FAST-LIO.
3. Confirm /Odometry and /cloud_registered are stable.
4. Start static TF: base_link → livox_frame.
5. Start livox_to_base_odom_node.
6. Confirm /visual_odometry_base_link is stable.
7. Start PX4 external vision bridge.
8. Confirm PX4 EKF2 local position is valid.
9. Start ROG-Map/SUPER.
10. Start FAST_LIO_SLAM / SC-PGO backend.
11. Start slam_map_odom_node.
12. Start Offboard setpoint bridge.
```

Important:

```text
PX4 should not wait for loop closure.
PX4 only needs stable local external vision.
```

---

## 16. Validation Checklist

### FAST-LIO local odom

```bash
ros2 topic echo /Odometry
ros2 topic echo /cloud_registered
```

Expected:

```text
/Odometry stable
/cloud_registered floor flat, walls vertical
```

### Body-corrected odom

```bash
ros2 topic echo /visual_odometry_base_link
ros2 run tf2_ros tf2_echo camera_init base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
```

Expected:

```text
camera_init → base_link exists
base_link → livox_frame exists
/visual_odometry_base_link child_frame_id = base_link
```

### SLAM backend

```bash
ros2 topic list | grep slam
ros2 topic echo /slam/optimized_path
ros2 topic echo /slam/optimized_map
ros2 run tf2_ros tf2_echo map camera_init
```

Expected:

```text
optimized path grows over time
optimized map appears in RViz
map → camera_init exists after slam_map_odom_node starts
```

### PX4

Check:

```text
PX4 local position valid
EKF2 innovations not diverging
No position jump when SLAM loop closure happens
Offboard hold stable
```

---

## 17. Common Failure Modes

| Symptom | Likely Cause | Fix |
|---|---|---|
| PX4 jumps after loop closure | Optimized SLAM pose sent to PX4 | Use `/visual_odometry_base_link`, not `/global_pose` |
| SUPER map jumps | Feeding loop-corrected odom into ROG-Map | Keep SUPER on raw FAST-LIO |
| Global map correct but drone control bad | Wrong stream used for PX4 | PX4 should use local corrected body odom |
| Drone moves opposite direction | ENU/NED conversion wrong | Fix MAVROS/px4_msgs bridge |
| PX4 attitude inconsistent | FLU/FRD or mount TF wrong | Check quaternion conversion and `base_link → livox_frame` |
| RViz global pose jumps | Expected after loop closure | Smooth `map → camera_init` if needed |
| Return-home goal is wrong | Sent `map` goal directly to SUPER | Convert `map` goal to `camera_init` first |
| SLAM backend fails to detect loops | Scan Context params too strict, sparse scans, aggressive motion | Tune keyframe spacing, descriptor params, loop threshold |
| Orin NX CPU overloaded | Dense map publishing / too frequent keyframes | Downsample cloud, reduce keyframe rate, lower map publish frequency |

---

## 18. Recommended Orin NX Settings

For Jetson Orin NX 16GB:

| Setting | Recommendation |
|---|---|
| FAST-LIO | Keep real-time priority |
| ROG-Map/SUPER | Keep local planning priority |
| FAST_LIO_SLAM keyframe rate | Lower than odometry rate |
| Cloud downsample for SLAM | Use voxel filtering |
| Optimized map publishing | Low rate, e.g. 0.2–1 Hz |
| Loop closure | Run in background thread |
| Pose graph optimization | Do not block control pipeline |
| RViz on drone | Avoid during flight if CPU/GPU constrained |

Priority order:

```text
1. PX4 bridge / external vision
2. FAST-LIO
3. SUPER local planner
4. FAST_LIO_SLAM backend
5. RViz/map visualization
```

---

## 19. Minimal Implementation Plan

### Phase 1 — Control-safe external vision

```text
FAST-LIO /Odometry
        ↓
livox_to_base_odom_node
        ↓
/visual_odometry_base_link
        ↓
PX4 EKF2
```

Fly position hold before adding SLAM.

### Phase 2 — Add FAST_LIO_SLAM in passive mode

```text
FAST-LIO /Odometry + /cloud_registered
        ↓
FAST_LIO_SLAM
        ↓
/slam/optimized_path
/slam/optimized_map
```

Do not connect it to control.

### Phase 3 — Add `map → camera_init`

```text
FAST_LIO_SLAM optimized pose
        ↓
slam_map_odom_node
        ↓
map → camera_init
```

Use only for visualization/report.

### Phase 4 — Global planning integration

```text
global goal in map
        ↓
global_to_local_goal_node
        ↓
/goal_pose in camera_init
        ↓
SUPER
```

---

## 20. Final System Rule

```text
Local odometry controls the drone.
Global SLAM corrects the map.
Loop closure moves map→camera_init, not camera_init→base_link.
```

This keeps PX4/SUPER smooth while still giving loop-closed global mapping for recon, return-home, and reporting.
