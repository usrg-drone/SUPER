# super_px4_mavros_offboard_bridge

ROS 2 Humble `rclpy` bridge for sending FAST-LIO external vision and SUPER planner setpoints to PX4 through MAVROS.

## Main interfaces

- Subscribes: `/Odometry` (`nav_msgs/msg/Odometry`)
- Publishes: `/mavros/vision_pose/pose` (`geometry_msgs/msg/PoseStamped`)
- Subscribes: `/planning/pos_cmd` (`mars_quadrotor_msgs/msg/PositionCommand`)
- Publishes: `/mavros/setpoint_raw/local` (`mavros_msgs/msg/PositionTarget`)
- Calls: `/mavros/cmd/arming`, `/mavros/set_mode`

## Live parameters

- `arm`: request arm/disarm.
- `offboard_mode`: stream setpoints and request PX4 `OFFBOARD`.
- `hold_position`: force a latched position hold setpoint.
- `planner_enabled`: forward SUPER `/planning/pos_cmd` when true.
- `takeoff_altitude`: ENU hover altitude used after arming/offboard before planner control.
- `publish_vision_pose`: enable external vision pose publishing.
- `publish_setpoints`: enable MAVROS setpoint publishing.

The external vision relay keeps a continuous output pose by applying an internal transform offset when incoming SLAM odometry has a large discontinuity.
