# qcar2_localization

Cartographer + EKF localization package for QCar2 virtual environment.

## Overview

| Item | Value |
|------|-------|
| Last updated | 2026-03-03 |
| Active launch | `localization_ekf_launch.py` |
| Scope | EKF-first localization pipeline (`use_ekf:=true` by default) |

Default data flow:

`/qcar2_joint -> joint_to_twist_node -> /wheel/twist_raw`
`/qcar2_imu -----------------------> ekf_filter_node`
`ekf_filter_node -> /wheel/odometry -> cartographer_node`
`/scan ---------------------------------------------> cartographer_node`

Also launched with localization stack support:
- `qcar2_nodes/qcar2_virtual_launch.py` include
- `nav2_map_server/map_server`
- `nav2_lifecycle_manager/lifecycle_manager`
- LiDAR TF via `qcar2_nodes/fixed_lidar_frame_virtual` or `tf2_ros/static_transform_publisher`
- `rviz2` (optional)

## Launch

```bash
ros2 launch qcar2_localization localization_ekf_launch.py
```

## Key Launch Arguments

- `configuration_basename` (default: `scenario3_lidar_odom.lua`)
- `joint_to_twist_params_file`, `use_joint_to_twist`
- `ekf_params_file`, `use_ekf`, `ekf_odom_topic`
- `use_cartographer_occupancy_grid`, `resolution`, `publish_period_sec`
- `use_qcar2_lidar_tf`, `lidar_tf_x/y/z/yaw/pitch/roll`, `lidar_parent_frame`, `lidar_child_frame`
- `use_rviz`, `rviz_config_file`
- `map_yaml`, `load_state_filename`, `use_sim_time`, `autostart`

## In-use Files

### Launch
- `launch/localization_ekf_launch.py`

### Source
- `src/joint_to_twist_node.cpp`

### Config
- `config/joint_to_twist/joint_to_twist.yaml`
- `config/robot_localization/ekf_joint_imu.yaml`
- `config/cartographer/scenario3_lidar_odom.lua`

### Visualization
- `rviz/localization.rviz`

## Runtime Dependencies

Required by the active launch:
- `robot_localization` (for `ekf_node`)
- `cartographer_ros`
- `qcar2_nodes`
- `nav2_map_server`
- `nav2_lifecycle_manager`
- `tf2_ros` (static transform publisher path)
- `launch`
- `launch_ros`
- `ament_index_python`
- `ros2launch`
- `rviz2` (optional at launch-time if `use_rviz:=true`)

Not required as a direct dependency for this package's EKF-first pipeline:
- `geographic_info`

## Notes

- `localization_ekf_launch.py` remaps EKF output `/odometry/filtered` to `/wheel/odometry`.
- Cartographer default config is `scenario3_lidar_odom.lua` (`use_imu_data = false`) because IMU is fused in EKF.
- EKF config uses `publish_tf: false`; TF chain authority remains in Cartographer.
- `map_yaml` and `load_state_filename` defaults are environment-specific absolute paths and should usually be overridden per machine.
