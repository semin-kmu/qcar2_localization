# qcar2_localization

Cartographer + EKF localization package for QCar2 virtual environment.

## Overview

| Item | Value |
|------|-------|
| Last updated | 2026-03-01 |
| Active launch | `localization_ekf_launch.py` |
| Scope | EKF mode only |

This package is now maintained for a single pipeline:

`/qcar2_joint -> joint_to_twist_node -> /wheel/twist_raw`
`/qcar2_imu -----------------------> ekf_filter_node`
`ekf_filter_node -> /wheel/odometry -> cartographer_node`
`/scan ---------------------------------------------> cartographer_node`

## Launch

```bash
ros2 launch qcar2_localization localization_ekf_launch.py
```

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
- `rviz2` (optional at launch-time if `use_rviz:=true`)

Not required as a direct dependency for this package's EKF-only pipeline:
- `geographic_info`

## Notes

- `localization_ekf_launch.py` remaps EKF output `/odometry/filtered` to `/wheel/odometry`.
- Cartographer default config is `scenario3_lidar_odom.lua` (`use_imu_data = false`) because IMU is fused in EKF.
- EKF config uses `publish_tf: false`; TF chain authority remains in Cartographer.
