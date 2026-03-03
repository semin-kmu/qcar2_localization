# Localization Analysis (EKF-first)

## Document Status
- Last updated: 2026-03-03
- Active launch: `launch/localization_ekf_launch.py`
- Scope: EKF-first mode (`use_ekf:=true` default). Launch includes toggles for selective node disable.

## 1. System Pipeline

```text
/qcar2_joint -> joint_to_twist_node -> /wheel/twist_raw
/qcar2_imu -----------------------> ekf_filter_node
                                  -> /wheel/odometry
/scan ---------------------------> cartographer_node
/wheel/odometry ----------------> cartographer_node
```

## 2. Active Nodes

### 2.1 joint_to_twist_node
- Source: `src/joint_to_twist_node.cpp`
- Input: `/qcar2_joint` (`sensor_msgs/JointState`)
- Output: `/wheel/twist_raw` (`geometry_msgs/TwistWithCovarianceStamped`)
- Behavior:
  - Uses `velocity[velocity_index]`
  - Applies `velocity_scale`, deadband, clipping, low-pass filter
  - Publishes `twist.linear.x` only (no angular velocity output)

### 2.2 ekf_filter_node (robot_localization)
- Package/executable: `robot_localization/ekf_node`
- Config: `config/robot_localization/ekf_joint_imu.yaml`
- Inputs:
  - `twist0: /wheel/twist_raw`
  - `imu0: /qcar2_imu`
- Output:
  - `/odometry/filtered` (remapped to `/wheel/odometry` in launch)
- Key settings:
  - `two_d_mode: true`
  - `publish_tf: false`
  - `world_frame: odom`

### 2.3 cartographer_node
- Package/executable: `cartographer_ros/cartographer_node`
- Config: `config/cartographer/scenario3_lidar_odom.lua`
- Input remap:
  - `/odom` <- `/wheel/odometry`
- Sensor mode:
  - `use_odometry = true`
  - `TRAJECTORY_BUILDER_2D.use_imu_data = false`

### 2.4 Map/TF/Visualization support
- Included launch: `qcar2_nodes/qcar2_virtual_launch.py`
- `nav2_map_server/map_server`
- `nav2_lifecycle_manager/lifecycle_manager`
- `qcar2_nodes/fixed_lidar_frame_virtual` or `tf2_ros/static_transform_publisher`
- `rviz2` (optional)

## 3. Launch Arguments (Current)

From `launch/localization_ekf_launch.py`:
- `use_sim_time` (default: `false`)
- `map_yaml` (default: `/workspaces/isaac_ros-dev/ros2/map/map.yaml`)
- `autostart` (default: `true`)
- `configuration_basename` (default: `scenario3_lidar_odom.lua`)
- `load_state_filename` (default: `/workspaces/isaac_ros-dev/ros2/qlabs_map.pbstream`)
- `use_joint_to_twist` (default: `true`)
- `joint_to_twist_params_file`
- `use_ekf` (default: `true`)
- `ekf_params_file`
- `ekf_odom_topic` (default: `/wheel/odometry`)
- `use_qcar2_lidar_tf`
- `lidar_tf_x`, `lidar_tf_y`, `lidar_tf_z`
- `lidar_tf_yaw`, `lidar_tf_pitch`, `lidar_tf_roll`
- `lidar_parent_frame`, `lidar_child_frame`
- `use_cartographer_occupancy_grid`
- `resolution` (default: `0.05`)
- `publish_period_sec` (default: `1.0`)
- `use_rviz`
- `rviz_config_file`

## 4. Files Kept for EKF-first Mode

- `launch/localization_ekf_launch.py`
- `src/joint_to_twist_node.cpp`
- `config/joint_to_twist/joint_to_twist.yaml`
- `config/robot_localization/ekf_joint_imu.yaml`
- `config/cartographer/scenario3_lidar_odom.lua`
- `rviz/localization.rviz`

## 5. Dependency Notes

Required for this EKF-first package flow:
- `robot_localization`
- `cartographer_ros`
- `qcar2_nodes`
- `nav2_map_server`
- `nav2_lifecycle_manager`
- `tf2_ros`
- `launch`
- `launch_ros`
- `ament_index_python`
- `ros2launch`

Not required as a direct dependency in this package for EKF-first operation:
- `geographic_info`

## 6. Operational Notes

- Default `map_yaml` and `load_state_filename` are environment-specific absolute paths and should typically be overridden.
- If `use_ekf:=false`, cartographer still remaps `/odom` to `ekf_odom_topic`; another source must publish that topic for odometry input.

## 7. Validation Constraints

- No `build` or `ros` command execution was performed for this update.
- Verification was done by static reference checks (launch/source/config consistency).
