import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('qcar2_localization')
    cartographer_config_dir = os.path.join(pkg_share, 'config', 'cartographer')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'localization.rviz')
    default_joint_to_twist_params = os.path.join(
        pkg_share, 'config', 'joint_to_twist', 'joint_to_twist.yaml'
    )
    default_ekf_params = os.path.join(
        pkg_share, 'config', 'robot_localization', 'ekf_joint_imu.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map_yaml')
    autostart = LaunchConfiguration('autostart')
    configuration_basename = LaunchConfiguration('configuration_basename')
    load_state_filename = LaunchConfiguration('load_state_filename')
    use_cartographer_occupancy_grid = LaunchConfiguration('use_cartographer_occupancy_grid')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    use_joint_to_twist = LaunchConfiguration('use_joint_to_twist')
    joint_to_twist_params_file = LaunchConfiguration('joint_to_twist_params_file')
    use_ekf = LaunchConfiguration('use_ekf')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    ekf_odom_topic = LaunchConfiguration('ekf_odom_topic')

    use_qcar2_lidar_tf = LaunchConfiguration('use_qcar2_lidar_tf')
    lidar_tf_x = LaunchConfiguration('lidar_tf_x')
    lidar_tf_y = LaunchConfiguration('lidar_tf_y')
    lidar_tf_z = LaunchConfiguration('lidar_tf_z')
    lidar_tf_yaw = LaunchConfiguration('lidar_tf_yaw')
    lidar_tf_pitch = LaunchConfiguration('lidar_tf_pitch')
    lidar_tf_roll = LaunchConfiguration('lidar_tf_roll')
    lidar_parent_frame = LaunchConfiguration('lidar_parent_frame')
    lidar_child_frame = LaunchConfiguration('lidar_child_frame')

    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    map_yaml_param = ParameterValue(map_yaml, value_type=str)
    autostart_param = ParameterValue(autostart, value_type=bool)

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_yaml', default_value='/workspaces/isaac_ros-dev/ros2/map/map.yaml'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='scenario3_lidar_odom.lua',
            description='Cartographer config file. EKF odom only mode starts with scenario3 by default.',
        ),
        DeclareLaunchArgument(
            'load_state_filename', default_value='/workspaces/isaac_ros-dev/ros2/qlabs_map.pbstream'
        ),
        DeclareLaunchArgument('use_joint_to_twist', default_value='true'),
        DeclareLaunchArgument('joint_to_twist_params_file', default_value=default_joint_to_twist_params),
        DeclareLaunchArgument('use_ekf', default_value='true'),
        DeclareLaunchArgument('ekf_params_file', default_value=default_ekf_params),
        DeclareLaunchArgument('ekf_odom_topic', default_value='/wheel/odometry'),
        DeclareLaunchArgument('use_cartographer_occupancy_grid', default_value='false'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config_file', default_value=default_rviz_config),
        DeclareLaunchArgument('use_qcar2_lidar_tf', default_value='true'),
        DeclareLaunchArgument('lidar_tf_x', default_value='0.1'),
        DeclareLaunchArgument('lidar_tf_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_z', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_yaw', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_parent_frame', default_value='base_link'),
        DeclareLaunchArgument('lidar_child_frame', default_value='base_scan'),
    ]

    qcar2_virtual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('qcar2_nodes'),
                'launch',
                'qcar2_virtual_launch.py',
            )
        )
    )

    fixed_lidar_tf_node = Node(
        condition=IfCondition(use_qcar2_lidar_tf),
        package='qcar2_nodes',
        executable='fixed_lidar_frame_virtual',
        name='fixed_lidar_frame',
        output='screen',
    )

    static_lidar_tf_node = Node(
        condition=UnlessCondition(use_qcar2_lidar_tf),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fixed_lidar_frame_static',
        output='screen',
        arguments=[
            lidar_tf_x,
            lidar_tf_y,
            lidar_tf_z,
            lidar_tf_yaw,
            lidar_tf_pitch,
            lidar_tf_roll,
            lidar_parent_frame,
            lidar_child_frame,
        ],
    )

    joint_to_twist_node = Node(
        condition=IfCondition(use_joint_to_twist),
        package='qcar2_localization',
        executable='joint_to_twist_node',
        name='joint_to_twist_node',
        output='screen',
        parameters=[
            joint_to_twist_params_file,
            {
                'use_sim_time': use_sim_time_param,
            },
        ],
    )

    ekf_node = Node(
        condition=IfCondition(use_ekf),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file,
            {
                'use_sim_time': use_sim_time_param,
            },
        ],
        remappings=[
            ('/odometry/filtered', ekf_odom_topic),
        ],
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_param}],
        arguments=[
            '-configuration_directory',
            cartographer_config_dir,
            '-configuration_basename',
            configuration_basename,
            '-load_state_filename',
            load_state_filename,
        ],
        remappings=[
            ('/odom', ekf_odom_topic),
        ],
    )

    cartographer_occupancy_grid_node = Node(
        condition=IfCondition(use_cartographer_occupancy_grid),
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_param}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec],
        remappings=[('/map', '/cartographer_map')],
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time_param,
                'yaml_filename': map_yaml_param,
            },
        ],
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time_param,
                'autostart': autostart_param,
                'node_names': ['map_server'],
            },
        ],
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2_localization',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time_param}],
    )

    return LaunchDescription(
        declare_args
        + [
            qcar2_virtual_launch,
            fixed_lidar_tf_node,
            static_lidar_tf_node,
            joint_to_twist_node,
            ekf_node,
            cartographer_node,
            cartographer_occupancy_grid_node,
            map_server_node,
            lifecycle_manager_localization,
            rviz_node,
        ]
    )
