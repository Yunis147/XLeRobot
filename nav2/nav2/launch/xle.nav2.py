import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    map_file    = '/home/rpd/lekiwi_ws/src/mobile-manipulator/nav2/nav2/map/xle_room_map.yaml'
    filter_yaml = '/home/rpd/lekiwi_ws/src/mobile-manipulator/nav2/nav2/params/laser_filter.yaml'
    nav2_params = '/home/rpd/lekiwi_ws/src/mobile-manipulator/nav2/nav2/params/dwb_params.yaml'

    # RPLidar Node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'angle_compensate': True,
        }],
        remappings=[('scan', '/scan_raw')]
    )

    # Laser Filter Node
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        output='screen',
        parameters=[filter_yaml],
        remappings=[
            ('scan', '/scan_raw'),
            ('scan_filtered', '/scan')
        ]
    )

    # Corrected Base-to-Laser TF (-1.57 Yaw restore)
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['--x', '0.01', '--y', '0.0', '--z', '0.6', 
                   '--yaw', '-1.57', '--pitch', '0.0', '--roll', '0.0', 
                   '--frame-id', 'base_link', '--child-frame-id', 'laser']
    )

    # Restored Footprint-to-Link TF
    static_tf_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='footprint_to_link_tf',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0', 
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', 
                   '--frame-id', 'base_footprint', '--child-frame-id', 'base_link']
    )

    # Nav2 Bringup
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false',
            'params_file': nav2_params
        }.items()
    )

    return LaunchDescription([
        rplidar_node,
        laser_filter_node,
        static_tf_laser,
        static_tf_footprint,
        nav2_launch
    ])
