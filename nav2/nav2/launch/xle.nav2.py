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

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0.01', '0.0', '0.6', '1.57', '0.0', '0.0', 'base_link', 'laser']
    )

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
        static_tf,
        nav2_launch
    ])