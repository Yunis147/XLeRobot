import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rplidar_dir = get_package_share_directory('rplidar_ros')

    # Launch files
    nav2_launch = os.path.join(
        nav2_bringup_dir,
        'launch',
        'bringup_launch.py'
    )

    rplidar_launch = os.path.join(
        rplidar_dir,
        'launch',
        'rplidar_a1_launch.py'
    )

    # Your saved map
    map_file = '/home/rpd/lekiwi_ws/src/mobile-manipulator/nav2/nav2/map/base_only_map.yaml'

    # Nav2 parameters
    params_file = '/home/rpd/lekiwi_ws/src/mobile-manipulator/nav2/nav2/params/dwb_params.yaml'

    return LaunchDescription([

        #
        # Start RPLidar
        #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch),
            launch_arguments={
                'serial_port': '/dev/ttyUSB0'
            }.items()
        ),

        #
        # base_link -> laser
        #
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=[
                '0.01',
                '0.0',
                '0.3',
                '-1.57',
                '0.0',
                '0.0',
                'base_link',
                'laser'
            ]
        ),

        #
        # Nav2
        #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'slam': 'False',
                'use_localization': 'True',
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': 'false',
                'autostart': 'true'
            }.items()
        ),
    ])
