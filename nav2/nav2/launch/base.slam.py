import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')

    rplidar_dir = get_package_share_directory('rplidar_ros')
    rplidar_launch_file = os.path.join(rplidar_dir, 'launch', 'rplidar_a1_launch.py')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_file),
            launch_arguments={'serial_port': '/dev/ttyUSB0'}.items()
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0.01', '0.0', '0.3', '-1.57', '0.0', '0.0', 'base_link', 'laser']
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={
                'use_sim_time': 'false',
                'transform_timeout': '0.5'
            }.items()
        ),
    ])
