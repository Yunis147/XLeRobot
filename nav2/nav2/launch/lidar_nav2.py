import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    
    rplidar_dir = get_package_share_directory('rplidar_ros')
    rplidar_launch_file = os.path.join(rplidar_dir, 'launch', 'rplidar_a1_launch.py')
    
    map_file = '/home/rpd/Nav2/src/nav2/nav2/map/lidar_map.yaml'
    params_file = '/home/rpd/Nav2/src/nav2/nav2/params/dwb_params.yaml'

    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_file)
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0.01', '0.0', '0.3', '-1.57', '0.0', '0.0', 'base_link', 'laser']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='footprint_to_link_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'map': map_file, 
                'use_sim_time': 'false', 
                'params_file': params_file
            }.items()
        ),

        # Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    name='rviz2',
        #    output='screen',
        #    parameters=[{'use_sim_time': False}]
        # )
    ])