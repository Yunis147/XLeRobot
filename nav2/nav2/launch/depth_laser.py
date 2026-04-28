import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the default SLAM Toolbox launch file
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')

    return LaunchDescription([
        # 1. STATIC TRANSFORM: Connects the robot's base to the camera
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0.1', '0.0', '0.2', '0', '0', '0', 'base_link', 'camera_link']
        ),

        # 1.5 THE FIX: The missing "footprint" bridge!
        # SLAM Toolbox refuses to work without a 'base_footprint' frame. 
        # This tells SLAM that footprint and base_link are the exact same place.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='footprint_to_link_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        ),

        # 2. DEPTH TO LASERSCAN: Converts 3D depth to 2D scan
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[
                ('depth', '/camera/camera/depth/image_rect_raw'),
                ('depth_camera_info', '/camera/camera/depth/camera_info'),
                ('scan', '/scan')
            ],
            parameters=[{
                'scan_time': 0.033,
                'range_min': 0.45,
                'range_max': 8.0,
                'scan_height': 5,
                'output_frame': 'camera_depth_frame',
                'use_sim_time': False
            }]
        ),

        # 3. SLAM TOOLBOX: The actual mapping node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={
                'use_sim_time': 'false',
                'transform_timeout': '0.5'
            }.items()
        ),

        # 4. RVIZ2: For visualization
       # Node(
         #   package='rviz2',
        #    executable='rviz2',
       #     name='rviz2',
      #      output='screen'
     #   )
    ])

#base_footprint → base_link → camera_link(no odom link)

