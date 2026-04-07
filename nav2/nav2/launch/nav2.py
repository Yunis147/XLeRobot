import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    map_file = '/home/rpd/Nav2/src/nav2/nav2/my_new_map.yaml'
    params_file = '/home/rpd/Nav2/src/nav2/nav2/dwb_params.yaml'

    return LaunchDescription([
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[('depth', '/camera/camera/depth/image_rect_raw'),
                        ('depth_camera_info', '/camera/camera/depth/camera_info'),
                        ('scan', '/scan')],
            parameters=[{
                'scan_time': 0.033, 
                'range_min': 0.45, 
                'range_max': 5.0, 
                'output_frame': 'camera_depth_frame',
                'use_sim_time': False  # ADDED: Must match Nav2's use_sim_time to prevent TF timestamp drops
            }]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'map': map_file, 
                'use_sim_time': 'false', 
                'params_file': params_file
            }.items()
        ),
        # ADDED: RViz is required so you can click "2D Pose Estimate" to initialize AMCL
      #  Node(
       #     package='rviz2',
        #    executable='rviz2',
         #   name='rviz2',
          #  output='screen',
           # parameters=[{'use_sim_time': False}]
        #)
    ])
