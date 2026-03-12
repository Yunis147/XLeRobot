import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    description_path = get_package_share_directory('lekiwi_description')
    bringup_path = get_package_share_directory('lekiwi_bringup')

    # ---- Set Gazebo resource path ----
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(description_path, 'URDF'),
            ':' + str(Path(description_path).parent.resolve()),
        ],
    )

    # ---- World argument ----
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Gazebo world'
    )

    # ---- Start Gazebo ----
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), '.sdf -r -v 1']
        }.items(),
    )

    # ---- Process Xacro ----
    xacro_file = os.path.join(
        description_path,
        'URDF',
        'lekiwi.urdf.xacro'
    )

    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_desc = doc.toxml()

    params = {'robot_description': robot_desc, 'use_sim_time': True}

    # ---- Robot State Publisher ----
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[params],
        output='screen'
    )

    # ---- Spawn Robot (Raised from ground) ----
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc,
            '-name', 'lekiwi',
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15'
        ],
        output='screen'
    )

    # ---- Clock Bridge ----
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # ---- Controller Spawners ----
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_position_controller'],
        output='screen'
    )

    base_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['base_velocity_controller'],
        output='screen'
    )

    # ✅ NEW GRIPPER CONTROLLER SPAWNER
    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen'
    )

    # ---- Controller startup chaining ----
    controller_chain_1 = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_spawner],
        )
    )

    controller_chain_2 = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[arm_spawner, base_spawner, gripper_spawner],
        )
    )

    return LaunchDescription([
        gazebo_resource_path,
        world_arg,
        gazebo,
        bridge,
        rsp,
        spawn_robot,
        controller_chain_1,
        controller_chain_2,
    ])
