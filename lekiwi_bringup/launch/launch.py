import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():

    description_path = get_package_share_directory('lekiwi_description')
    bringup_path = get_package_share_directory('lekiwi_bringup')

    xacro_file = os.path.join(
        description_path,
        'URDF',
        'lekiwi_hardware.urdf.xacro'
    )

    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    robot_description = {'robot_description': robot_desc}

    controller_config = os.path.join(
        bringup_path,
        'config',
        'controllers.yaml'
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    motor_odom_node = Node(
        package='lekiwi_bringup',  
        executable='motor_odom',
        output='screen'
    )

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

    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen'
    )
    
    arm_home = TimerAction(
        period=5.0,  
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/arm_position_controller/joint_trajectory',
                    'trajectory_msgs/msg/JointTrajectory',
                    '{joint_names: ["STS3215_03a-v1_Revolute-45", '
                    '"STS3215_03a-v1-1_Revolute-49", '
                    '"STS3215_03a-v1-2_Revolute-51", '
                    '"STS3215_03a-v1-3_Revolute-53", '
                    '"STS3215_03a_Wrist_Roll-v1_Revolute-55"], '
                    'points: [{positions: [0.00, 0.00, 0.00, 0.00, 0.00], '
                    'time_from_start: {sec: 3, nanosec: 0}}]}'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        rsp,
        control_node,
        motor_odom_node,  
        joint_state_spawner,
        arm_spawner,
        base_spawner,
        gripper_spawner,
        arm_home,
    ])
