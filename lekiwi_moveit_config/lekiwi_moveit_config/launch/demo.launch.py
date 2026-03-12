import os

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ------------------------------------------------------------
    # MoveIt Config
    # ------------------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder(
            "LeKiwi",
            package_name="lekiwi_moveit_config"
        )
        .to_moveit_configs()
    )

    # ------------------------------------------------------------
    # Move Group
    # ------------------------------------------------------------
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ],
    )

    # ------------------------------------------------------------
    # RViz
    # ------------------------------------------------------------
    rviz_config_file = os.path.join(
        FindPackageShare("lekiwi_moveit_config").find("lekiwi_moveit_config"),
        "config",
        "moveit.rviz"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}
        ],
    )

    # ------------------------------------------------------------
    # Controller Spawner (ONLY if not already active)
    # ------------------------------------------------------------
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_position_controller"],
        output="screen",
    )

    return LaunchDescription([
        arm_controller_spawner,
        move_group,
        rviz,
    ])