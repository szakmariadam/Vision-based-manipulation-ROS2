from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur", package_name="ur_moveit_config").to_moveit_configs()
    moveit_config.robot_description_kinematics["use_sim_time"] = True

    # MoveGroupInterface demo executable
    move_group_interface_demo = Node(
        name="move_group_interface_demo",
        package="robot_manipulation",
        executable="move_group_interface_demo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([move_group_interface_demo])