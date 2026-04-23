from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # object detection node
    detect_object = Node(
        package="object_detection_py",
        executable="detect_object",
        output="screen",
    )

    # object position node
    object_position = Node(
        package="object_detection_py",
        executable="object_position",
        output="screen",
    )

    # planning scene node
    planning_scene_updater = Node(
        package="object_detection",
        executable="planning_scene_updater",
        output="screen",
    )

    return LaunchDescription([detect_object, object_position, planning_scene_updater])