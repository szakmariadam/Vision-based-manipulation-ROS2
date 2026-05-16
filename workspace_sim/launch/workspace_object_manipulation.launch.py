from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    workspace_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("workspace_sim"), "/launch/workspace_moveit.launch.py"]
        ),
    )

    object_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("object_detection"), "/launch/object_detection.launch.py"]
        ),
    )

    object_detection_start = TimerAction(
        period=5.0,
        actions=[object_detection]
    )


    return LaunchDescription([workspace_moveit, object_detection_start])