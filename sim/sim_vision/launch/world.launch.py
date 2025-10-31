from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from launch.conditions import IfCondition

def generate_launch_description():
    # Path to your world file
    world_file = PathJoinSubstitution([
        FindPackageShare('sim_vision'),
        'worlds',
        'workspace_world.sdf'
    ])

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='rviz.rviz',
        description='RViz config file'
    )

    gz_bridge_params_path = os.path.join(
        get_package_share_directory('sim_vision'),
        'config',
        'gz_bridge.yaml'
    )

    # Declare world argument (optional)
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to world model file (.sdf) to load'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # Launch Gazebo Harmonic (gz sim)
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', LaunchConfiguration('world')],
        output='screen'
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([get_package_share_directory('sim_vision'), 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    return LaunchDescription([
        declare_world_arg,
        sim_time_arg,
        start_gazebo,
        gz_bridge_node,
        rviz_launch_arg,
        rviz_config_arg,
        rviz_node
    ])
