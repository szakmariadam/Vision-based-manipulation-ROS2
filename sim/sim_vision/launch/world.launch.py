from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to your world file
    world_file = PathJoinSubstitution([
        FindPackageShare('sim_vision'),
        'worlds',
        'workspace_world.sdf'
    ])

    # Declare world argument (optional)
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to world model file (.sdf) to load'
    )

    # Launch Gazebo Harmonic (gz sim)
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', LaunchConfiguration('world')],
        output='screen'
    )

    return LaunchDescription([
        declare_world_arg,
        start_gazebo
    ])
