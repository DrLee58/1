import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('physical_ai_amr')
    urdf_path = os.path.join(pkg_share, 'urdf', 'physical_ai_amr.urdf.xacro')
    world_path = os.path.join(pkg_share, 'worlds', 'empty_world.sdf')

    # Docker on Mac has no display. Default to Gazebo server only (-s).
    # For a GUI session, launch with: gz_args:="-r <world>"
    declare_gz_args = DeclareLaunchArgument(
        'gz_args',
        default_value=f'-s -r {world_path}',
        description='Arguments passed to gz sim (Harmonic)',
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': LaunchConfiguration('gz_args'),
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_physical_ai_amr',
        output='screen',
        arguments=[
            '-name', 'physical_ai_amr',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.08',
        ],
    )

    return LaunchDescription([
        declare_gz_args,
        gz_sim,
        robot_state_publisher,
        ros_gz_bridge,
        TimerAction(period=4.0, actions=[spawn_robot]),
    ])
