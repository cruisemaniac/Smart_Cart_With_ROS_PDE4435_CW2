import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ---- Package paths ----
    desc_pkg = get_package_share_directory('smart_cart_description')
    gz_pkg   = get_package_share_directory('smart_cart_gazebo')

    # ---- Process URDF from Xacro ----
    xacro_file = os.path.join(desc_pkg, 'urdf', 'smart_cart.urdf.xacro')
    robot_urdf  = xacro.process_file(xacro_file).toxml()

    # ---- World path ----
    world_file = os.path.join(gz_pkg, 'worlds', 'supermarket.sdf')

    # ---- Bridge config ----
    bridge_config = os.path.join(gz_pkg, 'config', 'ros_gz_bridge.yaml')

    return LaunchDescription([

        # ---- 1. Launch Gazebo Harmonic ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                )
            ),
            launch_arguments={
                'gz_args': f'-r {world_file}',
                'on_exit_shutdown': 'true',
            }.items(),
        ),

        # ---- 2. Robot State Publisher ----
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_urdf,
                'use_sim_time': True,
            }],
        ),

        # ---- 3. Spawn robot into Gazebo ----
        # Delay slightly so Gazebo is ready before spawn
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'smart_cart',
                        '-topic', '/robot_description',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.20',
                        '-R', '0.0',
                        '-P', '0.0',
                        '-Y', '0.0',
                    ],
                    output='screen',
                ),
            ],
        ),

        # ---- 4. ROS-Gz Bridge ----
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='ros_gz_bridge',
                    output='screen',
                    parameters=[{'config_file': bridge_config}],
                ),
            ],
        ),

        # ---- 5. RViz2 ----
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                ),
            ],
        ),

    ])