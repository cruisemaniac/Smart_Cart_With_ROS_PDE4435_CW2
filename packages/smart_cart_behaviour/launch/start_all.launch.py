"""
start_all.launch.py
Cart starts IDLE. Person teleop opens in a new xterm window.
Press [2] in the teleop window to activate follow-me.
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction, LogInfo, ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    desc_pkg = get_package_share_directory('smart_cart_description')
    gz_pkg   = get_package_share_directory('smart_cart_gazebo')

    xacro_file      = os.path.join(desc_pkg, 'urdf',   'smart_cart.urdf.xacro')
    world_file      = os.path.join(gz_pkg,   'worlds', 'supermarket.sdf')
    person_sdf_file = os.path.join(gz_pkg,   'models', 'person_actor.sdf')
    bridge_config   = os.path.join(gz_pkg,   'config', 'ros_gz_bridge.yaml')
    rviz_config     = os.path.join(gz_pkg,   'config', 'smart_cart.rviz')

    robot_urdf = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([

        LogInfo(msg='[SmartCart] Launching... Cart will be IDLE. Press [2] in teleop to follow.'),

        # t=0 Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py'
                )
            ),
            launch_arguments={'gz_args': f'-r {world_file}', 'on_exit_shutdown': 'true'}.items(),
        ),

        # t=0 Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_urdf, 'use_sim_time': True}],
        ),

        # t=3 Spawn smart cart
        TimerAction(period=3.0, actions=[
            Node(
                package='ros_gz_sim', executable='create', name='spawn_smart_cart',
                arguments=[
                    '-name', 'smart_cart', '-topic', '/robot_description',
                    '-x', '0.0', '-y', '0.0', '-z', '0.20',
                    '-R', '0.0', '-P', '0.0', '-Y', '0.0',
                ],
                output='screen',
            ),
        ]),

        # t=4 Spawn person actor
        TimerAction(period=4.0, actions=[
            LogInfo(msg='Spawning person actor...'),
            Node(
                package='ros_gz_sim', executable='create', name='spawn_person',
                arguments=[
                    '-name', 'person',
                    '-file', person_sdf_file,
                    '-x', '1.5', '-y', '0.0', '-z', '0.0',
                    '-R', '0.0', '-P', '0.0', '-Y', '0.0',
                ],
                output='screen',
            ),
        ]),

        # t=5 Bridge
        TimerAction(period=5.0, actions=[
            Node(
                package='ros_gz_bridge', executable='parameter_bridge',
                name='ros_gz_bridge', output='screen',
                parameters=[{'config_file': bridge_config}],
            ),
        ]),

        # t=6 Behaviour + Navigation (cart starts IDLE)
        TimerAction(period=6.0, actions=[
            LogInfo(msg='Starting behaviour nodes. Cart is IDLE.'),
            Node(
                package='smart_cart_behaviour', executable='obstacle_stop_node',
                name='obstacle_stop_node', output='screen',
                parameters=[{'use_sim_time': True}],
            ),
            Node(
                package='smart_cart_behaviour', executable='follow_me_node',
                name='follow_me_node', output='screen',
                parameters=[{'use_sim_time': True}],
            ),
            Node(
                package='smart_cart_navigation', executable='navigation_node',
                name='navigation_node', output='screen',
                parameters=[{
                    'use_sim_time':    True,
                    'mode':            'idle',
                    'follow_distance': 1.0,
                    'max_speed':       0.8,
                    'target_timeout':  5.0,
                }],
            ),
        ]),

        # t=7 Person teleop window
        # Opens a separate xterm window – this IS the remote control.
        # If xterm is not installed: sudo apt install xterm
        TimerAction(period=7.0, actions=[
            LogInfo(msg='Opening teleop window (W/A/S/D + buttons 1/2/3)...'),
            ExecuteProcess(
                cmd=[
                    'xterm',
                    '-fa', 'Monospace', '-fs', '12',
                    '-bg', 'black', '-fg', 'cyan',
                    '-title', 'Smart Cart Remote Control',
                    '-geometry', '62x22',
                    '-e',
                    'bash', '-c',
                    (
                        'source /opt/ros/jazzy/setup.bash && '
                        'source /home/jayashanka/ros/jazzy_ws/install/setup.bash && '
                        'ros2 run smart_cart_behaviour teleop_person_node; '
                        'echo ""; echo "Teleop ended. Press Enter to close."; read'
                    ),
                ],
                output='screen',
            ),
        ]),

        # t=8 RViz2
        TimerAction(period=8.0, actions=[
            Node(
                package='rviz2', executable='rviz2', name='rviz2',
                output='screen',
                arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
                parameters=[{'use_sim_time': True}],
            ),
        ]),

    ])