"""
start_all.launch.py  –  Smart Cart full simulation launcher
============================================================
Single command to start everything:
  ros2 launch smart_cart_behaviour start_all.launch.py

Startup sequence (timed to avoid race conditions):
  t=0s   Gazebo Harmonic  +  Robot State Publisher
  t=3s   Spawn robot into Gazebo
  t=5s   ROS–Gz topic bridge
  t=6s   Follow-Me node  +  Obstacle Stop node  +  Navigation node
  t=7s   RViz2 (with pre-saved config)
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # ── Package share directories ─────────────────────────────────────────
    desc_pkg = get_package_share_directory('smart_cart_description')
    gz_pkg   = get_package_share_directory('smart_cart_gazebo')
    beh_pkg  = get_package_share_directory('smart_cart_behaviour')

    # ── File paths ────────────────────────────────────────────────────────
    xacro_file    = os.path.join(desc_pkg, 'urdf', 'smart_cart.urdf.xacro')
    world_file    = os.path.join(gz_pkg,   'worlds', 'supermarket.sdf')
    bridge_config = os.path.join(gz_pkg,   'config', 'ros_gz_bridge.yaml')
    rviz_config   = os.path.join(gz_pkg,   'config', 'smart_cart.rviz')

    # ── Process URDF from Xacro ───────────────────────────────────────────
    robot_urdf = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([

        LogInfo(msg='[SmartCart] Starting full simulation...'),

        # ══════════════════════════════════════════════════════════════════
        # t=0  GAZEBO HARMONIC
        # ══════════════════════════════════════════════════════════════════
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py'
                )
            ),
            launch_arguments={
                'gz_args': f'-r {world_file}',
                'on_exit_shutdown': 'true',
            }.items(),
        ),

        # ══════════════════════════════════════════════════════════════════
        # t=0  ROBOT STATE PUBLISHER
        # ══════════════════════════════════════════════════════════════════
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

        # ══════════════════════════════════════════════════════════════════
        # t=3  SPAWN ROBOT
        # ══════════════════════════════════════════════════════════════════
        TimerAction(period=3.0, actions=[
            LogInfo(msg='[SmartCart] Spawning robot into Gazebo...'),
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_smart_cart',
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
        ]),

        # ══════════════════════════════════════════════════════════════════
        # t=5  ROS–GZ BRIDGE
        # ══════════════════════════════════════════════════════════════════
        TimerAction(period=5.0, actions=[
            LogInfo(msg='[SmartCart] Starting ROS-Gz bridge...'),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                output='screen',
                parameters=[{'config_file': bridge_config}],
            ),
        ]),

        # ══════════════════════════════════════════════════════════════════
        # t=6  BEHAVIOUR + NAVIGATION NODES
        # ══════════════════════════════════════════════════════════════════
        TimerAction(period=6.0, actions=[
            LogInfo(msg='[SmartCart] Starting behaviour and navigation nodes...'),

            # Obstacle Stop – reads /scan + /ultrasonic/*, outputs /cmd_vel
            Node(
                package='smart_cart_behaviour',
                executable='obstacle_stop_node',
                name='obstacle_stop_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),

            # Follow-Me – reads /scan, publishes /cmd_vel_raw
            Node(
                package='smart_cart_behaviour',
                executable='follow_me_node',
                name='follow_me_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),

            # Navigation – state machine (idle / follow / stop / manual)
            Node(
                package='smart_cart_navigation',
                executable='navigation_node',
                name='navigation_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'mode': 'follow',          # start in follow-me mode
                    'follow_distance': 1.0,
                    'max_speed': 0.8,
                }],
            ),
        ]),

        # ══════════════════════════════════════════════════════════════════
        # t=7  RVIZ2
        # ══════════════════════════════════════════════════════════════════
        TimerAction(period=7.0, actions=[
            LogInfo(msg='[SmartCart] Opening RViz2...'),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config]
                    if os.path.exists(rviz_config) else [],
                parameters=[{'use_sim_time': True}],
            ),
        ]),

    ])