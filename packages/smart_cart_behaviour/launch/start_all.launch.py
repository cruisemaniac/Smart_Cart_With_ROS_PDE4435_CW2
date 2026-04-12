"""
start_all.launch.py
════════════════════════════════════════════════════════════
Single command: ros2 launch smart_cart_behaviour start_all.launch.py

After launch completes, open a second terminal:
  ros2 run smart_cart_behaviour teleop_person_node

Controls:
  W/S    move person forward/backward
  A/D    turn person left/right
  1      remote STOP   – cart stops
  2      remote FOLLOW – cart follows person
  3      remote IDLE   – cart standby
  +/-    person speed
  ESC    quit
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
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

        LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),
        LogInfo(msg='  Smart Cart Simulation – UWB + LiDAR Follow-Me'),
        LogInfo(msg='  Cart starts IDLE. Run teleop in a second terminal.'),
        LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),

        # ── t=0  Gazebo ──────────────────────────────────────────────────
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

        # ── t=0  Robot State Publisher ───────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_urdf,
                         'use_sim_time': False}],
        ),

        # ── t=3  Spawn smart cart ────────────────────────────────────────
        TimerAction(period=3.0, actions=[
            LogInfo(msg='[1/5] Spawning Smart Cart...'),
            Node(
                package='ros_gz_sim', executable='create',
                name='spawn_smart_cart',
                arguments=[
                    '-name', 'smart_cart',
                    '-topic', '/robot_description',
                    '-x', '0.0', '-y', '0.0', '-z', '0.20',
                    '-R', '0.0', '-P', '0.0', '-Y', '0.0',
                ],
                output='screen',
            ),
        ]),

        # ── t=4  Spawn person ────────────────────────────────────────────
        # Spawned at x=-2.0 (in front of cart based on cart orientation)
        TimerAction(period=4.0, actions=[
            LogInfo(msg='[2/5] Spawning person actor...'),
            Node(
                package='ros_gz_sim', executable='create',
                name='spawn_person',
                arguments=[
                    '-name', 'person',
                    '-file', person_sdf_file,
                    '-x', '-2.0', '-y', '0.0', '-z', '0.12',
                    '-R', '0.0', '-P', '0.0', '-Y', '0.0',
                ],
                output='screen',
            ),
        ]),

        # ── t=5  ROS-Gz bridge ───────────────────────────────────────────
        TimerAction(period=5.0, actions=[
            LogInfo(msg='[3/5] Starting ROS-Gz bridge...'),
            Node(
                package='ros_gz_bridge', executable='parameter_bridge',
                name='ros_gz_bridge', output='screen',
                parameters=[{'config_file': bridge_config}],
            ),
        ]),

        # ── t=6  All ROS 2 nodes ─────────────────────────────────────────
        TimerAction(period=6.0, actions=[
            LogInfo(msg='[4/5] Starting all nodes...'),

            # Obstacle stop (safety layer)
            Node(
                package='smart_cart_behaviour',
                executable='obstacle_stop_node',
                name='obstacle_stop_node',
                output='screen',
                parameters=[{'use_sim_time': False}],
            ),

            # Follow-Me (UWB distance + LiDAR angle)
            Node(
                package='smart_cart_behaviour',
                executable='follow_me_node',
                name='follow_me_node',
                output='screen',
                parameters=[{'use_sim_time': False}],
            ),

            # Navigation state machine
            Node(
                package='smart_cart_navigation',
                executable='navigation_node',
                name='navigation_node',
                output='screen',
                parameters=[{
                    'use_sim_time':    False,
                    'mode':            'idle',
                    'follow_distance': 1.0,
                    'max_speed':       0.8,
                    'target_timeout':  5.0,
                }],
            ),

            # UWB simulator (trilateration via odometry)
            Node(
                package='smart_cart_navigation',
                executable='uwb_simulator_node',
                name='uwb_simulator_node',
                output='screen',
                parameters=[{'use_sim_time': False}],
            ),
        ]),

        # ── t=7  RViz2 ───────────────────────────────────────────────────
        TimerAction(period=7.0, actions=[
            Node(
                package='rviz2', executable='rviz2', name='rviz2',
                output='screen',
                arguments=['-d', rviz_config]
                    if os.path.exists(rviz_config) else [],
                parameters=[{'use_sim_time': False}],
            ),
        ]),

        # ── t=8  Instructions ────────────────────────────────────────────
        TimerAction(period=8.0, actions=[
            LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),
            LogInfo(msg='  READY. Open a new terminal and run:'),
            LogInfo(msg='  ros2 run smart_cart_behaviour teleop_person_node'),
            LogInfo(msg='  W/S=move  A/D=turn  2=FOLLOW  1=STOP  3=IDLE'),
            LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),
        ]),

    ])
