import os
import pathlib
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('smart_cart_design_vignesh')
    urdf_file = os.path.join(pkg_path, 'urdf', 'smart_cart_3.urdf')

    # Replace package:// URIs with absolute file:// paths so Gazebo can find the STL meshes
    robot_urdf = pathlib.Path(urdf_file).read_text()
    robot_urdf = robot_urdf.replace(
        'package://smart_cart_design_vignesh',
        'file://' + pkg_path
    )

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_urdf}],
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'smart_cart_3',
                        '-topic', '/robot_description',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.3',
                    ],
                    output='screen'
                ),
            ],
        ),
    ])
