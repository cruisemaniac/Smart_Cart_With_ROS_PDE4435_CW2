from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_name = 'smart_cart_design_vignesh'
    pkg_path = get_package_share_directory(pkg_name)

    urdf_file = os.path.join(pkg_path, 'urdf', 'smart_cart_3.urdf')

    return LaunchDescription([

        # Start Gazebo (Harmonic)
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v', '4', 'empty.sdf'],
            output='screen'
        ),

        # Spawn robot into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'smart_cart_3',
                '-file', urdf_file,
                '-x', '0',
                '-y', '0',
                '-z', '0.2'
            ],
            output='screen'
        ),
    ])
