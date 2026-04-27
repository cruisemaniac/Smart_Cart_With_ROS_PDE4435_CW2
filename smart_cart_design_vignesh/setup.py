from setuptools import setup

package_name = 'smart_cart_design_vignesh'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='Smart cart control',
    license='TODO',
    entry_points={
        'console_scripts': [
            'teleop_cart = smart_cart_design_vignesh.teleop_cart:main',
        ],
    },
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),

    ('share/' + package_name, ['package.xml']),

    # ✅ launch files
    ('share/' + package_name + '/launch', ['launch/gazebo.launch.py']),

    # ✅ URDF files
    ('share/' + package_name + '/urdf', ['urdf/smart_cart_3.urdf']),

    # ✅ mesh files
    ('share/' + package_name + '/meshes', [
        'meshes/base_link.STL',
        'meshes/FL_wheel.STL',
        'meshes/FR_wheel.STL',
        'meshes/RL_wheel.STL',
        'meshes/RR_wheel.STL',
    ]),
    ],
)
