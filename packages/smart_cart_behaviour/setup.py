from setuptools import setup

package_name = 'smart_cart_behaviour'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jayashanka',
    maintainer_email='jayasankaanushan199@gmail.com',
    description='Smart Cart behaviour nodes – follow-me, obstacle stop, person teleop',
    license='MIT',

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/start_all.launch.py']),
    ],

    entry_points={
        'console_scripts': [
            'obstacle_stop_node  = smart_cart_behaviour.obstacle_stop_node:main',
            'follow_me_node      = smart_cart_behaviour.follow_me_node:main',
            'teleop_person_node  = smart_cart_behaviour.teleop_person_node:main',
            'cart_teleop_node  = smart_cart_behaviour.cart_teleop_node:main',
        ],
    },
)
