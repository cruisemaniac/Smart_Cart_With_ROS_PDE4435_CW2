# from setuptools import find_packages, setup

# package_name = 'smart_cart_behaviour'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='jayashanka',
#     maintainer_email='jayashanka@todo.todo',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     extras_require={
#         'test': [
#             'pytest',
#         ],
#     },
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )

from setuptools import setup

package_name = 'smart_cart_behaviour'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Smart Cart follow-me and obstacle avoidance nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'obstacle_stop_node = smart_cart_behaviour.obstacle_stop_node:main',
            'follow_me_node     = smart_cart_behaviour.follow_me_node:main',
        ],
    },
)