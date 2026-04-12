from setuptools import find_packages, setup

package_name = 'smart_cart_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jayashanka',
    maintainer_email='jayasankaanushan199@gmail.com',
    description='Navigation and UWB simulation for Smart Cart',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'navigation_node     = smart_cart_navigation.navigation_node:main',
            'uwb_simulator_node  = smart_cart_navigation.uwb_simulator_node:main',
        ],
    },
)