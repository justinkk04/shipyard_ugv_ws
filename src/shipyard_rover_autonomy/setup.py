from setuptools import setup

package_name = 'shipyard_rover_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='justin',
    maintainer_email='justin@example.com',
    description='Simple ROS 2 autonomy nodes for the Shipyard UGV simulation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager = shipyard_rover_autonomy.mission_manager_node:main',
            'route_follower = shipyard_rover_autonomy.route_follower_node:main',
            'obstacle_avoidance = shipyard_rover_autonomy.obstacle_avoidance_node:main',
            'telemetry_logger = shipyard_rover_autonomy.telemetry_logger_node:main',
        ],
    },
)