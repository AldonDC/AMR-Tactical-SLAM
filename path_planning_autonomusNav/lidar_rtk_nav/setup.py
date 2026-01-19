from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lidar_rtk_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alfonso',
    maintainer_email='a00838034@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit_controller = lidar_rtk_nav.pure_pursuit_controller:main',
            'path_generator = lidar_rtk_nav.path_generator_node:main',
            'tactical_autopilot = lidar_rtk_nav.tactical_autopilot:main',
            'static_map_server = lidar_rtk_nav.static_map_server:main',
            'mission_planner_satellite = lidar_rtk_nav.mission_planner_satellite:main',
            'mission_planner_gui = lidar_rtk_nav.mission_planner_gui:main',
            'waypoint_collector = lidar_rtk_nav.waypoint_collector:main',
            'controller_dashboard = lidar_rtk_nav.controller_dashboard:main',
            'tactical_mission_control = lidar_rtk_nav.tactical_mission_control:main',
            'trajectory_tracker = lidar_rtk_nav.trajectory_tracker:main',
            'motion_simulator = lidar_rtk_nav.motion_simulator:main',
            'web_dashboard = lidar_rtk_nav.web_dashboard:main',
        ],
    },
)


