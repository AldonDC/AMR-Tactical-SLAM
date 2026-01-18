from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'lidar_rtk_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # RViz files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # Dashboard files
        (os.path.join('share', package_name, 'www'), glob('www/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alfonso',
    maintainer_email='alfonso@example.com',
    description='Professional 3D LiDAR-RTK SLAM for ROS 2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_preprocessing_node = lidar_rtk_slam.lidar_preprocessing_node:main',
            'rtk_preprocessing_node = lidar_rtk_slam.rtk_preprocessing_node:main',
            'scan_matching_node = lidar_rtk_slam.scan_matching_node:main',
            'rtk_slam_fusion_node = lidar_rtk_slam.rtk_slam_fusion_node:main',
            'map_builder_node = lidar_rtk_slam.map_builder_node:main',
            'loop_closure_node = lidar_rtk_slam.loop_closure_node:main',
            'tf_broadcaster_node = lidar_rtk_slam.tf_broadcaster_node:main',
            'web_bridge_node = lidar_rtk_slam.web_bridge_node:main',
            'viz_3d_node = lidar_rtk_slam.viz_3d_node:main',
            'rtk_direct_map_builder = lidar_rtk_slam.rtk_direct_map_builder:main',
            'satellite_navigation_node = lidar_rtk_slam.satellite_navigation_node:main',
        ],
    },
)
