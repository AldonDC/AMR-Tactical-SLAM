"""
RTK-Direct SLAM Launch File

This launch file uses the RTK-Direct approach where:
- RTK GPS provides the precise position (X, Y, Z)
- Heading is computed from movement direction
- LiDAR points are transformed directly to global frame

This eliminates scan matching drift and produces clean maps.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_rtk_slam')
    urdf_file = os.path.join(pkg_share, 'urdf', 'car.urdf')
    rviz_file = os.path.join(pkg_share, 'rviz', 'slam_pro.rviz')
    rviz_mission_file = os.path.join(pkg_share, 'rviz', 'mission_control.rviz')
    bag_path = '/home/alfonso/Documents/SEMESTRE TEC/pruebas_ros2_mapping&Localization/recorrido_2_bag'
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    sim_time = [{'use_sim_time': True}]
    
    return LaunchDescription([
        # 1. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
        ),
        
        # 2. LiDAR Preprocessing
        Node(
            package='lidar_rtk_slam',
            executable='lidar_preprocessing_node',
            parameters=sim_time,
            remappings=[
                ('input/pointcloud', '/velodyne_points'),
                ('output/filtered', '/lidar/filtered')
            ]
        ),
        
        # 3. RTK Preprocessing
        Node(
            package='lidar_rtk_slam',
            executable='rtk_preprocessing_node',
            parameters=sim_time,
            remappings=[
                ('input/fix', '/rtk/fix'),
                ('output/odom_enu', '/rtk/odom_enu')
            ]
        ),
        
        # 4. SLAM Map Builder
        Node(
            package='lidar_rtk_slam',
            executable='rtk_direct_map_builder',
            name='rtk_direct_map_builder',
            parameters=sim_time + [{
                'lidar_offset_deg': 90.0,
            }]
        ),
        
        # 5. TF: map -> odom (static)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=sim_time
        ),

        # 5b. TF: base_link -> rtk_antenna (static)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'rtk_antenna'],
            parameters=sim_time
        ),
        
        # 6. TF Broadcaster
        Node(
            package='lidar_rtk_slam',
            executable='tf_broadcaster_node',
            parameters=sim_time
        ),
        
        # 7. RViz 1: [HD 3D MAPPING]
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_3d',
            arguments=['-d', rviz_file],
            parameters=sim_time
        ),
        
        # 8. SATELLITE MISSION CONTROL (Python / Matplotlib)
        Node(
            package='lidar_rtk_slam',
            executable='satellite_navigation_node',
            name='satellite_mission_control',
            parameters=sim_time
        ),
        
        # 9. Play bag
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'play', bag_path, '--clock'],
                    output='screen'
                )
            ]
        )
    ])
