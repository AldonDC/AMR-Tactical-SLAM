#!/usr/bin/env python3
# =============================================================================
# SLAM Offline Launch File
# =============================================================================
# Launches the complete SLAM pipeline for offline processing with rosbag2
# 
# Usage:
#   ros2 launch lidar_rtk_slam slam_offline.launch.py
#   ros2 launch lidar_rtk_slam slam_offline.launch.py bag_file:=/path/to/bag
#   ros2 launch lidar_rtk_slam slam_offline.launch.py mode:=localization
# =============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('lidar_rtk_slam')
    
    # ==========================================================================
    # Launch Arguments
    # ==========================================================================
    declare_mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='Operation mode: mapping (V1) or localization (V2)'
    )
    
    declare_bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='',
        description='Path to rosbag2 file for offline processing'
    )
    
    declare_config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'slam_params.yaml'),
        description='Path to SLAM parameters YAML file'
    )
    
    declare_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    declare_record_arg = DeclareLaunchArgument(
        'record_output',
        default_value='false',
        description='Record output topics to rosbag2'
    )
    
    # ==========================================================================
    # Configuration
    # ==========================================================================
    mode = LaunchConfiguration('mode')
    config_file = LaunchConfiguration('config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    record_output = LaunchConfiguration('record_output')
    
    # ==========================================================================
    # Static TF Transforms (sensor calibration)
    # ==========================================================================
    # base_link -> velodyne (LiDAR mounted on top)
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0', '0', '1.5', '0', '0', '0', 'base_link', 'velodyne']
    )
    
    # base_link -> rtk_antenna
    static_tf_base_to_rtk = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_rtk_tf',
        arguments=['-0.5', '0', '1.8', '0', '0', '0', 'base_link', 'rtk_antenna']
    )
    
    # ==========================================================================
    # SLAM Nodes (Python)
    # ==========================================================================
    
    # 1. LiDAR Preprocessing Node
    lidar_preprocessing_node = Node(
        package='lidar_rtk_slam',
        executable='lidar_preprocessing_node',
        name='lidar_preprocessing',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('input/pointcloud', '/velodyne_points'),
            ('output/filtered', '/lidar/filtered'),
            ('output/ground', '/lidar/ground')
        ]
    )
    
    # 2. RTK Preprocessing Node  
    rtk_preprocessing_node = Node(
        package='lidar_rtk_slam',
        executable='rtk_preprocessing_node',
        name='rtk_preprocessing',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('input/fix', '/rtk/fix'),
            ('output/odom_enu', '/rtk/odom_enu'),
            ('output/pose_enu', '/rtk/pose_enu')
        ]
    )
    
    # 3. Scan Matching Node (ICP/NDT)
    scan_matching_node = Node(
        package='lidar_rtk_slam',
        executable='scan_matching_node',
        name='scan_matching',
        output='screen',
        parameters=[config_file, {'mode': mode}],
        remappings=[
            ('input/pointcloud', '/lidar/filtered'),
            ('input/rtk_odom', '/rtk/odom_enu'),
            ('output/slam_odom', '/slam/odom'),
            ('output/relative_transform', '/slam/relative_transform')
        ]
    )
    
    # 4. RTK-SLAM Fusion Node
    fusion_node = Node(
        package='lidar_rtk_slam',
        executable='rtk_slam_fusion_node',
        name='rtk_slam_fusion',
        output='screen',
        parameters=[config_file, {'mode': mode}],
        remappings=[
            ('input/slam_odom', '/slam/odom'),
            ('input/rtk_odom', '/rtk/odom_enu'),
            ('output/fused_odom', '/odom'),
            ('output/diagnostics', '/slam/diagnostics')
        ]
    )
    
    # 5. Map Builder Node
    map_builder_node = Node(
        package='lidar_rtk_slam',
        executable='map_builder_node',
        name='map_builder',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('input/pointcloud', '/lidar/filtered'),
            ('input/odom', '/odom'),
            ('output/map', '/map'),
            ('output/keyframes', '/map/keyframes'),
            ('output/path', '/slam/path')
        ]
    )
    
    # 6. Loop Closure Node
    loop_closure_node = Node(
        package='lidar_rtk_slam',
        executable='loop_closure_node',
        name='loop_closure',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('input/odom', '/odom'),
            ('output/correction', '/loop_closure/correction'),
            ('output/markers', '/loop_closure/markers')
        ]
    )
    
    # 7. TF Broadcaster Node
    tf_broadcaster_node = Node(
        package='lidar_rtk_slam',
        executable='tf_broadcaster_node',
        name='tf_broadcaster',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('input/odom', '/odom')
        ]
    )
    
    # ==========================================================================
    # Visualization
    # ==========================================================================
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam_visualization.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)
    )
    
    # ==========================================================================
    # Output Recording
    # ==========================================================================
    recorder = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/odom', '/map', '/slam/path', '/tf', '/tf_static',
            '-o', 'slam_output'
        ],
        condition=IfCondition(record_output),
        output='screen'
    )
    
    # ==========================================================================
    # Launch Description
    # ==========================================================================
    return LaunchDescription([
        # Arguments
        declare_mode_arg,
        declare_bag_file_arg,
        declare_config_file_arg,
        declare_rviz_arg,
        declare_record_arg,
        
        # Static TFs
        static_tf_base_to_lidar,
        static_tf_base_to_rtk,
        
        # SLAM Pipeline
        lidar_preprocessing_node,
        rtk_preprocessing_node,
        scan_matching_node,
        fusion_node,
        map_builder_node,
        loop_closure_node,
        tf_broadcaster_node,
        
        # Visualization
        rviz_node,
        
        # Recording
        recorder,
    ])
