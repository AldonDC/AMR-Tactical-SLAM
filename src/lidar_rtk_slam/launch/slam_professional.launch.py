#!/usr/bin/env python3
# =============================================================================
# SLAM Professional Visualization Launch
# =============================================================================
# Waymo-style visualization with 3D vehicle model
# =============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_rtk_slam')
    
    # URDF del vehículo
    urdf_file = os.path.join(pkg_share, 'urdf', 'slam_vehicle.urdf')
    
    # Leer el contenido del URDF
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    
    # ==========================================================================
    # Launch Arguments
    # ==========================================================================
    declare_mode_arg = DeclareLaunchArgument(
        'mode', default_value='mapping',
        description='Operation mode: mapping or localization'
    )
    
    declare_bag_file_arg = DeclareLaunchArgument(
        'bag_file', default_value='',
        description='Path to rosbag2 file'
    )
    
    declare_config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'slam_params.yaml'),
        description='SLAM parameters'
    )
    
    # ==========================================================================
    # Configuration
    # ==========================================================================
    mode = LaunchConfiguration('mode')
    config_file = LaunchConfiguration('config_file')
    
    # ==========================================================================
    # Robot State Publisher (para el modelo 3D del vehículo)
    # ==========================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # ==========================================================================
    # Static TF Transforms
    # ==========================================================================
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0', '0', '0.75', '0', '0', '0', 'base_link', 'velodyne']
    )
    
    static_tf_base_to_rtk = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_rtk_tf',
        arguments=['-0.5', '0', '0.9', '0', '0', '0', 'base_link', 'rtk_antenna']
    )
    
    # ==========================================================================
    # SLAM Nodes
    # ==========================================================================
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
    
    tf_broadcaster_node = Node(
        package='lidar_rtk_slam',
        executable='tf_broadcaster_node',
        name='tf_broadcaster',
        output='screen',
        parameters=[config_file],
        remappings=[('input/odom', '/odom')]
    )
    
    # ==========================================================================
    # RViz Professional Visualization
    # ==========================================================================
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam_professional.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
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
        
        # Robot model
        robot_state_publisher,
        
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
        
        # Visualization (delayed start)
        TimerAction(
            period=2.0,
            actions=[rviz_node]
        ),
    ])
