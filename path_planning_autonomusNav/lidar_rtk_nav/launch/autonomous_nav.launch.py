import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg_slam = get_package_share_directory('lidar_rtk_slam')
    
    urdf_file = os.path.join(pkg_slam, 'urdf', 'car.urdf')
    rviz_file = os.path.join(pkg_slam, 'rviz', 'slam_pro.rviz')
    bag_path = '/home/alfonso/Documents/SEMESTRE TEC/pruebas_ros2_mapping&Localization/recorrido_2_bag'
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    return LaunchDescription([
        # 1. Robot Model
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
        ),
        
        # 2. RTK Preprocessing (necesario para posición)
        Node(
            package='lidar_rtk_slam',
            executable='rtk_preprocessing_node',
            parameters=[{'use_sim_time': True}],
            remappings=[('input/fix', '/rtk/fix'), ('output/odom_enu', '/rtk/odom_enu')]
        ),
        
        # 3. Static Map Server
        Node(
            package='lidar_rtk_nav',
            executable='static_map_server',
            parameters=[{'map_file': '/home/alfonso/Documents/SEMESTRE TEC/pruebas_ros2_mapping&Localization/auto_calibrated_map.ply'}]
        ),
        
        # 4. Waypoint Collector
        Node(
            package='lidar_rtk_nav',
            executable='waypoint_collector',
            output='screen'
        ),
        
        # 5. Autopilot
        Node(
            package='lidar_rtk_nav',
            executable='tactical_autopilot',
            output='screen',
            remappings=[('/odom', '/rtk/odom_enu')]
        ),
        
        # 6. Dashboard de Métricas
        Node(
            package='lidar_rtk_nav',
            executable='controller_dashboard',
            output='screen'
        ),
        
        # 7. Vista Satelital (NUEVO)
        Node(
            package='lidar_rtk_nav',
            executable='mission_planner_satellite',
            output='screen'
        ),
        
        # 8. TFs
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['0','0','0','0','0','0','map','odom']),
        Node(package='lidar_rtk_slam', executable='tf_broadcaster_node', parameters=[{'use_sim_time': True}], remappings=[('/odom', '/rtk/odom_enu')]),
        
        # 9. RViz
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_file], parameters=[{'use_sim_time': True}]),
        
        # 10. BAG PLAYER (Para simular GPS)
        TimerAction(
            period=2.0,
            actions=[ExecuteProcess(cmd=['ros2', 'bag', 'play', bag_path, '--clock'], output='screen')]
        )
    ])
