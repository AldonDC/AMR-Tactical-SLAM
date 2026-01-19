import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg_slam = get_package_share_directory('lidar_rtk_slam')
    bag_path = '/home/alfonso/Documents/SEMESTRE TEC/pruebas_ros2_mapping&Localization/recorrido_2_bag'
    
    urdf_file = os.path.join(pkg_slam, 'urdf', 'car.urdf')
    rviz_file = os.path.join(pkg_slam, 'rviz', 'slam_pro.rviz')
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    return LaunchDescription([
        # 1. Robot Model (para ver el carro en RViz)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
        ),
        
        # 2. RTK Preprocessing (solo para posición, NO mapeo)
        Node(
            package='lidar_rtk_slam',
            executable='rtk_preprocessing_node',
            parameters=[{'use_sim_time': True}],
            remappings=[('input/fix', '/rtk/fix'), ('output/odom_enu', '/rtk/odom_enu')]
        ),
        
        # 3. Trajectory Tracker (traza V1/V2 sin hacer SLAM)
        Node(
            package='lidar_rtk_nav',
            executable='trajectory_tracker',
            output='screen'
        ),
        
        # 4. Static Map Server (carga el .ply SIN modificarlo)
        Node(
            package='lidar_rtk_nav',
            executable='static_map_server',
            parameters=[{'map_file': '/home/alfonso/Documents/SEMESTRE TEC/pruebas_ros2_mapping&Localization/auto_calibrated_map.ply'}]
        ),

        
        # 5. Motion Simulator (sigue bag, después responde a cmd_vel)
        Node(
            package='lidar_rtk_nav',
            executable='motion_simulator',
            output='screen'
        ),
        
        # 6. Autopilot (Pure Pursuit) - Usa /sim/odom después del bag
        Node(
            package='lidar_rtk_nav',
            executable='tactical_autopilot',
            output='screen',
            remappings=[('/odom', '/sim/odom')]
        ),
        
        # 7. TFs
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['0','0','0','0','0','0','map','odom']),
        
        # 8. WEB DASHBOARD (Interfaz Principal)
        Node(
            package='lidar_rtk_nav',
            executable='web_dashboard',
            output='screen'
        ),


        
        # 7. RViz (visualización 3D)
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': True}]
        ),
        
        # 8. BAG PLAYER
        TimerAction(
            period=2.0,
            actions=[ExecuteProcess(cmd=['ros2', 'bag', 'play', bag_path, '--clock'], output='screen')]
        )
    ])
