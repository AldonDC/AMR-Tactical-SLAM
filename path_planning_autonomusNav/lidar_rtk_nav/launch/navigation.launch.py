import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 🟢 Generador de Rutas
        Node(
            package='lidar_rtk_nav',
            executable='path_generator',
            name='path_generator',
            output='screen',
            parameters=[{'path_file': 'mission_path.json'}]
        ),
        
        # 🏎️ Controlador Pure Pursuit
        Node(
            package='lidar_rtk_nav',
            executable='pure_pursuit_controller',
            name='pure_pursuit_controller',
            output='screen',
            parameters=[{
                'lookahead_distance': 2.5,
                'linear_velocity': 0.6,
                'max_angular_velocity': 1.2,
                'goal_tolerance': 1.0
            }]
        )
    ])
