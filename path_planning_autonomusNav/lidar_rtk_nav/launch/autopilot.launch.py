from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_rtk_nav',
            executable='tactical_autopilot',
            name='tactical_autopilot',
            output='screen',
            parameters=[{
                'v_linear': 0.5,
                'lookahead': 2.0,
                'goal_tolerance': 1.0
            }]
        )
    ])
