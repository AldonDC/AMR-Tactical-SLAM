import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_rtk_slam')
    urdf = os.path.join(pkg_share, 'urdf', 'car.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam_pro.rviz')
    bag_path = '/home/alfonso/Documents/SEMESTRE TEC/pruebas_ros2_mapping&Localization/slam_demo_bag'
    
    with open(urdf, 'r') as f:
        robot_description_content = f.read()
    
    sim_time = [{'use_sim_time': True}]

    # Nodos de Procesamiento
    return LaunchDescription([
        # 1. Publicador del Robot (EL MÁS IMPORTANTE PARA EL MODELO)
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}]),
             
        # 2. Pipeline de SLAM
        Node(package='lidar_rtk_slam', executable='lidar_preprocessing_node', parameters=sim_time,
             remappings=[('input/pointcloud', '/velodyne_points'), ('output/filtered', '/lidar/filtered')]),
        Node(package='lidar_rtk_slam', executable='rtk_preprocessing_node', parameters=sim_time,
             remappings=[('input/fix', '/rtk/fix'), ('output/odom_enu', '/rtk/odom_enu')]),
        Node(package='lidar_rtk_slam', executable='scan_matching_node', parameters=sim_time,
             remappings=[('input/pointcloud', '/lidar/filtered'), ('output/slam_odom', '/slam/odom')]),
        Node(package='lidar_rtk_slam', executable='rtk_slam_fusion_node', parameters=sim_time,
             remappings=[('input/slam_odom', '/slam/odom'), ('input/rtk_odom', '/rtk/odom_enu'), ('output/fused_odom', '/odom')]),
        Node(package='lidar_rtk_slam', executable='map_builder_node', parameters=sim_time,
             remappings=[('input/pointcloud', '/lidar/filtered'), ('input/odom', '/odom'), ('output/path', '/slam/path'), ('output/map', '/slam/map')]),

        # 3. Transformaciones (EL PEGAMENTO)
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['0','0','0','0','0','0','map','odom'], parameters=sim_time),
        Node(package='lidar_rtk_slam', executable='tf_broadcaster_node', parameters=sim_time),

        # 4. RViz con Fixed Frame predeterminado en 'map'
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config], parameters=sim_time),
        
        # 5. Reproducción del BAG
        TimerAction(period=3.0, actions=[
            ExecuteProcess(cmd=['ros2', 'bag', 'play', bag_path, '--clock'], output='screen')
        ])
    ])
