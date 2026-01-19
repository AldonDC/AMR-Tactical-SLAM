#!/usr/bin/env python3
"""
Trajectory Tracker - Solo traza trayectorias V1/V2 sin hacer mapeo SLAM
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy

class TrajectoryTracker(Node):
    def __init__(self):
        super().__init__('trajectory_tracker')
        
        # Trayectorias
        self.trajectory_v1 = []  # Vuelta 1 (Mapping)
        self.trajectory_v2 = []  # Vuelta 2 (Localization)
        
        # Detección de vueltas
        self.lap_distance_threshold = 5.0  # metros para detectar vuelta completa
        self.start_position = None
        self.current_lap = 1
        self.min_distance_before_lap = 30.0  # Distancia mínima antes de contar vuelta
        self.distance_traveled = 0.0
        self.last_position = None
        
        # QoS
        self.sub_odom = self.create_subscription(
            Odometry, '/rtk/odom_enu', self.odom_callback, 10)
        
        # Publishers
        self.pub_path_v1 = self.create_publisher(Path, '/slam/path_mapping', 10)
        self.pub_path_v2 = self.create_publisher(Path, '/slam/path_localization', 10)
        
        # Timer para publicar
        self.create_timer(0.5, self.publish_paths)
        
        self.get_logger().info('📍 Trajectory Tracker iniciado')
        self.get_logger().info('   V1 (Naranja) = Primera vuelta')
        self.get_logger().info('   V2 (Cyan) = Segunda vuelta')
    
    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Guardar posición inicial
        if self.start_position is None:
            self.start_position = (x, y)
            self.last_position = (x, y)
            self.get_logger().info(f'🏁 Posición inicial: ({x:.2f}, {y:.2f})')
        
        # Calcular distancia recorrida
        if self.last_position:
            dx = x - self.last_position[0]
            dy = y - self.last_position[1]
            self.distance_traveled += (dx**2 + dy**2)**0.5
            self.last_position = (x, y)
        
        # Detectar cambio de vuelta
        if self.current_lap == 1 and self.distance_traveled > self.min_distance_before_lap:
            dist_to_start = ((x - self.start_position[0])**2 + (y - self.start_position[1])**2)**0.5
            if dist_to_start < self.lap_distance_threshold:
                self.current_lap = 2
                self.get_logger().info('🔄 VUELTA 2 DETECTADA - Cambiando a trayectoria V2')
        
        # Crear pose
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        # Agregar a la trayectoria correspondiente
        if self.current_lap == 1:
            self.trajectory_v1.append(pose)
        else:
            self.trajectory_v2.append(pose)
    
    def publish_paths(self):
        # Publicar V1
        if self.trajectory_v1:
            path_v1 = Path()
            path_v1.header.stamp = self.get_clock().now().to_msg()
            path_v1.header.frame_id = 'map'
            path_v1.poses = self.trajectory_v1
            self.pub_path_v1.publish(path_v1)
        
        # Publicar V2
        if self.trajectory_v2:
            path_v2 = Path()
            path_v2.header.stamp = self.get_clock().now().to_msg()
            path_v2.header.frame_id = 'map'
            path_v2.poses = self.trajectory_v2
            self.pub_path_v2.publish(path_v2)

def main():
    rclpy.init()
    node = TrajectoryTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
