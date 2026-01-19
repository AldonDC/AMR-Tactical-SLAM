#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class WaypointCollector(Node):
    """
    Recolecta waypoints desde RViz (botón Publish Point).
    """
    def __init__(self):
        super().__init__('waypoint_collector')
        
        self.waypoints = []
        
        # Suscriptor al botón "Publish Point" de RViz
        self.sub_point = self.create_subscription(
            PointStamped, '/clicked_point', self.point_callback, 10)
        
        # Publisher de la ruta
        self.pub_path = self.create_publisher(Path, '/navigation/path', 10)
        
        self.get_logger().info('🎯 Waypoint Collector Activo')
        self.get_logger().info('   Usa "Publish Point" en RViz para agregar waypoints')
        self.get_logger().info('   Escribe "send" en terminal para enviar misión')
    
    def point_callback(self, msg: PointStamped):
        """Recibe punto de RViz y lo agrega a la lista."""
        x, y = msg.point.x, msg.point.y
        self.waypoints.append((x, y))
        
        self.get_logger().info(f'✅ Waypoint {len(self.waypoints)}: X={x:.2f}, Y={y:.2f}')
        
        # Publicar path actualizado
        self.publish_path()
    
    def publish_path(self):
        """Publica la ruta completa."""
        if not self.waypoints:
            return
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for x, y in self.waypoints:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            path_msg.poses.append(pose)
        
        self.pub_path.publish(path_msg)
    
    def clear_waypoints(self):
        """Limpia todos los waypoints."""
        self.waypoints.clear()
        self.get_logger().info('🗑️ Waypoints limpiados')

def main():
    rclpy.init()
    node = WaypointCollector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
