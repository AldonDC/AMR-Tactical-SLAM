#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as point_cloud2
import numpy as np
from std_msgs.msg import String
import json

class WebDataOptimizer(Node):
    """
    Optimiza datos pesados (LiDAR) para que Rosbridge los pueda 
    enviar a la web sin saturar el ancho de banda.
    """
    def __init__(self):
        super().__init__('web_bridge')
        
        # Suscripciones a datos crudos de SLAM
        self.create_subscription(PointCloud2, '/lidar/filtered', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, '/slam/path', self.path_callback, 10)
        
        # Publicación de datos optimizados para la Web (en formato JSON string)
        self.pub_web = self.create_publisher(String, '/web/dashboard_data', 10)
        
        self.get_logger().info("Web Data Optimizer (Rosbridge Partner) is ready")

    def lidar_callback(self, msg):
        try:
            # Leer puntos x,y,z
            gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            pts_struct = np.fromiter(gen, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
            if len(pts_struct) == 0: return
            
            # Decimación agresiva para fluidez en la web (1 de cada 8 puntos)
            pts_struct = pts_struct[::8]
            pts_flat = np.stack([pts_struct['x'], pts_struct['y'], pts_struct['z']], axis=1).flatten().tolist()
            
            self.send_to_web({
                'type': 'LIDAR',
                'points': pts_flat,
                'count': len(pts_struct)
            })
        except Exception as e:
            self.get_logger().error(f"Lidar processing error: {e}")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # Yaw de cuaternión
        yaw = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        
        self.send_to_web({
            'type': 'TELEMETRY',
            'x': pos.x, 'y': pos.y, 'z': pos.z,
            'yaw': yaw,
            'speed': float(np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2))
        })

    def path_callback(self, msg):
        # Simplificar el camino para la web
        path_pts = []
        for pose in msg.poses[::4]:
            path_pts.extend([pose.pose.position.x, pose.pose.position.y])
            
        self.send_to_web({
            'type': 'PATH',
            'points': path_pts
        })

    def send_to_web(self, data):
        msg = String()
        msg.data = json.dumps(data)
        self.pub_web.publish(msg)

def main():
    rclpy.init()
    node = WebDataOptimizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
