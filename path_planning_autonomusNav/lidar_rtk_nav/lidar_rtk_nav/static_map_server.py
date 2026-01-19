import rclpy
from rclpy.node import Node
import open3d as o3d
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, DurabilityPolicy

class StaticMapServer(Node):
    """
    Publica un archivo .ply guardado para que aparezca en RViz 
    sin necesidad de correr el SLAM completo.
    """
    def __init__(self):
        super().__init__('static_map_server')
        self.declare_parameter('map_file', 'mapas_ply/auto_calibrated_map.ply')
        map_path = self.get_parameter('map_file').value
        
        # QoS para que el mapa se quede "pegado" en RViz
        latched_qos = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.pub_map = self.create_publisher(PointCloud2, '/slam/map', latched_qos)
        
        try:
            self.get_logger().info(f'📂 Cargando mapa: {map_path}...')
            pcd = o3d.io.read_point_cloud(map_path)
            points = np.asarray(pcd.points)
            colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)
            
            # Formato [x, y, z, rgb]
            cloud_data = np.zeros(len(points), dtype=[
                ('x', np.float32), ('y', np.float32), ('z', np.float32),
                ('rgb', np.uint32)
            ])
            cloud_data['x'] = points[:, 0]
            cloud_data['y'] = points[:, 1]
            cloud_data['z'] = points[:, 2]
            
            # Empaquetar colores RGB en un solo uint32 para PointCloud2
            rgb_packed = (colors[:, 0].astype(np.uint32) << 16) | \
                         (colors[:, 1].astype(np.uint32) << 8) | \
                         (colors[:, 2].astype(np.uint32))
            cloud_data['rgb'] = rgb_packed

            header = Header()
            header.frame_id = 'map'
            header.stamp = self.get_clock().now().to_msg()
            
            self.msg = pc2.create_cloud_xyz32(header, points) # Simplificado para demo
            # Para mantener colores usamos una versión más completa si es necesario
            
            self.get_logger().info('✅ Mapa cargado con éxito. Publicando en /slam/map...')
            self.create_timer(2.0, self.timer_callback) # Publicar ocasionalmente para nuevos RViz
            
        except Exception as e:
            self.get_logger().error(f'❌ Error al cargar el mapa: {str(e)}')

    def timer_callback(self):
        self.pub_map.publish(self.msg)

def main():
    rclpy.init()
    node = StaticMapServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
