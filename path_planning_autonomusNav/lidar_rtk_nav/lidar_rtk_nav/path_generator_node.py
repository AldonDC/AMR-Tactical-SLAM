import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math

class PathGenerator(Node):
    """
    Path Generator Node.
    Listen to '2D Nav Goal' from RViz and create a path.
    """
    def __init__(self):
        super().__init__('path_generator')
        
        self.current_pos = None
        
        # Suscribirse a la posición actual (SLAM)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
            
        # Suscribirse al botón "2D Nav Goal" de RViz
        self.sub_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
            
        self.pub_path = self.create_publisher(Path, '/navigation/path', 10)
        
        self.get_logger().info('� Navegación "Click & Go" Activa')
        self.get_logger().info('   Simplemente usa el botón "2D Nav Goal" en RViz para moverte.')

    def odom_callback(self, msg: Odometry):
        self.current_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def goal_callback(self, msg: PoseStamped):
        """Genera una trayectoria recta desde el robot hasta el clic en RViz."""
        if self.current_pos is None:
            self.get_logger().warn('No tengo posición actual del SLAM todavía.')
            return

        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        
        self.get_logger().info(f'📍 Nuevo objetivo recibido: X={target_x:.2f}, Y={target_y:.2f}')
        
        # Generar puntos intermedios cada 0.5m
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        dx = target_x - self.current_pos[0]
        dy = target_y - self.current_pos[1]
        dist = math.sqrt(dx**2 + dy**2)
        num_pts = int(dist / 0.5)
        
        for i in range(num_pts + 1):
            t = i / num_pts
            pose = PoseStamped()
            pose.pose.position.x = self.current_pos[0] + dx * t
            pose.pose.position.y = self.current_pos[1] + dy * t
            path_msg.poses.append(pose)
            
        self.pub_path.publish(path_msg)
        self.get_logger().info('✅ Trayectoria enviada al controlador.')

    def add_waypoint(self, pos):
        self.waypoints.append(pos)
        self.last_recorded_pos = pos
        self.get_logger().info(f'📍 Waypoint added: {pos}')
        self.publish_path()

    def publish_path(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        for pt in self.waypoints:
            ps = PoseStamped()
            ps.pose.position.x = pt[0]
            ps.pose.position.y = pt[1]
            msg.poses.append(ps)
            
        self.pub_path.publish(msg)

    def save_path(self):
        with open(self.path_file, 'w') as f:
            json.dump(self.waypoints, f)
        self.get_logger().info(f'💾 Path saved to {self.path_file}')

    def load_path(self):
        if os.path.exists(self.path_file):
            try:
                with open(self.path_file, 'r') as f:
                    self.waypoints = json.load(f)
                self.get_logger().info(f'📂 Loaded {len(self.waypoints)} waypoints from {self.path_file}')
                self.publish_path()
            except:
                self.get_logger().warn('Failed to load path file.')

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
