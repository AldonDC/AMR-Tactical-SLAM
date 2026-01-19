#!/usr/bin/env python3
"""
Motion Simulator - Simula movimiento del robot respondiendo a /cmd_vel
Después de que el bag termina, el robot puede moverse hacia los waypoints.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class MotionSimulator(Node):
    def __init__(self):
        super().__init__('motion_simulator')
        
        # Estado del robot
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.initialized = False
        
        # Velocidades actuales
        self.v_linear = 0.0
        self.v_angular = 0.0
        
        # Control de bag
        self.bag_active = True
        self.last_bag_time = self.get_clock().now()
        self.bag_timeout = 3.0  # segundos sin datos = bag terminó
        
        # Suscriptores
        self.sub_odom_bag = self.create_subscription(
            Odometry, '/rtk/odom_enu', self.odom_bag_callback, 10)
        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # Publicadores
        self.pub_odom = self.create_publisher(Odometry, '/sim/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer de simulación (20 Hz)
        self.dt = 0.05
        self.create_timer(self.dt, self.update_simulation)
        
        self.get_logger().info('🎮 Motion Simulator iniciado')
        self.get_logger().info('   Mientras el bag corre: sigue la posición del RTK')
        self.get_logger().info('   Después del bag: responde a /cmd_vel')
    
    def odom_bag_callback(self, msg: Odometry):
        """Recibe posición del bag (RTK)."""
        self.last_bag_time = self.get_clock().now()
        
        if self.bag_active:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            self.yaw = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))
            
            if not self.initialized:
                self.initialized = True
                self.get_logger().info(f'📍 Posición inicial: ({self.x:.2f}, {self.y:.2f})')
    
    def cmd_callback(self, msg: Twist):
        """Recibe comandos de velocidad."""
        self.v_linear = msg.linear.x
        self.v_angular = msg.angular.z
    
    def update_simulation(self):
        """Actualiza la simulación."""
        if not self.initialized:
            return
        
        # Verificar si el bag terminó
        time_since_bag = (self.get_clock().now() - self.last_bag_time).nanoseconds / 1e9
        
        if time_since_bag > self.bag_timeout and self.bag_active:
            self.bag_active = False
            self.get_logger().info('🏁 Bag terminado - Ahora puedes navegar con waypoints!')
            self.get_logger().info(f'📍 Última posición: ({self.x:.2f}, {self.y:.2f})')
        
        # Si el bag no está activo, simular movimiento
        if not self.bag_active:
            # Modelo cinemático simple (diferencial)
            self.x += self.v_linear * math.cos(self.yaw) * self.dt
            self.y += self.v_linear * math.sin(self.yaw) * self.dt
            self.yaw += self.v_angular * self.dt
        
        # Publicar odometría simulada
        self.publish_odom()
    
    def publish_odom(self):
        """Publica odometría y TF."""
        now = self.get_clock().now()
        
        # Odometría
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2)
        
        odom.twist.twist.linear.x = self.v_linear
        odom.twist.twist.angular.z = self.v_angular
        
        self.pub_odom.publish(odom)
        
        # TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.yaw / 2)
        t.transform.rotation.w = math.cos(self.yaw / 2)
        
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = MotionSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
