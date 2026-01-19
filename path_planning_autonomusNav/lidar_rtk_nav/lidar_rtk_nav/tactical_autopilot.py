#!/usr/bin/env python3
"""
PURE PURSUIT CONTROLLER - Modelo de Bicicleta Cinemático
Implementación correcta para navegación autónoma AMR

Referencias:
- Coulter, R. Craig. "Implementation of the Pure Pursuit Path Tracking Algorithm"
- Snider, Jarrod M. "Automatic Steering Methods for Autonomous Automobile Path Tracking"
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32MultiArray
import math
import numpy as np

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('tactical_autopilot')
        
        # ================== PARÁMETROS DEL VEHÍCULO ==================
        self.declare_parameter('wheelbase', 1.0)          # Distancia entre ejes (metros)
        self.declare_parameter('max_steering_deg', 17.5)  # Ángulo máximo de giro (grados)
        self.declare_parameter('max_velocity', 1.0)       # Velocidad máxima (m/s)
        self.declare_parameter('min_velocity', 0.3)       # Velocidad mínima (m/s)
        self.declare_parameter('goal_tolerance', 2.0)     # Tolerancia para waypoint (metros)
        
        # Parámetros de Lookahead
        self.declare_parameter('lookahead_gain', 0.5)     # k en Ld = k*v + Ld_min
        self.declare_parameter('lookahead_min', 2.0)      # Distancia mínima de lookahead
        self.declare_parameter('lookahead_max', 6.0)      # Distancia máxima de lookahead
        
        # Cargar parámetros
        self.L = self.get_parameter('wheelbase').value
        self.max_steer = math.radians(self.get_parameter('max_steering_deg').value)
        self.v_max = self.get_parameter('max_velocity').value
        self.v_min = self.get_parameter('min_velocity').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.ld_gain = self.get_parameter('lookahead_gain').value
        self.ld_min = self.get_parameter('lookahead_min').value
        self.ld_max = self.get_parameter('lookahead_max').value
        
        # ================== ESTADO ==================
        self.current_pose = None  # (x, y, yaw)
        self.current_velocity = 0.0
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.active = False
        
        # ================== ROS2 INTERFACES ==================
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, '/navigation/path', self.path_callback, 10)
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_metrics = self.create_publisher(Float32MultiArray, '/autopilot/metrics', 10)
        
        # Timer de control a 20 Hz
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('═══════════════════════════════════════════')
        self.get_logger().info('🎯 PURE PURSUIT CONTROLLER (Bicycle Model)')
        self.get_logger().info(f'   Wheelbase: {self.L:.2f} m')
        self.get_logger().info(f'   Max Steering: ±{math.degrees(self.max_steer):.1f}°')
        self.get_logger().info(f'   Velocity Range: {self.v_min:.1f} - {self.v_max:.1f} m/s')
        self.get_logger().info(f'   Lookahead: {self.ld_min:.1f} - {self.ld_max:.1f} m')
        self.get_logger().info('═══════════════════════════════════════════')
    
    # ================== CALLBACKS ==================
    def odom_callback(self, msg: Odometry):
        """Actualiza la pose y velocidad actual del robot."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extraer yaw del quaternion
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))
        
        self.current_pose = (x, y, yaw)
        self.current_velocity = math.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )
    
    def path_callback(self, msg: Path):
        """Recibe los waypoints de navegación."""
        if len(msg.poses) == 0:
            self.waypoints = []
            self.active = False
            self.pub_cmd.publish(Twist())
            self.get_logger().info('⏹️ Navegación detenida')
            return
        
        self.waypoints = [
            (p.pose.position.x, p.pose.position.y) 
            for p in msg.poses
        ]
        self.current_waypoint_idx = 0
        self.active = True
        self.get_logger().info(f'🎯 Recibidos {len(self.waypoints)} waypoints')
    
    # ================== PURE PURSUIT ==================
    def compute_lookahead_distance(self):
        """
        Calcula la distancia de lookahead dinámica.
        Ld = k * v + Ld_min (limitado por Ld_max)
        """
        ld = self.ld_gain * self.current_velocity + self.ld_min
        return max(self.ld_min, min(self.ld_max, ld))
    
    def find_lookahead_point(self, x, y):
        """
        Encuentra el punto de lookahead en la línea hacia el waypoint.
        
        Para navegación punto-a-punto (sin path continuo),
        el lookahead point es el waypoint actual si está más lejos que Ld,
        o un punto intermedio si está más cerca.
        """
        if self.current_waypoint_idx >= len(self.waypoints):
            return None, 0
        
        target = self.waypoints[self.current_waypoint_idx]
        dx = target[0] - x
        dy = target[1] - y
        dist_to_target = math.sqrt(dx**2 + dy**2)
        
        ld = self.compute_lookahead_distance()
        
        if dist_to_target <= ld:
            # El waypoint está dentro del círculo de lookahead
            # Usar el waypoint directamente
            return target, dist_to_target
        else:
            # El waypoint está fuera, usar punto en la línea a distancia Ld
            ratio = ld / dist_to_target
            lp_x = x + dx * ratio
            lp_y = y + dy * ratio
            return (lp_x, lp_y), ld
    
    def compute_steering_angle(self, x, y, yaw, lookahead_point, ld):
        """
        Calcula el ángulo de steering usando la geometría del modelo de bicicleta.
        
        Fórmula Pure Pursuit:
        δ = atan(2 * L * sin(α) / Ld)
        
        donde:
        - L = wheelbase
        - α = ángulo entre heading del vehículo y lookahead point
        - Ld = distancia de lookahead
        """
        # Vector al lookahead point en coordenadas del vehículo
        dx = lookahead_point[0] - x
        dy = lookahead_point[1] - y
        
        # Ángulo al lookahead point (frame global)
        angle_to_point = math.atan2(dy, dx)
        
        # Error de heading (alpha)
        alpha = self.normalize_angle(angle_to_point - yaw)
        
        # Curvatura: κ = 2 * sin(α) / Ld
        if ld > 0:
            curvature = 2.0 * math.sin(alpha) / ld
        else:
            curvature = 0.0
        
        # Ángulo de steering: δ = atan(L * κ)
        steering = math.atan(self.L * curvature)
        
        # Limitar al ángulo máximo de steering
        steering = max(-self.max_steer, min(self.max_steer, steering))
        
        return steering, alpha
    
    def compute_velocity(self, steering, dist_to_goal):
        """
        Calcula la velocidad adaptativa basada en:
        1. Curvatura (menor velocidad en curvas cerradas)
        2. Distancia al goal (desacelerar al acercarse)
        """
        # Factor de curvatura: reducir velocidad en curvas
        steer_ratio = abs(steering) / self.max_steer
        curvature_factor = 1.0 - 0.5 * steer_ratio  # 50% reducción máxima
        
        # Factor de proximidad: desacelerar cerca del goal
        if dist_to_goal < 3.0:
            proximity_factor = max(0.5, dist_to_goal / 3.0)
        else:
            proximity_factor = 1.0
        
        # Velocidad final
        velocity = self.v_max * curvature_factor * proximity_factor
        velocity = max(self.v_min, min(self.v_max, velocity))
        
        return velocity
    
    @staticmethod
    def normalize_angle(angle):
        """Normaliza ángulo al rango [-π, π]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    # ================== CONTROL LOOP ==================
    def control_loop(self):
        """Loop principal de control."""
        metrics = Float32MultiArray()
        
        # Sin pose disponible
        if self.current_pose is None:
            metrics.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.pub_metrics.publish(metrics)
            return
        
        # Sin waypoints o inactivo
        if not self.waypoints or not self.active:
            metrics.data = [self.ld_min, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.pub_metrics.publish(metrics)
            self.pub_cmd.publish(Twist())
            return
        
        x, y, yaw = self.current_pose
        
        # Verificar si completamos todos los waypoints
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('🏁 ¡MISIÓN COMPLETADA!')
            self.active = False
            self.pub_cmd.publish(Twist())
            metrics.data = [self.ld_min, 0.0, float(len(self.waypoints)), 
                           float(len(self.waypoints)), 0.0, 0.0, 0.0]
            self.pub_metrics.publish(metrics)
            return
        
        # Distancia al waypoint actual
        target = self.waypoints[self.current_waypoint_idx]
        dist_to_goal = math.sqrt((target[0] - x)**2 + (target[1] - y)**2)
        
        # ¿Llegamos al waypoint?
        if dist_to_goal < self.goal_tol:
            self.get_logger().info(
                f'✅ Waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)} alcanzado'
            )
            self.current_waypoint_idx += 1
            return
        
        # ============ PURE PURSUIT ============
        # 1. Encontrar punto de lookahead
        lookahead_point, ld = self.find_lookahead_point(x, y)
        
        if lookahead_point is None:
            self.pub_cmd.publish(Twist())
            return
        
        # 2. Calcular ángulo de steering
        steering, alpha = self.compute_steering_angle(x, y, yaw, lookahead_point, ld)
        
        # 3. Calcular velocidad adaptativa
        velocity = self.compute_velocity(steering, dist_to_goal)
        
        # 4. Convertir a velocidad angular (modelo de bicicleta)
        # ω = v * tan(δ) / L
        angular_vel = velocity * math.tan(steering) / self.L
        
        # Limitar velocidad angular
        max_angular = 1.5  # rad/s
        angular_vel = max(-max_angular, min(max_angular, angular_vel))
        
        # ============ PUBLICAR COMANDO ============
        cmd = Twist()
        cmd.linear.x = velocity
        cmd.angular.z = angular_vel
        self.pub_cmd.publish(cmd)
        
        # ============ MÉTRICAS ============
        steering_deg = math.degrees(steering)
        
        # Dirección en texto
        if steering_deg > 2:
            direction = "← IZQ"
        elif steering_deg < -2:
            direction = "DER →"
        else:
            direction = "RECTO"
        
        self.get_logger().info(
            f'🚗 {direction} | δ={steering_deg:+5.1f}° | '
            f'v={velocity:.2f}m/s | ω={angular_vel:+.2f}rad/s | '
            f'WP={self.current_waypoint_idx+1}/{len(self.waypoints)} | '
            f'd={dist_to_goal:.1f}m | Ld={ld:.1f}m',
            throttle_duration_sec=0.3
        )
        
        # [lookahead, dist_wp, current_wp, total_wp, steering_deg, velocity, angular_vel]
        metrics.data = [
            ld,
            dist_to_goal,
            float(self.current_waypoint_idx + 1),
            float(len(self.waypoints)),
            steering_deg,
            velocity,
            angular_vel
        ]
        self.pub_metrics.publish(metrics)


def main():
    rclpy.init()
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
