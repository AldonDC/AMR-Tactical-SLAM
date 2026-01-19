#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import tkinter as tk
from tkinter import ttk
import threading

class ControllerDashboard(Node):
    """
    Dashboard que muestra métricas del controlador Pure Pursuit en tiempo real.
    """
    def __init__(self):
        super().__init__('controller_dashboard')
        
        # Variables de métricas
        self.steering_angle = 0.0
        self.angular_vel = 0.0
        self.linear_vel = 0.0
        self.lookahead_dist = 0.0
        self.dist_to_waypoint = 0.0
        self.current_waypoint = 0
        self.total_waypoints = 0
        
        # Suscriptores
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.sub_metrics = self.create_subscription(
            Float32MultiArray, '/autopilot/metrics', self.metrics_callback, 10)
        
        self.get_logger().info('📊 Controller Dashboard Inicializado')
        
        # Crear GUI
        self.create_dashboard()
    
    def cmd_callback(self, msg: Twist):
        """Actualiza velocidades del comando."""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        self.update_display()
    
    def metrics_callback(self, msg: Float32MultiArray):
        """Recibe métricas del autopiloto."""
        # Formato: [lookahead, dist_to_wp, current_wp, total_wp, steering_angle]
        if len(msg.data) >= 5:
            self.lookahead_dist = msg.data[0]
            self.dist_to_waypoint = msg.data[1]
            self.current_waypoint = int(msg.data[2])
            self.total_waypoints = int(msg.data[3])
            self.steering_angle = msg.data[4]
            self.update_display()
    
    def create_dashboard(self):
        """Crea el dashboard compacto."""
        self.root = tk.Tk()
        self.root.title('🎮 AUTOPILOT DASHBOARD')
        self.root.geometry('400x400')
        self.root.configure(bg='#0a0a0a')
        self.root.attributes('-topmost', True)  # Siempre al frente
        
        # Título
        title = tk.Label(self.root, text='🎮 PURE PURSUIT METRICS', 
                        bg='#0a0a0a', fg='#00ff00', font=('Courier', 14, 'bold'))
        title.pack(pady=10)
        
        # Frame de métricas
        metrics_frame = tk.Frame(self.root, bg='#1a1a1a', relief=tk.RIDGE, bd=3)
        metrics_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Etiquetas de métricas
        self.labels = {}
        metrics = [
            ('📐 Steering Angle', 'steering', '°'),
            ('🔄 Angular Velocity', 'angular', 'rad/s'),
            ('⚡ Linear Velocity', 'linear', 'm/s'),
            ('📊 Lookahead Distance', 'lookahead', 'm'),
            ('🎯 Distance to Waypoint', 'dist_wp', 'm'),
            ('📍 Current Waypoint', 'waypoint', '')
        ]
        
        for i, (name, key, unit) in enumerate(metrics):
            # Nombre
            tk.Label(metrics_frame, text=name, bg='#1a1a1a', fg='cyan',
                    font=('Courier', 10), anchor='w').grid(row=i, column=0, sticky='w', padx=10, pady=5)
            
            # Valor
            value_label = tk.Label(metrics_frame, text='0.00', bg='#1a1a1a', fg='lime',
                                  font=('Courier', 12, 'bold'), anchor='e')
            value_label.grid(row=i, column=1, sticky='e', padx=5, pady=5)
            self.labels[key] = (value_label, unit)
        
        # Estado
        self.status_label = tk.Label(self.root, text='⏸️ STANDBY', 
                                     bg='#0a0a0a', fg='yellow', font=('Courier', 11, 'bold'))
        self.status_label.pack(pady=10)
    
    def update_display(self):
        """Actualiza los valores en la GUI."""
        if not hasattr(self, 'labels'):
            return
        
        try:
            # Actualizar valores
            self.labels['steering'][0].config(text=f'{self.steering_angle:.2f}{self.labels["steering"][1]}')
            self.labels['angular'][0].config(text=f'{self.angular_vel:.3f}{self.labels["angular"][1]}')
            self.labels['linear'][0].config(text=f'{self.linear_vel:.2f}{self.labels["linear"][1]}')
            self.labels['lookahead'][0].config(text=f'{self.lookahead_dist:.2f}{self.labels["lookahead"][1]}')
            self.labels['dist_wp'][0].config(text=f'{self.dist_to_waypoint:.2f}{self.labels["dist_wp"][1]}')
            self.labels['waypoint'][0].config(text=f'{self.current_waypoint}/{self.total_waypoints}')
            
            # Actualizar estado
            if self.linear_vel > 0.01:
                self.status_label.config(text='🚀 ACTIVE', fg='lime')
            else:
                self.status_label.config(text='⏸️ STANDBY', fg='yellow')
        except:
            pass
    
    def run(self):
        """Ejecuta el dashboard."""
        self.root.mainloop()

def main():
    rclpy.init()
    node = ControllerDashboard()
    
    # ROS en thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    # GUI en thread principal
    node.run()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
