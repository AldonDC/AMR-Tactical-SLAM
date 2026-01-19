#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
import numpy as np
import requests
import math
from io import BytesIO
from PIL import Image
import threading

class TacticalMissionControl(Node):
    """
    INTERFAZ SATELITAL TODO-EN-UNO
    - Mapa satelital ESRI
    - Trayectorias V1/V2 del SLAM
    - Click para agregar waypoints de navegación
    - Métricas del controlador en tiempo real
    """
    def __init__(self):
        super().__init__('tactical_mission_control')
        
        self.zoom = 18
        self.current_fix = None
        self.origin = None
        
        # Trayectorias SLAM
        self.trajectory_v1 = []  # Mapping (naranja)
        self.trajectory_v2 = []  # Localization (cyan)
        
        # Waypoints de navegación
        self.nav_waypoints = []
        
        # Estado del robot
        self.robot_pos = None
        
        # Métricas del controlador
        self.metrics = {
            'steering': 0.0,
            'angular_vel': 0.0,
            'linear_vel': 0.0,
            'lookahead': 0.0,
            'dist_wp': 0.0,
            'current_wp': 0,
            'total_wp': 0
        }
        
        # Historial para gráficas en tiempo real
        self.max_history = 200  # Últimos 200 puntos
        self.history_time = []
        self.history_steering = []
        self.history_angular = []
        self.history_dist_wp = []
        self.history_linear = []
        self.start_time = None
        self.mission_active = False
        
        self.data_lock = threading.Lock()

        
        # Suscripciones
        self.sub_fix = self.create_subscription(NavSatFix, '/rtk/fix', self.fix_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/rtk/odom_enu', self.odom_callback, 10)
        self.sub_sim_odom = self.create_subscription(Odometry, '/sim/odom', self.odom_callback, 10)  # Simulador
        self.sub_path_v1 = self.create_subscription(Path, '/slam/path_mapping', self.path_v1_callback, 10)
        self.sub_path_v2 = self.create_subscription(Path, '/slam/path_localization', self.path_v2_callback, 10)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.sub_metrics = self.create_subscription(Float32MultiArray, '/autopilot/metrics', self.metrics_callback, 10)

        
        # Publishers
        self.pub_nav_path = self.create_publisher(Path, '/navigation/path', 10)
        
        self.get_logger().info('🛰️ TACTICAL MISSION CONTROL Inicializado')
        
        # Configurar matplotlib
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(20, 11))
        self.fig.patch.set_facecolor('#1a1a1a')
        
        # Layout simple y limpio: 2 filas principales
        # Fila 1: Mapa (izquierda) + Panel control (derecha)
        # Fila 2: 4 gráficas
        gs = self.fig.add_gridspec(2, 2, 
            width_ratios=[1.5, 1], 
            height_ratios=[1.2, 1],
            wspace=0.08, hspace=0.12)
        
        # Subdividir lado derecho para panel + botones
        gs_right = gs[0, 1].subgridspec(3, 1, height_ratios=[4, 1, 1], hspace=0.1)
        
        # Subdividir fila inferior para 4 gráficas
        gs_graphs = gs[1, :].subgridspec(1, 4, wspace=0.15)
        
        # MAPA SATELITAL - Izquierda grande
        self.ax_map = self.fig.add_subplot(gs[0, 0])
        
        # PANEL DE CONTROL - Derecha arriba
        self.ax_metrics = self.fig.add_subplot(gs_right[0])
        
        # BOTONES
        from matplotlib.widgets import Button
        self.ax_btn_start = self.fig.add_subplot(gs_right[1])
        self.ax_btn_clear = self.fig.add_subplot(gs_right[2])
        
        # GRÁFICAS EN TIEMPO REAL - Abajo
        self.ax_steering = self.fig.add_subplot(gs_graphs[0])
        self.ax_angular = self.fig.add_subplot(gs_graphs[1])
        self.ax_distance = self.fig.add_subplot(gs_graphs[2])
        self.ax_velocity = self.fig.add_subplot(gs_graphs[3])
        
        # Configurar botones
        self.btn_start = Button(self.ax_btn_start, 'START MISSION', color='#1a6b1a', hovercolor='#2a9b2a')
        self.btn_start.label.set_fontsize(14)
        self.btn_start.label.set_fontweight('bold')
        self.btn_start.on_clicked(self.start_mission)
        
        self.btn_clear = Button(self.ax_btn_clear, 'STOP / CLEAR', color='#6b1a1a', hovercolor='#9b2a2a')
        self.btn_clear.label.set_fontsize(14)
        self.btn_clear.label.set_fontweight('bold')
        self.btn_clear.on_clicked(self.clear_and_stop)
        
        self.fig.canvas.manager.set_window_title('TACTICAL MISSION CONTROL')
        
        # ELEMENTOS DEL MAPA
        self.zoom = 19
        self.map_img = self.ax_map.imshow(np.zeros((256, 256, 3), dtype=np.uint8))
        self.traj_v1_line, = self.ax_map.plot([], [], '-', color='#ff8c00', linewidth=2.5, alpha=0.7, label='V1: Mapping', zorder=4)
        self.traj_v2_line, = self.ax_map.plot([], [], '-', color='#00ddff', linewidth=2.5, alpha=0.9, label='V2: RTK Ref', zorder=5)
        self.nav_waypoints_line, = self.ax_map.plot([], [], 'o', color='#00ff00', markersize=12, markeredgecolor='white', markeredgewidth=2, label='Waypoints', zorder=6)
        self.robot_marker, = self.ax_map.plot([], [], '^', color='#ff2222', markersize=14, markeredgecolor='white', markeredgewidth=2, label='Robot', zorder=7)
        
        self.ax_map.set_xlabel('Longitude', fontsize=10, color='#999999')
        self.ax_map.set_ylabel('Latitude', fontsize=10, color='#999999')
        self.ax_map.set_title('SATELLITE MAP  •  Click to add waypoints', fontsize=12, color='white', weight='bold', pad=8)
        self.ax_map.legend(loc='upper left', fontsize=8, facecolor='#222222', edgecolor='#444444')
        self.ax_map.tick_params(colors='#666666', labelsize=8)
        
        # PANEL DE MÉTRICAS
        self.ax_metrics.set_facecolor('#0d0d0d')
        self.ax_metrics.axis('off')
        self.metrics_text = self.ax_metrics.text(0.05, 0.95, '', fontsize=10, color='#00ff88', family='monospace', verticalalignment='top', linespacing=1.5)
        
        # Configurar gráficas en tiempo real
        self.setup_realtime_graphs()
        
        self.map_center_tile = None
        
        # Click event
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        # Animación
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=False)
        
        plt.subplots_adjust(left=0.04, right=0.97, top=0.95, bottom=0.06)


    
    def setup_realtime_graphs(self):
        """Configura las gráficas en tiempo real con estilo profesional."""
        
        # Estilo común para gráficas con fondo blanco
        for ax in [self.ax_steering, self.ax_angular, self.ax_distance, self.ax_velocity]:
            ax.set_facecolor('white')
            ax.spines['top'].set_color('#cccccc')
            ax.spines['right'].set_color('#cccccc')
            ax.spines['bottom'].set_color('#666666')
            ax.spines['left'].set_color('#666666')
        
        # Gráfica de Steering
        self.ax_steering.set_title('Steering Angle (°)', fontsize=11, color='#333333', weight='bold', pad=8)
        self.ax_steering.set_ylabel('Degrees', fontsize=9, color='#555555')
        self.ax_steering.set_xlabel('Time (s)', fontsize=9, color='#555555')
        self.ax_steering.axhline(y=0, color='#aaaaaa', linestyle='-', linewidth=0.8)
        self.ax_steering.set_ylim(-45, 45)
        self.ax_steering.grid(True, alpha=0.4, color='#dddddd')
        self.ax_steering.tick_params(colors='#444444', labelsize=8)
        self.line_steering, = self.ax_steering.plot([], [], '-', color='#e6a800', linewidth=2.5)
        
        # Gráfica de Angular Velocity
        self.ax_angular.set_title('Angular Velocity (rad/s)', fontsize=11, color='#333333', weight='bold', pad=8)
        self.ax_angular.set_ylabel('rad/s', fontsize=9, color='#555555')
        self.ax_angular.set_xlabel('Time (s)', fontsize=9, color='#555555')
        self.ax_angular.axhline(y=0, color='#aaaaaa', linestyle='-', linewidth=0.8)
        self.ax_angular.set_ylim(-1.5, 1.5)
        self.ax_angular.grid(True, alpha=0.4, color='#dddddd')
        self.ax_angular.tick_params(colors='#444444', labelsize=8)
        self.line_angular, = self.ax_angular.plot([], [], '-', color='#e65c00', linewidth=2.5)
        
        # Gráfica de Distance to WP
        self.ax_distance.set_title('Distance to Waypoint (m)', fontsize=11, color='#333333', weight='bold', pad=8)
        self.ax_distance.set_ylabel('Meters', fontsize=9, color='#555555')
        self.ax_distance.set_xlabel('Time (s)', fontsize=9, color='#555555')
        self.ax_distance.set_ylim(0, 50)
        self.ax_distance.grid(True, alpha=0.4, color='#dddddd')
        self.ax_distance.tick_params(colors='#444444', labelsize=8)
        self.line_distance, = self.ax_distance.plot([], [], '-', color='#00994d', linewidth=2.5)
        
        # Gráfica de Linear Velocity
        self.ax_velocity.set_title('Linear Velocity (m/s)', fontsize=11, color='#333333', weight='bold', pad=8)
        self.ax_velocity.set_ylabel('m/s', fontsize=9, color='#555555')
        self.ax_velocity.set_xlabel('Time (s)', fontsize=9, color='#555555')
        self.ax_velocity.set_ylim(0, 2)
        self.ax_velocity.grid(True, alpha=0.4, color='#dddddd')
        self.ax_velocity.tick_params(colors='#444444', labelsize=8)
        self.line_velocity, = self.ax_velocity.plot([], [], '-', color='#0080cc', linewidth=2.5)



    
    def fix_callback(self, msg: NavSatFix):
        with self.data_lock:
            self.current_fix = msg
            if self.origin is None:
                self.origin = {'lat': msg.latitude, 'lon': msg.longitude}
    
    def odom_callback(self, msg: Odometry):
        with self.data_lock:
            x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
            if self.origin:
                lat, lon = self.enu_to_lla(x, y, 0)
                self.robot_pos = {'lat': lat, 'lon': lon}
    
    def path_v1_callback(self, msg: Path):
        with self.data_lock:
            self.trajectory_v1 = []
            for pose in msg.poses:
                if self.origin:
                    lat, lon = self.enu_to_lla(pose.pose.position.x, pose.pose.position.y, 0)
                    self.trajectory_v1.append({'lat': lat, 'lon': lon})
    
    def path_v2_callback(self, msg: Path):
        with self.data_lock:
            self.trajectory_v2 = []
            for pose in msg.poses:
                if self.origin:
                    lat, lon = self.enu_to_lla(pose.pose.position.x, pose.pose.position.y, 0)
                    self.trajectory_v2.append({'lat': lat, 'lon': lon})
    
    def cmd_callback(self, msg: Twist):
        with self.data_lock:
            self.metrics['linear_vel'] = msg.linear.x
            self.metrics['angular_vel'] = msg.angular.z
    
    def metrics_callback(self, msg: Float32MultiArray):
        with self.data_lock:
            if len(msg.data) >= 5:
                self.metrics['lookahead'] = msg.data[0]
                self.metrics['dist_wp'] = msg.data[1]
                self.metrics['current_wp'] = int(msg.data[2])
                self.metrics['total_wp'] = int(msg.data[3])
                self.metrics['steering'] = msg.data[4]
    
    def on_click(self, event):
        """Maneja clics en el mapa para agregar waypoints."""
        if event.inaxes != self.ax_map or not self.origin:
            return
        
        lon, lat = event.xdata, event.ydata
        
        with self.data_lock:
            # Convertir GPS a ENU
            x, y = self.lla_to_enu(lat, lon, 0)
            self.nav_waypoints.append({'lat': lat, 'lon': lon, 'x': x, 'y': y})
            self.get_logger().info(f'📍 Waypoint {len(self.nav_waypoints)} agregado (Presiona START para confirmar)')
    
    def start_mission(self, event):
        """Confirma y publica la misión."""
        with self.data_lock:
            if not self.nav_waypoints:
                self.get_logger().warn('⚠️ No hay waypoints para ejecutar')
                return
            self.publish_nav_path()
    
    def clear_waypoints(self, event):
        """Limpia todos los waypoints."""
        with self.data_lock:
            self.nav_waypoints.clear()
            self.get_logger().info('🗑️ Waypoints eliminados')
    
    def stop_mission(self, event):
        """Detiene la misión actual."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        self.pub_nav_path.publish(path_msg)
        self.get_logger().info('⏹️ MISIÓN DETENIDA')
    
    def clear_and_stop(self, event):
        """Limpia waypoints y detiene el robot."""
        self.clear_waypoints(event)
        self.stop_mission(event)

        self.get_logger().info('⏹️ MISIÓN DETENIDA')
    
    def publish_nav_path(self):
        """Publica la ruta de navegación."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for wp in self.nav_waypoints:
            pose = PoseStamped()
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            path_msg.poses.append(pose)
        
        self.pub_nav_path.publish(path_msg)
        self.get_logger().info(f'🚀 Ruta publicada: {len(self.nav_waypoints)} waypoints')
    
    def enu_to_lla(self, e, n, u):
        if not self.origin: return 0, 0
        lat0, lon0 = self.origin['lat'], self.origin['lon']
        R = 6378137.0
        dlat = n / R
        dlon = e / (R * math.cos(math.radians(lat0)))
        return lat0 + math.degrees(dlat), lon0 + math.degrees(dlon)
    
    def lla_to_enu(self, lat, lon, alt):
        if not self.origin: return 0, 0
        lat0, lon0 = self.origin['lat'], self.origin['lon']
        R = 6378137.0
        dlat = math.radians(lat - lat0)
        dlon = math.radians(lon - lon0)
        n = dlat * R
        e = dlon * R * math.cos(math.radians(lat0))
        return e, n
    
    def deg2num(self, lat, lon, zoom):
        n = 2 ** zoom
        xtile = int((lon + 180) / 360 * n)
        ytile = int((1 - math.log(math.tan(math.radians(lat)) + 1/math.cos(math.radians(lat))) / math.pi) / 2 * n)
        return xtile, ytile
    
    def num2deg(self, xtile, ytile, zoom):
        n = 2 ** zoom
        lon = xtile / n * 360 - 180
        lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
        lat = math.degrees(lat_rad)
        return lat, lon
    
    def fetch_tile(self, x, y, z):
        url = f"https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
        try:
            r = requests.get(url, timeout=2)
            if r.status_code == 200:
                return np.array(Image.open(BytesIO(r.content)))
        except:
            pass
        return np.zeros((256, 256, 3), dtype=np.uint8)
    
    def update_plot(self, frame):
        with self.data_lock:
            if self.current_fix is None:
                return self.map_img,
            
            lat, lon = self.current_fix.latitude, self.current_fix.longitude
            if math.isnan(lat) or math.isnan(lon):
                return self.map_img,
            
            xtile, ytile = self.deg2num(lat, lon, self.zoom)
            
            # Actualizar mapa si es necesario
            if self.map_center_tile != (xtile, ytile):
                self.map_center_tile = (xtile, ytile)
                full_map = np.zeros((256*3, 256*3, 3), dtype=np.uint8)
                for i, dx in enumerate([-1, 0, 1]):
                    for j, dy in enumerate([-1, 0, 1]):
                        tile = self.fetch_tile(xtile + dx, ytile + dy, self.zoom)
                        full_map[j*256:(j+1)*256, i*256:(i+1)*256] = tile
                
                lat_max, lon_min = self.num2deg(xtile - 1, ytile - 1, self.zoom)
                lat_min, lon_max = self.num2deg(xtile + 2, ytile + 2, self.zoom)
                
                self.map_img.set_data(full_map)
                self.map_img.set_extent([lon_min, lon_max, lat_min, lat_max])
                self.ax_map.set_xlim(lon_min, lon_max)
                self.ax_map.set_ylim(lat_min, lat_max)
            
            # Actualizar trayectorias SLAM
            if self.trajectory_v1:
                lats = [p['lat'] for p in self.trajectory_v1]
                lons = [p['lon'] for p in self.trajectory_v1]
                self.traj_v1_line.set_data(lons, lats)
            
            if self.trajectory_v2:
                lats = [p['lat'] for p in self.trajectory_v2]
                lons = [p['lon'] for p in self.trajectory_v2]
                self.traj_v2_line.set_data(lons, lats)
            
            # Actualizar waypoints de navegación
            if self.nav_waypoints:
                lats = [wp['lat'] for wp in self.nav_waypoints]
                lons = [wp['lon'] for wp in self.nav_waypoints]
                self.nav_waypoints_line.set_data(lons, lats)
            
            # Actualizar robot
            if self.robot_pos:
                self.robot_marker.set_data([self.robot_pos['lon']], [self.robot_pos['lat']])
            
            # Actualizar métricas con indicaciones de giro
            steering = self.metrics['steering']
            angular = self.metrics['angular_vel']
            linear = self.metrics['linear_vel']
            dist_wp = self.metrics['dist_wp']
            current_wp = self.metrics['current_wp']
            total_wp = self.metrics['total_wp']
            lookahead = self.metrics['lookahead']
            
            # Determinar dirección de giro
            if steering > 10:
                direction = "LEFT"
                arrow = "◄◄◄"
            elif steering < -10:
                direction = "RIGHT"
                arrow = "►►►"
            else:
                direction = "STRAIGHT"
                arrow = "▲▲▲"
            
            # Estado de misión
            if total_wp == 0:
                status = "STANDBY"
                status_icon = "○"
            elif linear > 0.01:
                status = "NAVIGATING"
                status_icon = "●"
            else:
                status = "STOPPED"
                status_icon = "■"
            
            # Barra de volante visual
            steer_norm = max(-45, min(45, steering))
            bar_total = 25
            center_pos = bar_total // 2
            steer_pos = int((steer_norm + 45) / 90 * bar_total)
            bar_chars = list("─" * bar_total)
            bar_chars[center_pos] = "┼"
            if 0 <= steer_pos < bar_total:
                bar_chars[steer_pos] = "●"
            steering_bar = "".join(bar_chars)
            
            # Dashboard mejorado y limpio
            metrics_str = f"""
┌─────────────────────────────────┐
│     PURE PURSUIT CONTROLLER     │
├─────────────────────────────────┤
│                                 │
│  {status_icon} STATUS:  {status:<12}      │
│                                 │
├─────────────────────────────────┤
│                                 │
│  TURN:  {direction:^8}   {arrow}          │
│                                 │
├─────────────────────────────────┤
│  STEERING ANGLE                 │
│                                 │
│    {steering:+6.1f}°                      │
│  [{steering_bar}]│
│                                 │
├─────────────────────────────────┤
│  VELOCITY                       │
│    Linear:   {linear:5.2f} m/s          │
│    Angular:  {angular:+5.2f} rad/s        │
│                                 │
├─────────────────────────────────┤
│  WAYPOINT NAVIGATION            │
│    Lookahead:  {lookahead:4.1f} m           │
│    Distance:   {dist_wp:4.1f} m           │
│    Progress:   {int(current_wp)}/{int(total_wp)}              │
│                                 │
└─────────────────────────────────┘
"""
            self.metrics_text.set_text(metrics_str)

            
            # Actualizar historial para gráficas
            if total_wp > 0:  # Solo cuando hay misión activa
                if self.start_time is None:
                    self.start_time = self.get_clock().now()
                    self.mission_active = True
                
                current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                
                self.history_time.append(current_time)
                self.history_steering.append(steering)
                self.history_angular.append(angular)
                self.history_dist_wp.append(dist_wp)
                self.history_linear.append(linear)
                
                # Limitar tamaño del historial
                if len(self.history_time) > self.max_history:
                    self.history_time = self.history_time[-self.max_history:]
                    self.history_steering = self.history_steering[-self.max_history:]
                    self.history_angular = self.history_angular[-self.max_history:]
                    self.history_dist_wp = self.history_dist_wp[-self.max_history:]
                    self.history_linear = self.history_linear[-self.max_history:]
                
                # Actualizar gráficas
                if len(self.history_time) > 1:
                    self.line_steering.set_data(self.history_time, self.history_steering)
                    self.line_angular.set_data(self.history_time, self.history_angular)
                    self.line_distance.set_data(self.history_time, self.history_dist_wp)
                    self.line_velocity.set_data(self.history_time, self.history_linear)
                    
                    # Ajustar límites X
                    x_min = max(0, current_time - 30)  # Últimos 30 segundos
                    x_max = current_time + 1
                    self.ax_steering.set_xlim(x_min, x_max)
                    self.ax_angular.set_xlim(x_min, x_max)
                    self.ax_distance.set_xlim(x_min, x_max)
                    self.ax_velocity.set_xlim(x_min, x_max)
            else:
                # Resetear cuando no hay misión
                if self.mission_active:
                    self.history_time.clear()
                    self.history_steering.clear()
                    self.history_angular.clear()
                    self.history_dist_wp.clear()
                    self.history_linear.clear()
                    self.start_time = None
                    self.mission_active = False


        
        return self.map_img, self.traj_v1_line, self.traj_v2_line, self.nav_waypoints_line, self.robot_marker


def main():
    rclpy.init()
    node = TacticalMissionControl()
    
    # ROS en thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    plt.show()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
