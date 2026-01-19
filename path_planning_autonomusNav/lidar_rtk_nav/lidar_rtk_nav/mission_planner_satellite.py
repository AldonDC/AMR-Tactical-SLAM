#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
import numpy as np
import requests
import math
from io import BytesIO
from PIL import Image
import threading

class MissionPlannerSatelliteView(Node):
    """
    Vista Satelital para Planificación de Misiones.
    Muestra waypoints, ruta planeada y posición del robot en tiempo real.
    """
    def __init__(self):
        super().__init__('mission_planner_satellite')
        
        self.zoom = 18
        self.current_fix = None
        self.origin = None
        self.waypoints = []  # Lista de waypoints planeados
        self.current_path = []  # Ruta activa
        self.current_pose = None
        
        self.data_lock = threading.Lock()
        
        # Suscripciones
        self.sub_odom = self.create_subscription(Odometry, '/rtk/odom_enu', self.odom_callback, 10)
        self.sub_path = self.create_subscription(Path, '/navigation/path', self.path_callback, 10)
        
        # Configurar matplotlib
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.fig.canvas.manager.set_window_title('🛰️ MISSION PLANNER - Satellite View')
        
        # Elementos visuales
        self.map_img = self.ax.imshow(np.zeros((256, 256, 3), dtype=np.uint8))
        self.waypoint_markers, = self.ax.plot([], [], 'go', markersize=12, label='Waypoints', zorder=5)
        self.path_line, = self.ax.plot([], [], 'c-', linewidth=3, label='Ruta Planeada', zorder=4)
        self.robot_marker, = self.ax.plot([], [], 'ro', markersize=15, label='Robot', zorder=6)
        
        self.ax.set_xlabel('Longitud', fontsize=12, color='cyan')
        self.ax.set_ylabel('Latitud', fontsize=12, color='cyan')
        self.ax.set_title('🗺️ MISSION PLANNER | Vista Satelital', fontsize=16, color='white', weight='bold')
        self.ax.legend(loc='upper right')
        self.ax.grid(True, alpha=0.3)
        
        self.map_center_tile = None
        
        self.get_logger().info('🛰️ Mission Planner Satellite View Inicializado')
        
        # Animación
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=200, blit=False)
        
    def odom_callback(self, msg: Odometry):
        with self.data_lock:
            x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
            
            if self.origin is None:
                # Establecer origen aproximado (necesitamos suscripción a /rtk/fix)
                self.origin = {'E': 0, 'N': 0, 'lat': 19.0165015, 'lon': -98.2419030}
            
            # Convertir ENU a GPS
            lat, lon = self.enu_to_lla(x, y, 0)
            self.current_pose = {'lat': lat, 'lon': lon, 'x': x, 'y': y}
            
    def path_callback(self, msg: Path):
        with self.data_lock:
            self.current_path = []
            for pose in msg.poses:
                x, y = pose.pose.position.x, pose.pose.position.y
                if self.origin:
                    lat, lon = self.enu_to_lla(x, y, 0)
                    self.current_path.append({'lat': lat, 'lon': lon})
    
    def enu_to_lla(self, e, n, u):
        if not self.origin: return 0, 0
        lat0, lon0 = self.origin['lat'], self.origin['lon']
        R = 6378137.0
        dlat = n / R
        dlon = e / (R * math.cos(math.radians(lat0)))
        return lat0 + math.degrees(dlat), lon0 + math.degrees(dlon)
    
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
            if self.current_pose is None:
                return self.map_img,
            
            lat, lon = self.current_pose['lat'], self.current_pose['lon']
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
                self.ax.set_xlim(lon_min, lon_max)
                self.ax.set_ylim(lat_min, lat_max)
            
            # Actualizar ruta planeada
            if self.current_path:
                lats = [p['lat'] for p in self.current_path]
                lons = [p['lon'] for p in self.current_path]
                self.path_line.set_data(lons, lats)
            
            # Actualizar robot
            self.robot_marker.set_data([lon], [lat])
            
        return self.map_img, self.path_line, self.robot_marker

def main():
    rclpy.init()
    node = MissionPlannerSatelliteView()
    
    # Thread para ROS
    import threading
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    plt.show()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
