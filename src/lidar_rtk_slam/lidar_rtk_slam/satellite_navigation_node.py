#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path, Odometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon, Circle
import numpy as np
import requests
from io import BytesIO
from PIL import Image
import math
import threading
import time

class SatelliteMissionControl(Node):
    def __init__(self):
        super().__init__('satellite_mission_control')
        
        # Parámetros
        self.declare_parameter('zoom', 18)
        self.zoom = self.get_parameter('zoom').value
        
        # Estado
        self.current_fix = None
        self.origin = None # (lat, lon, alt)
        self.path_mapping_raw = [] 
        self.path_loc_raw = []     
        self.tiles_cache = {}
        self.map_center_tile = None
        self.current_heading = 0.0
        self.current_speed = 0.0
        self.phase = "INITIALIZING"
        self.total_dist = 0.0
        
        # WGS84 constants
        self.WGS84_A = 6378137.0
        self.WGS84_E2 = 0.00669437999014
        
        # Locks
        self.data_lock = threading.Lock()
        
        # Suscriptores
        self.create_subscription(NavSatFix, '/rtk/fix', self.fix_callback, 10)
        self.create_subscription(Path, '/slam/path_mapping', self.path_mapping_callback, 10)
        self.create_subscription(Path, '/slam/path_localization', self.path_loc_callback, 10)
        self.create_subscription(Odometry, '/rtk/odom_enu', self.odom_callback, 10)
        
        # UI Setup (Estilo Cyberpunk/Tactical)
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(12, 9), facecolor='#050505')
        self.gs = self.fig.add_gridspec(1, 1)
        self.ax = self.fig.add_subplot(self.gs[0, 0])
        
        self.fig.canvas.manager.set_window_title('🛰️ MISSION CONTROL - AMR 2026')
        
        self.ax.set_aspect('equal')
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        for spine in self.ax.spines.values():
            spine.set_edgecolor('#00FFFF')
            spine.set_linewidth(1.5)
            spine.set_alpha(0.3)
        
        # Elementos del Mapa
        self.map_img = self.ax.imshow(np.zeros((256, 256, 3)), extent=[0, 1, 0, 1], interpolation='bilinear')
        self.line_mapping, = self.ax.plot([], [], color='#FF8C00', linewidth=2, label='V1: MAPPING', alpha=0.8, zorder=5)
        self.line_loc, = self.ax.plot([], [], color='#00FFFF', linewidth=2.5, label='V2: LOCALIZATION', alpha=0.9, zorder=6)
        
        # Icono del Vehículo (Triángulo táctico)
        self.vehicle_poly = Polygon([[0,0], [0,0], [0,0]], facecolor='#FF0000', edgecolor='white', linewidth=1, zorder=15)
        self.ax.add_patch(self.vehicle_poly)
        
        # Overlays de Texto (HUD)
        self.txt_title = self.fig.text(0.5, 0.96, "AMR 2026 TACTICAL MISSION CONTROL", 
                                      color='#00FFFF', fontsize=14, fontweight='bold', ha='center', alpha=0.8)
        
        # Panel Lateral (Información en tiempo real)
        self.info_box = self.fig.text(0.02, 0.93, "", color='white', family='monospace', fontsize=10, 
                                     bbox=dict(facecolor='black', alpha=0.6, edgecolor='#00FFFF', boxstyle='round,pad=1'))
        
        # Badge de fase
        self.phase_badge = self.fig.text(0.85, 0.93, " PHASE: N/A ", color='black', fontweight='bold',
                                        bbox=dict(facecolor='#888888', alpha=0.9, edgecolor='none', boxstyle='round,pad=0.3'))

        # Timer para animación
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, cache_frame_data=False)
        
        self.get_logger().info("🚀 Tactical Satellite Node Started!")
        
    def deg2num(self, lat_deg, lon_deg, zoom):
        if math.isnan(lat_deg) or math.isnan(lon_deg): return 0, 0
        lat_rad = math.radians(lat_deg)
        n = 2.0 ** zoom
        xtile = int((lon_deg + 180.0) / 360.0 * n)
        ytile = int((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
        return xtile, ytile

    def num2deg(self, xtile, ytile, zoom):
        n = 2.0 ** zoom
        lon_deg = xtile / n * 360.0 - 180.0
        lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
        lat_deg = math.degrees(lat_rad)
        return lat_deg, lon_deg

    def enu_to_lla(self, e, n, u):
        if self.origin is None: return None
        lat0, lon0, alt0 = self.origin
        lat0_rad = math.radians(lat0)
        sin_lat0 = math.sin(lat0_rad)
        cos_lat0 = math.cos(lat0_rad)
        N0 = self.WGS84_A / math.sqrt(1.0 - self.WGS84_E2 * sin_lat0 * sin_lat0)
        dlat_rad = n / (N0 * (1.0 - self.WGS84_E2) / (1.0 - self.WGS84_E2 * sin_lat0 * sin_lat0) + alt0)
        dlon_rad = e / ((N0 + alt0) * cos_lat0)
        lat = lat0 + math.degrees(dlat_rad)
        lon = lon0 + math.degrees(dlon_rad)
        return lat, lon

    def fetch_tile(self, x, y, z):
        tile_id = f"{z}/{x}/{y}"
        if tile_id in self.tiles_cache: return self.tiles_cache[tile_id]
        url = f"https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
        try:
            response = requests.get(url, timeout=1.0)
            if response.status_code == 200:
                img = Image.open(BytesIO(response.content))
                self.tiles_cache[tile_id] = np.array(img)
                return self.tiles_cache[tile_id]
        except: pass
        return np.zeros((256, 256, 3), dtype=np.uint8)

    def fix_callback(self, msg):
        with self.data_lock:
            if math.isnan(msg.latitude): return
            self.current_fix = msg
            if self.origin is None:
                self.origin = (msg.latitude, msg.longitude, msg.altitude)

    def path_mapping_callback(self, msg):
        with self.data_lock:
            self.path_mapping_raw = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self.phase = "MAPPING"

    def path_loc_callback(self, msg):
        with self.data_lock:
            self.path_loc_raw = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            if len(self.path_loc_raw) > 0: self.phase = "LOCALIZATION"

    def odom_callback(self, msg):
        with self.data_lock:
            q = msg.pose.pose.orientation
            # Simple yaw from quaternion
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.current_heading = math.atan2(siny_cosp, cosy_cosp)
            
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            self.current_speed = math.sqrt(vx*vx + vy*vy) * 3.6 # km/h

    def update_vehicle_icon(self, lat, lon, heading):
        # Escala aproximada del triángulo en grados (~5 metros)
        size = 0.00004 
        # Triángulo apuntando al heading (0 = Este en math, pero heading 0 es Norte en SLAM a veces)
        # En el mapa: lat es Y, lon es X.
        # Heading 0 en mi SLAM suele ser Este o Norte dependiendo de la calibración.
        # Ajustamos el ángulo:
        angle = heading 
        
        p1 = [lon + size * math.cos(angle), lat + size * math.sin(angle)]
        p2 = [lon + size/2 * math.cos(angle + 2.5), lat + size/2 * math.sin(angle + 2.5)]
        p3 = [lon + size/2 * math.cos(angle - 2.5), lat + size/2 * math.sin(angle - 2.5)]
        
        self.vehicle_poly.set_xy([p1, p2, p3])

    def update_plot(self, frame):
        with self.data_lock:
            if self.current_fix is None: return
            
            lat, lon = self.current_fix.latitude, self.current_fix.longitude
            if math.isnan(lat) or math.isnan(lon): return
            
            xtile, ytile = self.deg2num(lat, lon, self.zoom)
            
            # Update Map Mosaic (1x1 for faster performance or 3x3 for context)
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

            # Trajectories
            if self.origin:
                if self.path_mapping_raw:
                    pts = [self.enu_to_lla(e, n, 0) for e, n in self.path_mapping_raw]
                    lats, lons = zip(*pts)
                    self.line_mapping.set_data(lons, lats)
                if self.path_loc_raw:
                    pts = [self.enu_to_lla(e, n, 0) for e, n in self.path_loc_raw]
                    lats, lons = zip(*pts)
                    self.line_loc.set_data(lons, lats)
            
            # Vehicle Icon
            self.update_vehicle_icon(lat, lon, self.current_heading)
            
            # HUD Update
            info_text = (
                f"LAT: {lat:11.7f}\n"
                f"LON: {lon:11.7f}\n"
                f"SPD: {self.current_speed:5.1f} km/h\n"
                f"HDG: {math.degrees(self.current_heading):5.1f}°\n"
                f"PTS: {len(self.path_mapping_raw) + len(self.path_loc_raw):5d}"
            )
            self.info_box.set_text(info_text)
            
            # Phase Badge Update
            self.phase_badge.set_text(f" {self.phase} ")
            if self.phase == "MAPPING": self.phase_badge.set_bbox(dict(facecolor='#FF8C00', alpha=0.9, edgecolor='none', boxstyle='round'))
            elif self.phase == "LOCALIZATION": self.phase_badge.set_bbox(dict(facecolor='#00FFFF', alpha=0.9, edgecolor='none', boxstyle='round'))
            
        return self.map_img, self.line_mapping, self.line_loc, self.vehicle_poly

def main():
    rclpy.init()
    node = SatelliteMissionControl()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    plt.show()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
