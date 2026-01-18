#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import threading
import time

class LioSamMasterViz(Node):
    def __init__(self):
        super().__init__('viz_pro_node')
        
        # Suscripciones con nombres de tópicos reales
        self.create_subscription(PointCloud2, '/lidar/filtered', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, '/slam/path', self.path_callback, 10)
        
        self.lock = threading.Lock()
        self.active_scan = None
        self.vehicle_pose = np.eye(4)
        self.path_points = None
        
        self.get_logger().info("Visualizador Pro-Style LIO-SAM Cargado")

    def lidar_callback(self, msg):
        try:
            gen = pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True)
            pts_struct = np.fromiter(gen, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
            if len(pts_struct) == 0: return
            
            pts = np.stack([pts_struct['x'], pts_struct['y'], pts_struct['z']], axis=1)
            with self.lock:
                pts_h = np.hstack([pts, np.ones((pts.shape[0], 1))])
                # Transformar puntos a coordenadas del mundo para mapeo visual
                self.active_scan = (self.vehicle_pose @ pts_h.T).T[:, :3]
        except Exception as e:
            pass

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self.lock:
            self.vehicle_pose = np.eye(4)
            self.vehicle_pose[0:3, 3] = [pos.x, pos.y, pos.z]
            # Convertir Yaw
            yaw = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
            c, s = np.cos(yaw), np.sin(yaw)
            self.vehicle_pose[0:0+2, 0:0+2] = [[c, -s], [s, c]]

    def path_callback(self, msg):
        with self.lock:
            if len(msg.poses) > 0:
                self.path_points = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses])

def build_pro_vehicle():
    """Crea un coche detallado para SLAM"""
    # Cuerpo (Box + Color Gris Carbón)
    car = o3d.geometry.TriangleMesh.create_box(width=2.5, height=1.4, depth=0.6)
    car.translate([-1.25, -0.7, 0.2])
    car.paint_uniform_color([0.1, 0.1, 0.12])
    
    # Sensor VLP-16 (Cilindro con lente)
    puck = o3d.geometry.TriangleMesh.create_cylinder(radius=0.15, height=0.25)
    puck.translate([0, 0, 0.85])
    puck.paint_uniform_color([0.8, 0.8, 0.8])
    
    # Ruedas
    for x, y in [(-0.8, 0.65), (0.8, 0.65), (-0.8, -0.65), (0.8, -0.65)]:
        wheel = o3d.geometry.TriangleMesh.create_cylinder(radius=0.35, height=0.25)
        wheel.rotate(o3d.geometry.get_rotation_matrix_from_xyz([np.pi/2, 0, 0]))
        wheel.translate([x, y, 0.35])
        wheel.paint_uniform_color([0.05, 0.05, 0.05])
        car += wheel
        
    return car + puck

def main():
    rclpy.init()
    node = LioSamMasterViz()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="SLAM ARCHITECTURE | V1 MAPPING - V2 LOCALIZATION", width=1440, height=810)
    
    # Geometrías
    global_pcd = o3d.geometry.PointCloud() # El Mapa Global
    live_pcd = o3d.geometry.PointCloud()   # El Rayo Laser en tiempo real
    path_line = o3d.geometry.LineSet()     # La trayectoria (camino)
    car = build_pro_vehicle()              # El coche
    
    vis.add_geometry(global_pcd)
    vis.add_geometry(live_pcd)
    vis.add_geometry(path_line)
    vis.add_geometry(car)
    
    # Configuración de Cámara Técnica
    ctr = vis.get_view_control()
    ctr.set_zoom(0.06)
    ctr.set_lookat([0, 0, 0])
    ctr.set_up([1, 0, 0])
    ctr.set_front([0.5, 0.5, 1.0])
    
    # Fondo y Render
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.01, 0.01, 0.02])
    opt.point_size = 1.0
    
    try:
        while vis.poll_events():
            with node.lock:
                if node.active_scan is not None:
                    # 1. Puntos que están entrando (Blanco Neón)
                    live_pcd.points = o3d.utility.Vector3dVector(node.active_scan)
                    live_pcd.paint_uniform_color([0.0, 1.0, 1.0]) # Cian Pro
                    
                    # 2. Acumular en Mapa Global con FILTRO VOXEL (Limpieza total)
                    map_pts = np.asarray(global_pcd.points)
                    merged = np.vstack([map_pts, node.active_scan]) if len(map_pts) > 0 else node.active_scan
                    
                    # Filtro Voxel 0.2m para que no se vea desordenado
                    temp_pcd = o3d.geometry.PointCloud()
                    temp_pcd.points = o3d.utility.Vector3dVector(merged)
                    global_pcd.points = temp_pcd.voxel_down_sample(0.2).points
                    
                    # Estética de Calor (Altura Z)
                    z_pts = np.asarray(global_pcd.points)[:, 2]
                    z_n = (z_pts - np.min(z_pts)) / (np.max(z_pts) - np.min(z_pts) + 1e-6)
                    colors = np.zeros_like(np.asarray(global_pcd.points))
                    colors[:, 0] = z_n * 0.8 # Red
                    colors[:, 1] = 1.0 - z_n # Green
                    colors[:, 2] = 0.5 # Blue
                    global_pcd.colors = o3d.utility.Vector3dVector(colors)
                    
                    # 3. Trayectoria (Estela estilo LIO-SAM)
                    if node.path_points is not None:
                        path_line.points = o3d.utility.Vector3dVector(node.path_points)
                        lines = [[i, i+1] for i in range(len(node.path_points)-1)]
                        path_line.lines = o3d.utility.Vector2iVector(lines)
                        path_line.paint_uniform_color([1.0, 0.7, 0.0]) # Naranja/Oro
                    
                    # 4. Actualizar Coche
                    car.vertices = build_pro_vehicle().vertices
                    car.transform(node.vehicle_pose)
                    
                    # Centrar cámara suavemente
                    ctr.set_lookat(node.vehicle_pose[:3, 3])
                    
                    vis.update_geometry(global_pcd)
                    vis.update_geometry(live_pcd)
                    vis.update_geometry(path_line)
                    vis.update_geometry(car)
                    node.active_scan = None
            
            vis.update_renderer()
            time.sleep(0.01)
    finally:
        vis.destroy_window()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
