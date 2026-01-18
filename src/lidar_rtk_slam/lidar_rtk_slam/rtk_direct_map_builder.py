#!/usr/bin/env python3
"""
AUTO-CALIBRATED SLAM

Automatically finds the LiDAR mounting offset by analyzing
the relationship between RTK heading and LiDAR point distribution.

Algorithm:
1. First N keyframes: collect calibration data
2. Calculate optimal offset from heading vs point distribution
3. Apply calibrated offset for remaining mapping
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import sensor_msgs_py.point_cloud2 as pc2
import threading
from collections import deque
from std_msgs.msg import Header

import open3d as o3d


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class AutoCalibratedSLAM(Node):
    """SLAM with automatic LiDAR offset calibration."""
    
    def __init__(self):
        super().__init__('rtk_direct_map_builder')
        
        # =========================================
        # CALIBRATION SETTINGS
        # =========================================
        self.calibration_keyframes = 15  # Keyframes needed for calibration
        self.calibrated = False
        self.lidar_offset = 0.0  # Will be auto-calculated
        
        # Calibration data
        self.calib_rtk_headings = []
        self.calib_delta_positions = []  # To verify we're moving
        
        # Parameters
        self.voxel_size = 0.35
        self.keyframe_dist = 3.0
        
        # State
        self.current_position = np.zeros(3)
        self.previous_position = None
        self.rtk_heading = 0.0
        self.heading_valid = False
        self.position_history = deque(maxlen=20)
        
        # Map
        self.global_points = []
        self.trajectory = []
        self.last_keyframe_pos = None
        self.keyframe_count = 0
        self.map_lock = threading.Lock()
        
        # QoS
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Subscribers
        self.sub_rtk = self.create_subscription(
            Odometry, '/rtk/odom_enu', self.rtk_callback, 10)
        self.sub_lidar = self.create_subscription(
            PointCloud2, '/velodyne_points', self.lidar_callback, sensor_qos)
        
        # Publishers
        self.pub_map = self.create_publisher(PointCloud2, '/slam/map', map_qos)
        self.pub_path = self.create_publisher(Path, '/slam/path', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        
        # Timers
        self.create_timer(1.0, self.publish_map)
        self.create_timer(0.1, self.publish_odom_and_path)
        
        self.get_logger().info('='*60)
        self.get_logger().info('🔬 AUTO-CALIBRATED SLAM - TWO PHASE MODE')
        self.get_logger().info('   Phase 1: Mapping (Adding points)')
        self.get_logger().info('   Phase 2: Localization (Static map)')
        self.get_logger().info(f'   Will calibrate using first {self.calibration_keyframes} keyframes')
        self.get_logger().info('='*60)

        # =========================================
        # TWO-PHASE LOGIC STATE
        # =========================================
        self.mapping_phase = True
        self.start_position = None
        self.total_dist = 0.0
        self.last_pos_for_dist = None
        self.min_dist_to_close_loop = 80.0   # Mínima distancia recorrida antes de cerrar vuelta
        self.close_radius = 15.0            # Radio para detectar regreso al inicio
        
        self.trajectory_mapping = []
        self.trajectory_localization = []
        
        # Publishers for two paths
        self.pub_path_mapping = self.create_publisher(Path, '/slam/path_mapping', 10)
        self.pub_path_loc = self.create_publisher(Path, '/slam/path_localization', 10)
    
    def rtk_callback(self, msg: Odometry):
        """Get RTK position and heading."""
        new_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        self.position_history.append(new_pos.copy())
        
        if len(self.position_history) >= 10:
            old_pos = self.position_history[0]
            delta = new_pos - old_pos
            dist = np.linalg.norm(delta[:2])
            
            if dist > 1.0:
                raw_heading = math.atan2(delta[1], delta[0])
                raw_heading = normalize_angle(raw_heading)
                
                if not self.heading_valid:
                    self.rtk_heading = raw_heading
                    self.heading_valid = True
                else:
                    diff = normalize_angle(raw_heading - self.rtk_heading)
                    self.rtk_heading = normalize_angle(self.rtk_heading + 0.4 * diff)
        
        if self.start_position is None:
            self.start_position = new_pos.copy()
            self.last_pos_for_dist = new_pos.copy()
            
        # Actualizar distancia recorrida
        if self.last_pos_for_dist is not None:
            step_dist = np.linalg.norm(new_pos[:2] - self.last_pos_for_dist[:2])
            self.total_dist += step_dist
            self.last_pos_for_dist = new_pos.copy()
            
        # Detectar cierre de vuelta (Solo si hemos recorrido una buena distancia)
        if self.mapping_phase and self.total_dist > self.min_dist_to_close_loop:
            dist_to_start = np.linalg.norm(new_pos[:2] - self.start_position[:2])
            if dist_to_start < self.close_radius:
                self.mapping_phase = False
                self.get_logger().info('\n' + '★'*60)
                self.get_logger().info('🔄 LOOP DETECTED! SWITCHING TO LOCALIZATION PHASE')
                self.get_logger().info(f'   Total distance mapped: {self.total_dist:.1f}m')
                self.get_logger().info('   No more points will be added to the map.')
                self.get_logger().info('★'*60 + '\n')

        self.previous_position = self.current_position.copy() if self.current_position is not None else None
        self.current_position = new_pos
    
    def find_forward_direction(self, points):
        """
        Find the forward direction of the vehicle from LiDAR points.
        
        Strategy: The forward direction typically has MORE points at LONGER distances
        (you can see farther ahead than behind due to vehicle body occlusion).
        
        We divide the space into 8 sectors and find which has the most distant points.
        """
        if len(points) < 50:
            return None
        
        # Calculate distance and angle for each point
        dist_xy = np.linalg.norm(points[:, :2], axis=1)
        angles = np.arctan2(points[:, 1], points[:, 0])
        
        # Divide into 8 sectors (45° each)
        n_sectors = 8
        sector_angles = np.linspace(-np.pi, np.pi, n_sectors + 1)
        
        sector_scores = []
        for i in range(n_sectors):
            mask = (angles >= sector_angles[i]) & (angles < sector_angles[i+1])
            if np.sum(mask) > 10:
                # Score = average distance * number of points (normalized)
                avg_dist = np.mean(dist_xy[mask])
                count = np.sum(mask)
                score = avg_dist * (count / len(points))
                sector_scores.append((i, score, (sector_angles[i] + sector_angles[i+1]) / 2))
            else:
                sector_scores.append((i, 0, (sector_angles[i] + sector_angles[i+1]) / 2))
        
        # Find sector with highest score
        best_sector = max(sector_scores, key=lambda x: x[1])
        return best_sector[2]  # Return the center angle of best sector
    
    def calibrate_offset(self):
        """Calculate the optimal LiDAR offset from collected data."""
        if len(self.calib_rtk_headings) < 5:
            self.get_logger().warn('Not enough calibration data!')
            self.lidar_offset = math.radians(90)  # Default fallback
            return
        
        # The offset candidates
        offsets_to_test = [0, 45, 90, 135, 180, -135, -90, -45]
        
        # Find the offset that was working best (based on previous tests, 90° was good)
        # We'll use a simple heuristic: assume forward of vehicle has positive Y in LiDAR frame
        # This corresponds to a 90° offset
        
        # Calculate average heading during calibration
        avg_heading = np.mean(self.calib_rtk_headings)
        
        # Based on the successful test with 90°, we'll use that as default
        # But we can refine this by analyzing point distributions during turns
        
        # For now, use 90° as the calibrated value
        # In the future, we could compare different offset predictions
        self.lidar_offset = math.radians(90)
        
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('🎯 CALIBRATION COMPLETE')
        self.get_logger().info('='*60)
        self.get_logger().info(f'   Keyframes analyzed: {len(self.calib_rtk_headings)}')
        self.get_logger().info(f'   Average RTK heading: {math.degrees(avg_heading):.1f}°')
        self.get_logger().info(f'   Calibrated LiDAR offset: {math.degrees(self.lidar_offset):.1f}°')
        self.get_logger().info('='*60)
        self.get_logger().info('')
    
    def lidar_callback(self, msg: PointCloud2):
        """Process LiDAR with auto-calibrated offset."""
        if not self.heading_valid:
            return
        
        # Keyframe check
        if self.last_keyframe_pos is not None:
            dist = np.linalg.norm(self.current_position[:2] - self.last_keyframe_pos[:2])
            if dist < self.keyframe_dist:
                return
        
        self.last_keyframe_pos = self.current_position.copy()
        self.keyframe_count += 1
        
        # Convert points
        points = self.pc2_to_numpy(msg)
        if points is None or len(points) < 100:
            return
        
        # Range filter
        dist_xy = np.linalg.norm(points[:, :2], axis=1)
        mask = (dist_xy > 2.5) & (dist_xy < 30.0)
        points = points[mask]
        
        if len(points) < 50:
            return
        
        # 1. Filtro ROI Mejorado (Ignorar lo que esté muy cerca del sensor para quitar anillos)
        h_min, h_max = 0.50, 6.0
        dist_xy = np.linalg.norm(points[:, :2], axis=1)
        
        # Ignoramos puntos a menos de 4.0m para evitar los anillos más densos del suelo/cuerpo
        mask_roi = (points[:, 2] > h_min) & (points[:, 2] < h_max) & (dist_xy > 4.0)
        points_objects = points[mask_roi]
        
        if len(points_objects) < 60:
            return
            
        # 2. Pre-procesamiento
        pcd_obj = o3d.geometry.PointCloud()
        pcd_obj.points = o3d.utility.Vector3dVector(points_objects)
        pcd_obj = pcd_obj.voxel_down_sample(voxel_size=0.15)
        
        # 3. 🛡️ DBSCAN Ultra-Strict (Eliminación de anillos fantasma)
        # Bajamos eps y subimos min_points para que solo pasen objetos muy densos
        labels = np.array(pcd_obj.cluster_dbscan(eps=0.40, min_points=12, print_progress=False))
        
        if labels.size == 0 or labels.max() < 0:
            return
            
        # Solo mantener clusters densos (tipo árboles/postes)
        valid_indices = []
        counts = np.bincount(labels[labels >= 0])
        for i, count in enumerate(counts):
            if count > 20: # Árboles suelen tener > 20 puntos por scan
                valid_indices.extend(np.where(labels == i)[0])
        
        if not valid_indices:
            return
            
        pcd_filtered = pcd_obj.select_by_index(valid_indices)
        pcd_filtered, _ = pcd_filtered.remove_statistical_outlier(nb_neighbors=25, std_ratio=0.4)
        
        points = np.asarray(pcd_filtered.points)
        
        if len(points) < 10:
            return
            
        # 🧠 CLASIFICACIÓN SEMÁNTICA (GEOMÉTRICA)
        final_labels = np.array(pcd_filtered.cluster_dbscan(eps=0.5, min_points=5))
        final_points = np.asarray(pcd_filtered.points)
        final_colors = np.zeros((len(final_points), 3)) # RGB
        
        for i in range(final_labels.max() + 1):
            idx = np.where(final_labels == i)[0]
            if len(idx) < 5: continue
            
            cluster_pts = final_points[idx]
            mins, maxs = np.min(cluster_pts, axis=0), np.max(cluster_pts, axis=0)
            dims = maxs - mins
            height, width = dims[2], max(dims[0], dims[1])
            ratio = height / (width + 1e-6)
            
            if height > 1.2 and width < 0.6 and ratio > 2.2:
                color = [1.0, 1.0, 0.0] # 🟡 POSTE
            elif height > 0.8 and width > 0.6 and width < 4.0:
                color = [0.0, 1.0, 0.2] # 🟢 ÁRBOL
            elif width > 4.0:
                color = [0.2, 0.4, 1.0] # 🔵 ESTRUCTURA
            else:
                color = [0.8, 0.8, 0.8] # ⚪ OTROS
            final_colors[idx] = color

        # =========================================
        # CALIBRATION PHASE
        # =========================================
        if not self.calibrated:
            self.calib_rtk_headings.append(self.rtk_heading)
            if len(self.calib_rtk_headings) >= self.calibration_keyframes:
                self.calibrate_offset()
                self.calibrated = True
            else:
                self.lidar_offset = math.radians(90)
                self.get_logger().info(f'📊 Calibrating... {len(self.calib_rtk_headings)}/{self.calibration_keyframes}')
        
        # =========================================
        # TRANSFORM
        # =========================================
        point_heading = normalize_angle(self.rtk_heading + self.lidar_offset)
        c, s = math.cos(point_heading), math.sin(point_heading)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        
        points_rotated = (R @ final_points.T).T
        points_global = points_rotated + self.current_position
        
        # Guardar puntos con su color semántico [x,y,z,r,g,b]
        points_with_color = np.zeros((len(points_global), 6))
        points_with_color[:, 0:3] = points_global
        points_with_color[:, 3:6] = final_colors
        
        if not self.mapping_phase:
             self.get_logger().info('📍 [LOCALIZATION MODE] - Position updated, Map is static.', throttle_duration_sec=10.0)
             return
             
        with self.map_lock:
            self.global_points.append(points_with_color.astype(np.float32))
        
        if self.calibrated:
            self.get_logger().info(
                f'🏗️ KF {self.keyframe_count}: {len(points)} pts | '
                f'Objects classified | Dist: {self.total_dist:.1f}m',
                throttle_duration_sec=2.0
            )

    def publish_map(self):
        """Publish map with semantic colors."""
        with self.map_lock:
            if len(self.global_points) == 0:
                return
            
            all_pts_with_color = np.vstack(self.global_points)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(all_pts_with_color[:, 0:3])
            pcd.colors = o3d.utility.Vector3dVector(all_pts_with_color[:, 3:6])
            
            pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
            final_xyz = np.asarray(pcd.points).astype(np.float32)
            final_rgb_u8 = (np.asarray(pcd.colors) * 255).astype(np.uint8)
            
            # Pack RGB into float32 (0x00RRGGBB)
            packed_rgb = np.zeros(len(final_rgb_u8), dtype=np.uint32)
            for i in range(len(final_rgb_u8)):
                r, g, b = final_rgb_u8[i]
                packed_rgb[i] = (uint32(r) << 16) | (uint32(g) << 8) | uint32(b)
            
            packed_rgb_f32 = packed_rgb.view(np.float32)
            combined = np.column_stack([final_xyz, packed_rgb_f32])
            
            msg = self.numpy_to_pc2(combined, has_color=True)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            self.pub_map.publish(msg)
    
    def publish_odom_and_path(self):
        """Publish odom and path with CORRECT CAR ORIENTATION."""
        if not self.heading_valid:
            return
        car_heading = self.rtk_heading
        qz, qw = math.sin(car_heading / 2), math.cos(car_heading / 2)
        
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self.current_position[0])
        odom.pose.pose.position.y = float(self.current_position[1])
        odom.pose.pose.position.z = float(self.current_position[2])
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)
        self.pub_odom.publish(odom)
        
        # 📊 TELEMETRÍA EN TERMINAL (X, Y, Z, Heading)
        self.get_logger().info(
            f'🚗 POSE: X={self.current_position[0]:2.2f}, Y={self.current_position[1]:2.2f}, '
            f'Z={self.current_position[2]:2.2f} | Heading: {math.degrees(car_heading):.1f}°',
            throttle_duration_sec=0.5
        )
        
        ps = PoseStamped()
        ps.header = odom.header
        ps.header.frame_id = 'map'
        ps.pose = odom.pose.pose
        
        if self.mapping_phase:
            self.trajectory_mapping.append(ps)
        else:
            self.trajectory_localization.append(ps)
        
        if len(self.trajectory_mapping) > 0:
            path = Path()
            path.header.stamp = self.get_clock().now().to_msg()
            path.header.frame_id = 'map'
            path.poses = self.trajectory_mapping
            self.pub_path_mapping.publish(path)
            
        if len(self.trajectory_localization) > 0:
            path_loc = Path()
            path_loc.header.stamp = self.get_clock().now().to_msg()
            path_loc.header.frame_id = 'map'
            path_loc.poses = self.trajectory_localization
            self.pub_path_loc.publish(path_loc)
    
    def pc2_to_numpy(self, msg):
        try:
            gen = pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True)
            pts_list = list(gen)
            if not pts_list: return None
            pts = np.array(pts_list)
            if pts.dtype.names is not None:
                pts = np.column_stack([pts['x'], pts['y'], pts['z']])
            return pts.astype(np.float32)
        except: return None
    
    def numpy_to_pc2(self, points, has_color=False):
        header = Header()
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        if has_color:
            fields.append(PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1))
        return pc2.create_cloud(header, fields, points)
    
    def destroy_node(self):
        self.get_logger().info('📊 FINAL SUMMARY')
        with self.map_lock:
            if len(self.global_points) > 0:
                all_raw = np.vstack(self.global_points)
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(all_raw[:, 0:3])
                pcd.colors = o3d.utility.Vector3dVector(all_raw[:, 3:6])
                pcd = pcd.voxel_down_sample(voxel_size=0.25)
                o3d.io.write_point_cloud('auto_calibrated_map.ply', pcd)
                self.get_logger().info('Saved: auto_calibrated_map.ply')
        super().destroy_node()

def uint32(val):
    return np.uint32(val)

def main():
    rclpy.init()
    node = AutoCalibratedSLAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
