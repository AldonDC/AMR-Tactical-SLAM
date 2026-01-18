#!/usr/bin/env python3
"""
Map Building Node for ROS 2

Migrated from MATLAB: buildProfessionalGlobalMap, keyframe selection

Features:
- Keyframe-based map accumulation (distance threshold)
- Voxel grid map merging
- Trajectory-based spatial filtering
- Map size limiting
- PLY export on shutdown

Author: Alfonso
Date: 2025
"""

import math
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import sensor_msgs_py.point_cloud2 as pc2
import threading

# Optional: Open3D for voxel grid and PLY export
try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("Warning: Open3D not available, using numpy-based operations")


class MapBuilderNode(Node):
    """Keyframe-based map building node."""
    
    def __init__(self):
        super().__init__('map_builder')
        
        # Declare parameters - MAPA LIMPIO
        self.declare_parameter('map_builder.keyframe_distance', 2.0)  # Cada 2 metros
        self.declare_parameter('map_builder.voxel_resolution', 0.4)  # Voxel GRANDE = puntos limpios
        self.declare_parameter('map_builder.max_points', 50000)  # Menos puntos, más limpio
        self.declare_parameter('map_builder.trajectory_filter.enable', True)
        self.declare_parameter('map_builder.trajectory_filter.radius', 20.0)  # Solo puntos cercanos a trayectoria
        self.declare_parameter('visualization.publish_rate', 2.0)  # Actualizar más frecuente
        self.declare_parameter('visualization.path_publish_rate', 10.0)
        self.declare_parameter('output.export_ply', True)
        self.declare_parameter('output.ply_filename', 'final_map_3d.ply')
        self.declare_parameter('frames.map_frame', 'map')
        
        # Load parameters
        self.keyframe_distance = self.get_parameter('map_builder.keyframe_distance').value
        self.voxel_resolution = self.get_parameter('map_builder.voxel_resolution').value
        self.max_points = self.get_parameter('map_builder.max_points').value
        self.map_frame = 'map'
        self.trajectory_filter_enable = self.get_parameter('map_builder.trajectory_filter.enable').value
        self.trajectory_filter_radius = self.get_parameter('map_builder.trajectory_filter.radius').value
        self.map_publish_rate = self.get_parameter('visualization.publish_rate').value
        self.path_publish_rate = self.get_parameter('visualization.path_publish_rate').value
        self.export_ply = self.get_parameter('output.export_ply').value
        self.ply_filename = self.get_parameter('output.ply_filename').value
        self.map_frame = self.get_parameter('frames.map_frame').value
        
        # State
        self.global_map_points = np.array([]).reshape(0, 3)
        self.current_pose = None
        self.current_stamp = None
        self.has_pose = False
        self.trajectory = deque(maxlen=10000)
        self.last_keyframe_position = np.zeros(3)
        self.keyframe_count = 0
        
        # === MODO 2 FASES: Mapeo -> Localización ===
        self.origin_position = None  # Posición inicial
        self.mapping_mode = True  # True = mapeando, False = solo localizando
        self.lap_count = 0  # Contador de vueltas
        self.min_distance_for_lap = 50.0  # Distancia mínima antes de detectar vuelta
        self.lap_detection_radius = 10.0  # Radio para detectar que volvió al origen
        self.total_distance_traveled = 0.0
        self.last_position_for_distance = None
        
        # Locks for thread safety
        self.map_lock = threading.Lock()
        self.pose_lock = threading.Lock()
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=5
        )
        
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Subscribers
        self.sub_pointcloud = self.create_subscription(
            PointCloud2, 'input/pointcloud', self.pointcloud_callback, sensor_qos
        )
        self.sub_odom = self.create_subscription(
            Odometry, 'input/odom', self.odom_callback, 10
        )
        
        # Publishers
        self.pub_map = self.create_publisher(PointCloud2, 'output/map', map_qos)
        self.pub_keyframes = self.create_publisher(PointCloud2, 'output/keyframes', 10)
        self.pub_path = self.create_publisher(Path, 'output/path', 10)
        
        # Timers
        self.map_timer = self.create_timer(1.0 / self.map_publish_rate, self.publish_map)
        self.path_timer = self.create_timer(1.0 / self.path_publish_rate, self.publish_path)
        
        self.get_logger().info(
            f'Map Builder initialized - keyframe_dist={self.keyframe_distance:.1f}m, '
            f'voxel={self.voxel_resolution:.2f}m, max_pts={self.max_points}'
        )
    
    def destroy_node(self):
        """Export map on shutdown."""
        if self.export_ply and len(self.global_map_points) > 0:
            self.get_logger().info(f'Exporting map to {self.ply_filename}...')
            try:
                if HAS_OPEN3D:
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(self.global_map_points)
                    o3d.io.write_point_cloud(self.ply_filename, pcd)
                else:
                    # Simple PLY export without Open3D
                    with open(self.ply_filename, 'w') as f:
                        f.write('ply\n')
                        f.write('format ascii 1.0\n')
                        f.write(f'element vertex {len(self.global_map_points)}\n')
                        f.write('property float x\n')
                        f.write('property float y\n')
                        f.write('property float z\n')
                        f.write('end_header\n')
                        for p in self.global_map_points:
                            f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')
                
                self.get_logger().info(f'Map exported: {len(self.global_map_points)} points')
            except Exception as e:
                self.get_logger().error(f'Failed to export map: {e}')
        
        super().destroy_node()
    
    def odom_callback(self, msg: Odometry):
        """Store current pose from odometry."""
        with self.pose_lock:
            self.current_pose = msg.pose.pose
            self.current_stamp = msg.header.stamp
            self.has_pose = True
            
            # Store in trajectory
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose.pose
            self.trajectory.append(ps)
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Process incoming point cloud for map building."""
        if not self.has_pose:
            self.get_logger().warn('No odometry received yet', throttle_duration_sec=5.0)
            return
        
        # Get current pose (thread-safe)
        with self.pose_lock:
            pose = self.current_pose
            if pose is None:
                return
            
            current_position = np.array([
                pose.position.x,
                pose.position.y,
                pose.position.z
            ])
        
        # =====================================================================
        # DETECCIÓN DE VUELTA (Cambio de Mapeo a Localización)
        # =====================================================================
        if self.origin_position is None:
            self.origin_position = current_position.copy()
            self.get_logger().info(f'🚗 Origen establecido en ({current_position[0]:.1f}, {current_position[1]:.1f})')
        
        # Calcular distancia recorrida
        if self.last_position_for_distance is not None:
            step_distance = np.linalg.norm(current_position[:2] - self.last_position_for_distance[:2])
            self.total_distance_traveled += step_distance
        self.last_position_for_distance = current_position.copy()
        
        # Detectar si volvió al origen (completó una vuelta)
        if self.mapping_mode and self.total_distance_traveled > self.min_distance_for_lap:
            distance_to_origin = np.linalg.norm(current_position[:2] - self.origin_position[:2])
            if distance_to_origin < self.lap_detection_radius:
                self.lap_count += 1
                self.mapping_mode = False
                self.get_logger().info(
                    f'🏁 ¡VUELTA {self.lap_count} COMPLETADA! Distancia: {self.total_distance_traveled:.1f}m\n'
                    f'   📍 Cambiando a MODO LOCALIZACIÓN (ya no se añaden puntos al mapa)\n'
                    f'   🗺️  Mapa final: {len(self.global_map_points)} puntos'
                )
        
        # =====================================================================
        # Keyframe Selection (MATLAB: distance_from_last_keyframe >= threshold)
        # =====================================================================
        distance_from_last = np.linalg.norm(current_position - self.last_keyframe_position)
        
        if distance_from_last < self.keyframe_distance:
            return  # Not a keyframe
        
        # This is a keyframe!
        self.last_keyframe_position = current_position.copy()
        self.keyframe_count += 1
        
        # =====================================================================
        # Si estamos en modo LOCALIZACIÓN, no añadir puntos al mapa
        # =====================================================================
        if not self.mapping_mode:
            self.get_logger().debug(
                f'Localización: pose ({current_position[0]:.1f}, {current_position[1]:.1f})',
                throttle_duration_sec=2.0
            )
            return  # Solo localiza, no mapea
        
        # =====================================================================
        # Convert point cloud to numpy
        # =====================================================================
        points = self.pointcloud2_to_numpy(msg)
        if points is None or len(points) < 50:
            return
        
        # =====================================================================
        # Transform to map frame
        # =====================================================================
        # Get rotation quaternion
        q = np.array([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        R = self.quaternion_to_rotation_matrix(q)
        t = current_position
        
        # Transform points
        points_transformed = (R @ points[:, :3].T).T + t
        
        # =====================================================================
        # Trajectory-based spatial filtering
        # =====================================================================
        if self.trajectory_filter_enable:
            distances = np.linalg.norm(points_transformed - current_position, axis=1)
            mask = distances < self.trajectory_filter_radius
            points_transformed = points_transformed[mask]
        
        if len(points_transformed) == 0:
            return
        
        # =====================================================================
        # Merge into global map
        # =====================================================================
        with self.map_lock:
            self.global_map_points = np.vstack([
                self.global_map_points, 
                points_transformed
            ]) if len(self.global_map_points) > 0 else points_transformed
            
            # Voxel grid downsampling if exceeding max points
            if len(self.global_map_points) > self.max_points:
                if HAS_OPEN3D:
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(self.global_map_points)
                    pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_resolution)
                    self.global_map_points = np.asarray(pcd_down.points)
                else:
                    # Simple numpy voxel grid
                    self.global_map_points = self.voxel_downsample(
                        self.global_map_points, self.voxel_resolution
                    )
                
                self.get_logger().debug(
                    f'Map downsampled: {len(self.global_map_points)} points'
                )
        
        # Publish keyframe cloud
        self.publish_keyframe(points_transformed, msg.header)
        
        self.get_logger().info(
            f'Keyframe {self.keyframe_count} added, map size: {len(self.global_map_points)} points',
            throttle_duration_sec=5.0
        )
    
    def publish_map(self):
        """Periodically publish global map."""
        with self.map_lock:
            if len(self.global_map_points) == 0:
                return
            
            msg = self.numpy_to_pointcloud2(self.global_map_points)
            if msg is not None:
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.map_frame
                self.pub_map.publish(msg)
    
    def publish_path(self):
        """Periodically publish trajectory path."""
        with self.pose_lock:
            if len(self.trajectory) == 0:
                return
            
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = self.map_frame
            path_msg.poses = list(self.trajectory)
            
            self.pub_path.publish(path_msg)
    
    def publish_keyframe(self, points: np.ndarray, header):
        """Publish keyframe point cloud."""
        msg = self.numpy_to_pointcloud2(points)
        if msg is not None:
            msg.header = header
            msg.header.frame_id = self.map_frame
            self.pub_keyframes.publish(msg)
    
    def pointcloud2_to_numpy(self, msg: PointCloud2) -> np.ndarray:
        """Convert PointCloud2 to numpy array without casting errors."""
        try:
            gen = pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True)
            pts_struct = np.fromiter(gen, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
            
            if len(pts_struct) == 0: return None
            
            return np.stack([pts_struct['x'], pts_struct['y'], pts_struct['z']], axis=1).astype(np.float32)
        except Exception as e:
            self.get_logger().error(f'Error reading point cloud: {e}')
            return None
    
    def numpy_to_pointcloud2(self, points: np.ndarray) -> PointCloud2:
        """Convert numpy array to PointCloud2."""
        from std_msgs.msg import Header
        header = Header()
        return pc2.create_cloud_xyz32(header, points.tolist())
    
    def quaternion_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        """Convert quaternion [x, y, z, w] to 3x3 rotation matrix."""
        x, y, z, w = q
        
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ])
        
        return R
    
    def voxel_downsample(self, points: np.ndarray, voxel_size: float) -> np.ndarray:
        """Simple voxel grid downsampling using numpy."""
        if len(points) == 0:
            return points
        
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)
        keys = voxel_indices[:, 0] * 1000000 + voxel_indices[:, 1] * 1000 + voxel_indices[:, 2]
        _, unique_indices = np.unique(keys, return_index=True)
        
        return points[unique_indices]


def main(args=None):
    rclpy.init(args=args)
    node = MapBuilderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
