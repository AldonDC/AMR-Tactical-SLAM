#!/usr/bin/env python3
"""
LiDAR Preprocessing Node for ROS 2

Migrated from MATLAB: preprocessForProfessionalMap()

Features:
- Ego vehicle point removal
- Range filtering (cylinder)
- RANSAC ground plane removal
- Height filtering
- Voxel grid downsampling

Author: Alfonso
Date: 2025
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# Optional: Open3D for advanced filtering (fallback to numpy if not available)
try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("Warning: Open3D not available, using numpy-based filtering")


class LidarPreprocessingNode(Node):
    """LiDAR point cloud preprocessing node."""
    
    def __init__(self):
        super().__init__('lidar_preprocessing')
        
        # Declare parameters - OPTIMIZADOS PARA MAPA LIMPIO
        self.declare_parameter('preprocessing.ego_radius', 3.0)  # Más grande para quitar puntos del carro
        self.declare_parameter('preprocessing.cylinder_radius', 25.0)  # Más pequeño, solo cercano
        self.declare_parameter('preprocessing.min_height', -0.5)  # Incluir suelo para referencia
        self.declare_parameter('preprocessing.max_height', 4.0)  # Solo edificios bajos, no cielo
        self.declare_parameter('preprocessing.ransac.enable', True)
        self.declare_parameter('preprocessing.ransac.distance_threshold', 0.15)  # Más tolerante
        self.declare_parameter('preprocessing.ransac.max_iterations', 500)
        self.declare_parameter('preprocessing.ransac.ground_tolerance', 0.20)
        self.declare_parameter('preprocessing.voxel.enable', True)
        self.declare_parameter('preprocessing.voxel.leaf_size', 0.25)  # Voxel MÁS GRANDE = puntos más limpios
        
        # Load parameters
        self.ego_radius = self.get_parameter('preprocessing.ego_radius').value
        self.cylinder_radius = self.get_parameter('preprocessing.cylinder_radius').value
        self.min_height = self.get_parameter('preprocessing.min_height').value
        self.max_height = self.get_parameter('preprocessing.max_height').value
        self.ransac_enable = self.get_parameter('preprocessing.ransac.enable').value
        self.ransac_threshold = self.get_parameter('preprocessing.ransac.distance_threshold').value
        self.ransac_max_iter = self.get_parameter('preprocessing.ransac.max_iterations').value
        self.ground_tolerance = self.get_parameter('preprocessing.ransac.ground_tolerance').value
        self.voxel_enable = self.get_parameter('preprocessing.voxel.enable').value
        self.voxel_size = self.get_parameter('preprocessing.voxel.leaf_size').value
        
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Subscriber
        self.sub_pointcloud = self.create_subscription(
            PointCloud2,
            'input/pointcloud',
            self.pointcloud_callback,
            sensor_qos
        )
        
        # Publishers
        self.pub_filtered = self.create_publisher(PointCloud2, 'output/filtered', 10)
        self.pub_ground = self.create_publisher(PointCloud2, 'output/ground', 10)
        
        self.get_logger().info(
            f'LiDAR Preprocessing initialized - ego_r={self.ego_radius:.1f}m, '
            f'cyl_r={self.cylinder_radius:.1f}m, h=[{self.min_height:.1f},{self.max_height:.1f}]m, '
            f'voxel={self.voxel_size:.2f}m'
        )
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Process incoming point cloud."""
        # Convert ROS PointCloud2 to numpy array
        points = self.pointcloud2_to_numpy(msg)
        
        if points is None or len(points) == 0:
            self.get_logger().warn('Received empty point cloud', throttle_duration_sec=5.0)
            return
        
        # =====================================================================
        # Step 1: Ego Vehicle Removal (MATLAB: dists > egoRadius)
        # =====================================================================
        dist_2d = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        mask_ego = (dist_2d > self.ego_radius) & (dist_2d < self.cylinder_radius)
        points_no_ego = points[mask_ego]
        
        if len(points_no_ego) < 50:
            self.get_logger().warn(
                f'Too few points after ego removal: {len(points_no_ego)}',
                throttle_duration_sec=5.0
            )
            return
        
        # =====================================================================
        # Step 2: RANSAC Ground Removal (MATLAB: pcfitplane)
        # =====================================================================
        points_no_ground = points_no_ego
        points_ground = np.array([]).reshape(0, points.shape[1])
        
        if self.ransac_enable and len(points_no_ego) > 100:
            if HAS_OPEN3D:
                # Use Open3D for RANSAC
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points_no_ego[:, :3])
                
                try:
                    plane_model, inliers = pcd.segment_plane(
                        distance_threshold=self.ransac_threshold,
                        ransac_n=3,
                        num_iterations=self.ransac_max_iter
                    )
                    
                    if len(inliers) > 0:
                        # Calculate mean Z of ground plane
                        ground_z_mean = np.mean(points_no_ego[inliers, 2])
                        
                        # Separate ground and non-ground
                        z_diff = np.abs(points_no_ego[:, 2] - ground_z_mean)
                        mask_non_ground = z_diff >= self.ground_tolerance
                        
                        points_no_ground = points_no_ego[mask_non_ground]
                        points_ground = points_no_ego[~mask_non_ground]
                except Exception as e:
                    self.get_logger().debug(f'RANSAC failed: {e}')
            else:
                # Simple height-based ground removal (fallback)
                # Estimate ground as lowest 10% of points
                z_values = points_no_ego[:, 2]
                z_threshold = np.percentile(z_values, 10)
                mask_non_ground = z_values > (z_threshold + self.ground_tolerance)
                points_no_ground = points_no_ego[mask_non_ground]
                points_ground = points_no_ego[~mask_non_ground]
        
        # =====================================================================
        # Step 3: Height Filtering (MATLAB: minHeight, maxHeight)
        # =====================================================================
        if len(points_no_ground) > 0:
            mask_height = (
                (points_no_ground[:, 2] >= self.min_height) &
                (points_no_ground[:, 2] <= self.max_height)
            )
            points_height_filtered = points_no_ground[mask_height]
        else:
            # Fallback to simple height filter on ego-removed points
            mask_height = (
                (points_no_ego[:, 2] >= self.min_height) &
                (points_no_ego[:, 2] <= self.max_height)
            )
            points_height_filtered = points_no_ego[mask_height]
        
        # =====================================================================
        # Step 4: Voxel Grid Downsampling (MATLAB: pcdownsample)
        # =====================================================================
        if self.voxel_enable and len(points_height_filtered) > 100:
            if HAS_OPEN3D:
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points_height_filtered[:, :3])
                pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)
                points_filtered = np.asarray(pcd_down.points)
                
                # Add intensity if available
                if points_height_filtered.shape[1] > 3:
                    # Simple approach: keep first intensity per voxel
                    points_filtered = np.column_stack([
                        points_filtered,
                        np.zeros(len(points_filtered))  # Placeholder intensity
                    ])
            else:
                # Simple voxel grid using numpy
                points_filtered = self.voxel_downsample_numpy(
                    points_height_filtered, self.voxel_size
                )
        else:
            points_filtered = points_height_filtered
        
        # =====================================================================
        # Publish Results
        # =====================================================================
        if len(points_filtered) > 0:
            msg_filtered = self.numpy_to_pointcloud2(
                points_filtered, msg.header
            )
            self.pub_filtered.publish(msg_filtered)
        
        if len(points_ground) > 0:
            msg_ground = self.numpy_to_pointcloud2(
                points_ground, msg.header
            )
            self.pub_ground.publish(msg_ground)
        
        self.get_logger().debug(
            f'Preprocessed: {len(points)} -> {len(points_filtered)} points '
            f'(ground: {len(points_ground)})',
            throttle_duration_sec=2.0
        )
    
    def pointcloud2_to_numpy(self, msg: PointCloud2) -> np.ndarray:
        """Convert PointCloud2 message to numpy array without casting errors."""
        try:
            # Opción más rápida y segura: np.frombuffer con dtype estructurado
            # 1. Definir los nombres de campos que vienen en el mensaje
            field_names = [f.name for f in msg.fields]
            # 2. Leer puntos como estructurados
            gen = pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True)
            pts_struct = np.fromiter(gen, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
            
            if len(pts_struct) == 0: return None
            
            # 3. Convertir a array normal [N, 3] usando stack (método infalible)
            points = np.stack([pts_struct['x'], pts_struct['y'], pts_struct['z']], axis=1)
            return points.astype(np.float32)
            
        except Exception as e:
            self.get_logger().error(f'Error reading point cloud: {e}')
            return None
    
    def numpy_to_pointcloud2(self, points: np.ndarray, header) -> PointCloud2:
        """Convert numpy array to PointCloud2 message."""
        # Ensure 3D points
        if points.shape[1] >= 3:
            xyz = points[:, :3].astype(np.float32)
        else:
            return None
        
        # Create PointCloud2 message
        msg = pc2.create_cloud_xyz32(header, xyz.tolist())
        return msg
    
    def voxel_downsample_numpy(self, points: np.ndarray, voxel_size: float) -> np.ndarray:
        """Simple voxel grid downsampling using numpy."""
        if len(points) == 0:
            return points
        
        # Compute voxel indices
        voxel_indices = np.floor(points[:, :3] / voxel_size).astype(np.int32)
        
        # Create unique voxel keys
        # Use a large multiplier to create unique keys
        keys = (voxel_indices[:, 0] * 1000000 + 
                voxel_indices[:, 1] * 1000 + 
                voxel_indices[:, 2])
        
        # Get unique voxels and their representative points
        _, unique_indices = np.unique(keys, return_index=True)
        
        return points[unique_indices]


def main(args=None):
    rclpy.init(args=args)
    node = LidarPreprocessingNode()
    
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
