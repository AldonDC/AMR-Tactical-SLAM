#!/usr/bin/env python3
"""
NDT/ICP Scan Matching Node for ROS 2

Migrated from MATLAB: pcregisterndt / pcregistericp

Features:
- NDT scan-to-scan registration
- ICP fallback for sharp curves
- RTK-guided initial transform
- Adaptive parameters based on trajectory geometry

Author: Alfonso
Date: 2025
"""

import math
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import sensor_msgs_py.point_cloud2 as pc2
import message_filters

# Try to import Open3D for registration
try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("Warning: Open3D not available. Scan matching will use simple ICP fallback.")


class ScanMatchingNode(Node):
    """NDT/ICP scan matching node for SLAM."""
    
    def __init__(self):
        super().__init__('scan_matching')
        
        # Declare parameters - V1 (Mapping) - MÁS CONSERVADOR
        self.declare_parameter('scan_matching.ndt_v1.resolution', 2.0)  # Más fino
        self.declare_parameter('scan_matching.ndt_v1.max_iterations', 80)
        self.declare_parameter('scan_matching.ndt_v1.transformation_epsilon', 0.001)  # Más preciso
        
        # Declare parameters - V2 (Localization)
        self.declare_parameter('scan_matching.ndt_v2.resolution', 1.5)
        self.declare_parameter('scan_matching.ndt_v2.max_iterations', 50)
        self.declare_parameter('scan_matching.ndt_v2.transformation_epsilon', 0.005)
        
        # ICP fallback - MÁS ESTRICTO
        self.declare_parameter('scan_matching.icp_fallback.enable', True)
        self.declare_parameter('scan_matching.icp_fallback.curve_angle_threshold', 20.0)  # Detectar curvas antes
        self.declare_parameter('scan_matching.icp_fallback.max_iterations', 60)
        self.declare_parameter('scan_matching.icp_fallback.max_correspondence_distance', 1.0)  # Más estricto
        
        # Validation - MENOS TOLERANTE A SALTOS
        self.declare_parameter('scan_matching.max_spatial_jump', 2.0)  # Máximo 2m de salto
        self.declare_parameter('scan_matching.divergence_threshold', 1.0)  # Divergencia máxima 1m
        
        # Mode
        self.declare_parameter('mode', 'mapping')
        
        # Load parameters
        self.load_parameters()
        
        # State
        self.cloud_previous = None
        self.current_pose = np.eye(4)  # 4x4 transformation matrix
        self.last_rtk_position = np.zeros(3)
        self.previous_rtk_position = np.zeros(3)
        self.has_rtk = False
        self.pose_history = deque(maxlen=100)
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Synchronized subscribers using message_filters
        self.sub_pointcloud = message_filters.Subscriber(
            self, PointCloud2, 'input/pointcloud', qos_profile=sensor_qos
        )
        self.sub_rtk_odom = message_filters.Subscriber(
            self, Odometry, 'input/rtk_odom'
        )
        
        # Approximate time synchronizer
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_pointcloud, self.sub_rtk_odom],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)
        
        # Independent subscriber for pointcloud-only mode
        self.sub_pointcloud_independent = self.create_subscription(
            PointCloud2,
            'input/pointcloud',
            self.pointcloud_callback,
            sensor_qos
        )
        
        # Publishers
        self.pub_odom = self.create_publisher(Odometry, 'output/slam_odom', 10)
        self.pub_transform = self.create_publisher(
            TransformStamped, 'output/relative_transform', 10
        )
        
        self.get_logger().info(
            f'Scan Matching initialized - mode={self.mode}, '
            f'resolution={self.ndt_resolution:.1f}, max_iter={self.ndt_max_iter}'
        )
    
    def load_parameters(self):
        """Load parameters based on mode."""
        self.mode = self.get_parameter('mode').value
        
        if self.mode == 'localization':
            self.ndt_resolution = self.get_parameter('scan_matching.ndt_v2.resolution').value
            self.ndt_max_iter = self.get_parameter('scan_matching.ndt_v2.max_iterations').value
            self.ndt_epsilon = self.get_parameter('scan_matching.ndt_v2.transformation_epsilon').value
        else:
            self.ndt_resolution = self.get_parameter('scan_matching.ndt_v1.resolution').value
            self.ndt_max_iter = self.get_parameter('scan_matching.ndt_v1.max_iterations').value
            self.ndt_epsilon = self.get_parameter('scan_matching.ndt_v1.transformation_epsilon').value
        
        self.icp_enable = self.get_parameter('scan_matching.icp_fallback.enable').value
        self.curve_angle_threshold = self.get_parameter('scan_matching.icp_fallback.curve_angle_threshold').value
        self.icp_max_iter = self.get_parameter('scan_matching.icp_fallback.max_iterations').value
        self.icp_max_corr_dist = self.get_parameter('scan_matching.icp_fallback.max_correspondence_distance').value
        
        self.max_spatial_jump = self.get_parameter('scan_matching.max_spatial_jump').value
        self.divergence_threshold = self.get_parameter('scan_matching.divergence_threshold').value
    
    def sync_callback(self, cloud_msg: PointCloud2, rtk_msg: Odometry):
        """Callback with synchronized LiDAR and RTK data."""
        # Update RTK pose for initial guess
        self.last_rtk_position = np.array([
            rtk_msg.pose.pose.position.x,
            rtk_msg.pose.pose.position.y,
            rtk_msg.pose.pose.position.z
        ])
        self.has_rtk = True
        
        # Process the point cloud
        self.process_pointcloud(cloud_msg)
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Callback for pointcloud-only processing."""
        if not self.has_rtk:
            self.process_pointcloud(msg)
    
    def process_pointcloud(self, msg: PointCloud2):
        """Process incoming point cloud for registration."""
        # Convert to numpy
        points = self.pointcloud2_to_numpy(msg)
        
        if points is None or len(points) < 100:
            return
        
        # First frame - just store and return
        if self.cloud_previous is None:
            self.cloud_previous = points
            if self.has_rtk:
                self.current_pose[:3, 3] = self.last_rtk_position
            return
        
        # =====================================================================
        # Calculate initial transform from RTK motion
        # =====================================================================
        initial_guess = np.eye(4)
        
        if self.has_rtk and np.linalg.norm(self.previous_rtk_position) > 0:
            rtk_motion = self.last_rtk_position - self.previous_rtk_position
            initial_guess[:3, 3] = rtk_motion
        
        # =====================================================================
        # Detect sharp curves for adaptive algorithm selection
        # =====================================================================
        is_sharp_curve = False
        if len(self.pose_history) >= 3:
            v1 = self.pose_history[-1] - self.pose_history[-2]
            v2 = self.pose_history[-2] - self.pose_history[-3]
            
            norm_v1 = np.linalg.norm(v1)
            norm_v2 = np.linalg.norm(v2)
            
            if norm_v1 > 0.1 and norm_v2 > 0.1:
                cos_angle = np.dot(v1, v2) / (norm_v1 * norm_v2)
                cos_angle = np.clip(cos_angle, -1.0, 1.0)
                turn_angle = np.degrees(np.arccos(cos_angle))
                
                if turn_angle > self.curve_angle_threshold:
                    is_sharp_curve = True
                    self.get_logger().info(
                        f'Sharp curve detected: {turn_angle:.1f}°',
                        throttle_duration_sec=2.0
                    )
        
        # =====================================================================
        # Perform Registration
        # =====================================================================
        relative_transform = np.eye(4)
        registration_success = False
        fitness_score = float('inf')
        
        if HAS_OPEN3D:
            # Create Open3D point clouds
            source_pcd = o3d.geometry.PointCloud()
            source_pcd.points = o3d.utility.Vector3dVector(points[:, :3])
            
            target_pcd = o3d.geometry.PointCloud()
            target_pcd.points = o3d.utility.Vector3dVector(self.cloud_previous[:, :3])
            
            # Downsample for efficiency
            source_down = source_pcd.voxel_down_sample(voxel_size=0.5)
            target_down = target_pcd.voxel_down_sample(voxel_size=0.5)
            
            # Estimate normals for better registration
            source_down.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
            )
            target_down.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
            )
            
            try:
                # Point-to-plane ICP (more robust than point-to-point)
                result = o3d.pipelines.registration.registration_icp(
                    source_down, target_down,
                    max_correspondence_distance=self.icp_max_corr_dist,
                    init=initial_guess,
                    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                        max_iteration=self.ndt_max_iter
                    )
                )
                
                relative_transform = np.array(result.transformation).copy()
                fitness_score = result.fitness
                
                # Validate against RTK
                translation = relative_transform[:3, 3]
                if self.has_rtk and np.linalg.norm(self.previous_rtk_position) > 0:
                    rtk_motion = self.last_rtk_position - self.previous_rtk_position
                    divergence = np.linalg.norm(translation - rtk_motion)
                    
                    if divergence < self.divergence_threshold:
                        registration_success = True
                    else:
                        self.get_logger().warn(
                            f'ICP diverged from RTK: {divergence:.2f}m',
                            throttle_duration_sec=1.0
                        )
                else:
                    registration_success = True
                    
            except Exception as e:
                self.get_logger().debug(f'ICP failed: {e}')
        else:
            # Simple translation-only registration using RTK
            if self.has_rtk:
                relative_transform = initial_guess
                registration_success = True
        
        # =====================================================================
        # Fallback to RTK direct
        # =====================================================================
        if not registration_success and self.has_rtk:
            relative_transform = initial_guess
            registration_success = True
            self.get_logger().warn('Using RTK direct as fallback', throttle_duration_sec=2.0)
        
        if not registration_success:
            self.get_logger().warn('Registration failed, skipping frame', throttle_duration_sec=2.0)
            return
        
        # =====================================================================
        # Validate spatial jump
        # =====================================================================
        translation = relative_transform[:3, 3]
        if np.linalg.norm(translation) > self.max_spatial_jump:
            self.get_logger().warn(
                f'Excessive jump: {np.linalg.norm(translation):.2f}m > {self.max_spatial_jump:.2f}m',
                throttle_duration_sec=1.0
            )
            if self.has_rtk and np.linalg.norm(self.previous_rtk_position) > 0:
                relative_transform[:3, 3] = self.last_rtk_position - self.previous_rtk_position
            else:
                return
        
        # =====================================================================
        # Update global pose
        # =====================================================================
        self.current_pose = self.current_pose @ relative_transform
        
        # Store in history
        current_position = self.current_pose[:3, 3].copy()
        self.pose_history.append(current_position)
        
        # =====================================================================
        # Publish odometry
        # =====================================================================
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position.x = self.current_pose[0, 3]
        odom_msg.pose.pose.position.y = self.current_pose[1, 3]
        odom_msg.pose.pose.position.z = self.current_pose[2, 3]
        
        # Orientation from rotation matrix
        q = self.rotation_matrix_to_quaternion(self.current_pose[:3, :3])
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        # Covariance
        pose_cov = 0.01 + (1.0 - fitness_score) if fitness_score < 1.0 else 0.1
        odom_msg.pose.covariance[0] = pose_cov
        odom_msg.pose.covariance[7] = pose_cov
        odom_msg.pose.covariance[14] = pose_cov
        
        self.pub_odom.publish(odom_msg)
        
        # Publish relative transform
        transform_msg = TransformStamped()
        transform_msg.header.stamp = msg.header.stamp
        transform_msg.header.frame_id = 'previous_frame'
        transform_msg.child_frame_id = 'current_frame'
        transform_msg.transform.translation.x = relative_transform[0, 3]
        transform_msg.transform.translation.y = relative_transform[1, 3]
        transform_msg.transform.translation.z = relative_transform[2, 3]
        q_rel = self.rotation_matrix_to_quaternion(relative_transform[:3, :3])
        transform_msg.transform.rotation.x = q_rel[0]
        transform_msg.transform.rotation.y = q_rel[1]
        transform_msg.transform.rotation.z = q_rel[2]
        transform_msg.transform.rotation.w = q_rel[3]
        self.pub_transform.publish(transform_msg)
        
        # Update state
        self.cloud_previous = points
        self.previous_rtk_position = self.last_rtk_position.copy()
        
        self.get_logger().debug(
            f'Pose: ({self.current_pose[0,3]:.2f}, {self.current_pose[1,3]:.2f}, '
            f'{self.current_pose[2,3]:.2f}) fitness: {fitness_score:.4f}',
            throttle_duration_sec=1.0
        )
    
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
    
    def rotation_matrix_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """Convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
        # Ensure valid rotation matrix
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return np.array([x, y, z, w])


def main(args=None):
    rclpy.init(args=args)
    node = ScanMatchingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
