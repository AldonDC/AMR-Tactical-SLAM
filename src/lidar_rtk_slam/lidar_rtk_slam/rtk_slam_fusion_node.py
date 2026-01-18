#!/usr/bin/env python3
"""
RTK-SLAM Fusion Node for ROS 2

Migrated from MATLAB: RTK_FOLLOW_PRECISION weighted fusion

Features:
- Weighted fusion of RTK and SLAM poses
- Adaptive weight based on drift detection
- Mode-specific weights (V1 mapping vs V2 localization)
- Diagnostics publishing

Author: Alfonso
Date: 2025
"""

import math
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import message_filters


class RTKSLAMFusionNode(Node):
    """RTK-SLAM fusion node with adaptive weighted pose combination."""
    
    def __init__(self):
        super().__init__('rtk_slam_fusion')
        
        # Declare parameters - RTK TIENE PRIORIDAD
        self.declare_parameter('mode', 'mapping')
        self.declare_parameter('fusion.rtk_weight_v1', 0.95)  # RTK domina en mapeo
        self.declare_parameter('fusion.rtk_weight_v2', 0.98)  # RTK domina en localización
        self.declare_parameter('fusion.drift.enable', True)
        self.declare_parameter('fusion.drift.threshold', 0.5)  # Detectar drift más rápido
        self.declare_parameter('fusion.drift.boost_weight', 0.99)  # Casi 100% RTK cuando hay drift
        self.declare_parameter('fusion.smooth_factor', 0.5)  # Menos suavizado
        self.declare_parameter('frames.map_frame', 'map')
        self.declare_parameter('frames.odom_frame', 'odom')
        self.declare_parameter('frames.base_frame', 'base_link')
        
        # Load parameters
        self.mode = self.get_parameter('mode').value
        self.rtk_weight_v1 = self.get_parameter('fusion.rtk_weight_v1').value
        self.rtk_weight_v2 = self.get_parameter('fusion.rtk_weight_v2').value
        self.drift_enable = self.get_parameter('fusion.drift.enable').value
        self.drift_threshold = self.get_parameter('fusion.drift.threshold').value
        self.drift_boost_weight = self.get_parameter('fusion.drift.boost_weight').value
        self.smooth_factor = self.get_parameter('fusion.smooth_factor').value
        self.map_frame = self.get_parameter('frames.map_frame').value
        self.odom_frame = self.get_parameter('frames.odom_frame').value
        self.base_frame = self.get_parameter('frames.base_frame').value
        
        # Set current weight based on mode
        self.current_rtk_weight = self.rtk_weight_v2 if self.mode == 'localization' else self.rtk_weight_v1
        
        # State
        self.divergence_history = deque(maxlen=50)
        self.drift_correction_count = 0
        self.previous_fused_pos = np.zeros(3)
        self.has_previous_pose = False
        self.last_valid_pos = np.zeros(3)
        self.has_valid_pose = False
        
        # Synchronized subscribers
        self.sub_slam_odom = message_filters.Subscriber(self, Odometry, 'input/slam_odom')
        self.sub_rtk_odom = message_filters.Subscriber(self, Odometry, 'input/rtk_odom')
        
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_slam_odom, self.sub_rtk_odom],
            queue_size=20,
            slop=0.5
        )
        self.sync.registerCallback(self.sync_callback)
        
        # Publishers
        self.pub_fused_odom = self.create_publisher(Odometry, 'output/fused_odom', 10)
        self.pub_diagnostics = self.create_publisher(DiagnosticStatus, 'output/diagnostics', 10)
        
        self.get_logger().info(
            f'RTK-SLAM Fusion initialized - mode={self.mode}, '
            f'rtk_weight={self.current_rtk_weight:.2f}, drift_threshold={self.drift_threshold:.1f}m'
        )
    
    def sync_callback(self, slam_msg: Odometry, rtk_msg: Odometry):
        """Process synchronized SLAM and RTK odometry."""
        self.get_logger().info('Fusion: Sync received (GPS + SLAM)', throttle_duration_sec=10.0)
        # Extract positions
        slam_pos = np.array([
            slam_msg.pose.pose.position.x,
            slam_msg.pose.pose.position.y,
            slam_msg.pose.pose.position.z
        ])
        
        rtk_pos = np.array([
            rtk_msg.pose.pose.position.x,
            rtk_msg.pose.pose.position.y,
            rtk_msg.pose.pose.position.z
        ])
        
        # =====================================================================
        # Drift Detection (MATLAB: RTK_V2_DRIFT_THRESHOLD)
        # =====================================================================
        divergence = np.linalg.norm(slam_pos - rtk_pos)
        drift_detected = False
        adaptive_weight = self.current_rtk_weight
        
        if self.drift_enable:
            self.divergence_history.append(divergence)
            
            if divergence > self.drift_threshold:
                drift_detected = True
                self.drift_correction_count += 1
                adaptive_weight = self.drift_boost_weight
                
                self.get_logger().warn(
                    f'Drift detected: {divergence:.2f}m, boosting RTK weight to {adaptive_weight:.2f}',
                    throttle_duration_sec=1.0
                )
        
        # =====================================================================
        # Weighted Fusion (MATLAB: combined_translation = α*RTK + (1-α)*SLAM)
        # =====================================================================
        fused_pos = adaptive_weight * rtk_pos + (1.0 - adaptive_weight) * slam_pos
        
        # For orientation, prefer SLAM (has geometric information from LiDAR)
        slam_quat = np.array([
            slam_msg.pose.pose.orientation.x,
            slam_msg.pose.pose.orientation.y,
            slam_msg.pose.pose.orientation.z,
            slam_msg.pose.pose.orientation.w
        ])
        
        rtk_quat = np.array([
            rtk_msg.pose.pose.orientation.x,
            rtk_msg.pose.pose.orientation.y,
            rtk_msg.pose.pose.orientation.z,
            rtk_msg.pose.pose.orientation.w
        ])
        
        # SLERP interpolation for orientation (mostly SLAM, slight RTK influence)
        orientation_rtk_weight = 0.1
        fused_quat = self.slerp(slam_quat, rtk_quat, orientation_rtk_weight)
        
        # =====================================================================
        # Temporal Smoothing
        # =====================================================================
        if self.has_previous_pose and self.smooth_factor > 0:
            fused_pos = self.smooth_factor * fused_pos + (1.0 - self.smooth_factor) * self.previous_fused_pos
        
        self.previous_fused_pos = fused_pos.copy()
        self.has_previous_pose = True
        
        # Store as last valid pose if within threshold
        if divergence < self.drift_threshold:
            self.last_valid_pos = fused_pos.copy()
            self.has_valid_pose = True
        
        # =====================================================================
        # Publish Fused Odometry
        # =====================================================================
        fused_odom = Odometry()
        fused_odom.header.stamp = slam_msg.header.stamp
        fused_odom.header.frame_id = self.map_frame
        fused_odom.child_frame_id = self.base_frame
        
        fused_odom.pose.pose.position.x = fused_pos[0]
        fused_odom.pose.pose.position.y = fused_pos[1]
        fused_odom.pose.pose.position.z = fused_pos[2]
        
        fused_odom.pose.pose.orientation.x = fused_quat[0]
        fused_odom.pose.pose.orientation.y = fused_quat[1]
        fused_odom.pose.pose.orientation.z = fused_quat[2]
        fused_odom.pose.pose.orientation.w = fused_quat[3]
        
        # Combine covariances (weighted sum)
        for i in range(36):
            fused_odom.pose.covariance[i] = (
                adaptive_weight * rtk_msg.pose.covariance[i] +
                (1.0 - adaptive_weight) * slam_msg.pose.covariance[i]
            )
        
        # Use SLAM velocity
        fused_odom.twist = slam_msg.twist
        
        self.pub_fused_odom.publish(fused_odom)
        
        # =====================================================================
        # Publish Diagnostics
        # =====================================================================
        diag = DiagnosticStatus()
        diag.name = 'SLAM Fusion'
        diag.level = DiagnosticStatus.WARN if drift_detected else DiagnosticStatus.OK
        diag.message = 'Drift detected - using RTK correction' if drift_detected else 'Normal operation'
        
        diag.values.append(KeyValue(key='mode', value=self.mode))
        diag.values.append(KeyValue(key='rtk_weight', value=f'{adaptive_weight:.3f}'))
        diag.values.append(KeyValue(key='divergence_m', value=f'{divergence:.3f}'))
        diag.values.append(KeyValue(key='drift_corrections', value=str(self.drift_correction_count)))
        
        if len(self.divergence_history) > 0:
            avg_div = sum(self.divergence_history) / len(self.divergence_history)
            diag.values.append(KeyValue(key='avg_divergence_m', value=f'{avg_div:.3f}'))
        
        diag.values.append(KeyValue(key='fused_x', value=f'{fused_pos[0]:.3f}'))
        diag.values.append(KeyValue(key='fused_y', value=f'{fused_pos[1]:.3f}'))
        diag.values.append(KeyValue(key='fused_z', value=f'{fused_pos[2]:.3f}'))
        
        self.pub_diagnostics.publish(diag)
        
        self.get_logger().debug(
            f'Fusion: pos=({fused_pos[0]:.2f},{fused_pos[1]:.2f},{fused_pos[2]:.2f}) '
            f'div={divergence:.3f}m w={adaptive_weight:.2f}',
            throttle_duration_sec=1.0
        )
    
    def slerp(self, q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        """Spherical linear interpolation between two quaternions."""
        # Normalize quaternions
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        # Compute dot product
        dot = np.dot(q1, q2)
        
        # If dot is negative, negate one quaternion to take shorter path
        if dot < 0:
            q2 = -q2
            dot = -dot
        
        # If very close, use linear interpolation
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        # Standard SLERP
        theta_0 = np.arccos(dot)
        theta = theta_0 * t
        
        q3 = q2 - q1 * dot
        q3 = q3 / np.linalg.norm(q3)
        
        result = q1 * np.cos(theta) + q3 * np.sin(theta)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RTKSLAMFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
