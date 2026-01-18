#!/usr/bin/env python3
"""
Loop Closure Detection Node for ROS 2

Migrated from MATLAB: ENABLE_LOOP_CLOSURE

Features:
- Distance-based loop detection
- Gradual correction to avoid jumps
- Visualization markers for detected loops

Author: Alfonso
Date: 2025
"""

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA


class LoopClosureNode(Node):
    """Loop closure detection and correction node."""
    
    def __init__(self):
        super().__init__('loop_closure')
        
        # Declare parameters
        self.declare_parameter('loop_closure.enable', True)
        self.declare_parameter('loop_closure.distance_threshold', 6.0)
        self.declare_parameter('loop_closure.min_frames_gap', 30)
        self.declare_parameter('loop_closure.max_correction_factor', 0.5)
        self.declare_parameter('frames.map_frame', 'map')
        
        # Load parameters
        self.enable = self.get_parameter('loop_closure.enable').value
        self.distance_threshold = self.get_parameter('loop_closure.distance_threshold').value
        self.min_frames_gap = self.get_parameter('loop_closure.min_frames_gap').value
        self.max_correction_factor = self.get_parameter('loop_closure.max_correction_factor').value
        self.map_frame = self.get_parameter('frames.map_frame').value
        
        if not self.enable:
            self.get_logger().info('Loop closure is DISABLED')
            return
        
        # State
        self.pose_history = []  # List of (position, orientation, frame_id)
        self.loop_detections = []  # List of (current_pos, matched_pos, timestamp)
        self.frame_count = 0
        self.loop_count = 0
        
        # Subscriber
        self.sub_odom = self.create_subscription(
            Odometry, 'input/odom', self.odom_callback, 10
        )
        
        # Publishers
        self.pub_correction = self.create_publisher(
            PoseWithCovarianceStamped, 'output/correction', 10
        )
        self.pub_markers = self.create_publisher(
            MarkerArray, 'output/markers', 10
        )
        
        self.get_logger().info(
            f'Loop Closure initialized - threshold={self.distance_threshold:.1f}m, '
            f'min_gap={self.min_frames_gap} frames'
        )
    
    def odom_callback(self, msg: Odometry):
        """Process incoming odometry for loop detection."""
        if not self.enable:
            return
        
        self.frame_count += 1
        
        current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        current_orientation = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        
        # =====================================================================
        # Check for loop closure candidates
        # =====================================================================
        best_match_idx = -1
        min_distance = float('inf')
        
        for i, (pos, orient, frame_id) in enumerate(self.pose_history):
            # Check frame gap constraint
            frame_gap = self.frame_count - frame_id
            if frame_gap < self.min_frames_gap:
                continue
            
            # Calculate distance
            distance = np.linalg.norm(current_position - pos)
            
            if distance < self.distance_threshold and distance < min_distance:
                min_distance = distance
                best_match_idx = i
        
        # =====================================================================
        # Apply loop closure correction if found
        # =====================================================================
        if best_match_idx >= 0:
            self.loop_count += 1
            
            matched_position = self.pose_history[best_match_idx][0]
            matched_orientation = self.pose_history[best_match_idx][1]
            correction = matched_position - current_position
            
            # Gradual correction (MATLAB: correction_factor = min(0.5, threshold/distance))
            correction_factor = min(
                self.max_correction_factor,
                self.distance_threshold / min_distance
            )
            
            corrected_position = current_position + correction_factor * correction
            
            self.get_logger().info(
                f'🔄 LOOP CLOSURE #{self.loop_count} - Distance: {min_distance:.2f}m, '
                f'Correction: {np.linalg.norm(correction * correction_factor):.2f}m '
                f'(factor: {correction_factor:.2f})'
            )
            
            # Publish correction
            correction_msg = PoseWithCovarianceStamped()
            correction_msg.header.stamp = msg.header.stamp
            correction_msg.header.frame_id = self.map_frame
            
            correction_msg.pose.pose.position.x = corrected_position[0]
            correction_msg.pose.pose.position.y = corrected_position[1]
            correction_msg.pose.pose.position.z = corrected_position[2]
            
            # Interpolate orientation
            corrected_orientation = self.slerp(
                current_orientation, matched_orientation, correction_factor
            )
            
            correction_msg.pose.pose.orientation.x = corrected_orientation[0]
            correction_msg.pose.pose.orientation.y = corrected_orientation[1]
            correction_msg.pose.pose.orientation.z = corrected_orientation[2]
            correction_msg.pose.pose.orientation.w = corrected_orientation[3]
            
            # Set covariance (lower for more confident loops)
            cov = 0.01 + min_distance * 0.1
            correction_msg.pose.covariance[0] = cov
            correction_msg.pose.covariance[7] = cov
            correction_msg.pose.covariance[14] = cov
            correction_msg.pose.covariance[35] = 0.01
            
            self.pub_correction.publish(correction_msg)
            
            # Store loop detection for visualization
            self.loop_detections.append((
                current_position.copy(),
                matched_position.copy(),
                msg.header.stamp
            ))
            
            # Publish visualization markers
            self.publish_loop_markers(msg.header.stamp)
        
        # =====================================================================
        # Store pose in history (every N frames to save memory)
        # =====================================================================
        if self.frame_count % 10 == 0:
            self.pose_history.append((
                current_position.copy(),
                current_orientation.copy(),
                self.frame_count
            ))
            
            # Limit history size
            if len(self.pose_history) > 5000:
                self.pose_history.pop(0)
    
    def publish_loop_markers(self, stamp):
        """Publish visualization markers for loop closures."""
        marker_array = MarkerArray()
        
        marker_id = 0
        for current_pos, matched_pos, _ in self.loop_detections:
            # Line connecting loop endpoints
            line_marker = Marker()
            line_marker.header.stamp = stamp
            line_marker.header.frame_id = self.map_frame
            line_marker.ns = 'loop_closure_lines'
            line_marker.id = marker_id
            marker_id += 1
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.3
            line_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            line_marker.lifetime.sec = 0  # Permanent
            
            p1 = Point(x=current_pos[0], y=current_pos[1], z=current_pos[2])
            p2 = Point(x=matched_pos[0], y=matched_pos[1], z=matched_pos[2])
            line_marker.points = [p1, p2]
            
            marker_array.markers.append(line_marker)
            
            # Sphere at current position
            sphere_marker = Marker()
            sphere_marker.header.stamp = stamp
            sphere_marker.header.frame_id = self.map_frame
            sphere_marker.ns = 'loop_closure_spheres'
            sphere_marker.id = marker_id
            marker_id += 1
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position.x = current_pos[0]
            sphere_marker.pose.position.y = current_pos[1]
            sphere_marker.pose.position.z = current_pos[2]
            sphere_marker.pose.orientation.w = 1.0
            sphere_marker.scale.x = 1.0
            sphere_marker.scale.y = 1.0
            sphere_marker.scale.z = 1.0
            sphere_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)
            sphere_marker.lifetime.sec = 0
            
            marker_array.markers.append(sphere_marker)
        
        self.pub_markers.publish(marker_array)
    
    def slerp(self, q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        """Spherical linear interpolation between two quaternions."""
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        dot = np.dot(q1, q2)
        
        if dot < 0:
            q2 = -q2
            dot = -dot
        
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        theta_0 = np.arccos(dot)
        theta = theta_0 * t
        
        q3 = q2 - q1 * dot
        q3 = q3 / np.linalg.norm(q3)
        
        return q1 * np.cos(theta) + q3 * np.sin(theta)


def main(args=None):
    rclpy.init(args=args)
    node = LoopClosureNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
