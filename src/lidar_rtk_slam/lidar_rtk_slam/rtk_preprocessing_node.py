#!/usr/bin/env python3
"""
RTK GPS Preprocessing Node for ROS 2

Migrated from MATLAB: detectRTK_3D_Enhanced()

Features:
- LLA to ENU conversion (local tangent plane)
- Hampel filter for outlier removal
- Velocity-based jump detection
- Lever arm correction
- Heading estimation from motion

Author: Alfonso
Date: 2025
"""

import math
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from scipy.spatial.transform import Rotation as R

# WGS84 constants
WGS84_A = 6378137.0           # Semi-major axis (m)
WGS84_E2 = 0.00669437999014   # First eccentricity squared


class RTKPreprocessingNode(Node):
    """RTK GPS preprocessing node - converts NavSatFix to ENU odometry."""
    
    def __init__(self):
        super().__init__('rtk_preprocessing')
        
        # Declare parameters
        self.declare_parameter('rtk_preprocessing.origin.auto_set', True)
        self.declare_parameter('rtk_preprocessing.origin.latitude', 0.0)
        self.declare_parameter('rtk_preprocessing.origin.longitude', 0.0)
        self.declare_parameter('rtk_preprocessing.origin.altitude', 0.0)
        self.declare_parameter('rtk_preprocessing.hampel.enable', True)
        self.declare_parameter('rtk_preprocessing.hampel.window_size', 9)
        self.declare_parameter('rtk_preprocessing.hampel.threshold', 3.0)
        self.declare_parameter('rtk_preprocessing.max_speed_ms', 12.0)
        self.declare_parameter('rtk_preprocessing.lever_arm.x', 0.0)
        self.declare_parameter('rtk_preprocessing.lever_arm.y', 0.0)
        self.declare_parameter('rtk_preprocessing.lever_arm.z', 0.0)
        self.declare_parameter('rtk_preprocessing.z_weight', 2.0)
        self.declare_parameter('frames.map_frame', 'map')
        self.declare_parameter('frames.odom_frame', 'odom')
        
        # Load parameters
        self.origin_auto_set = self.get_parameter('rtk_preprocessing.origin.auto_set').value
        self.origin_lat = self.get_parameter('rtk_preprocessing.origin.latitude').value
        self.origin_lon = self.get_parameter('rtk_preprocessing.origin.longitude').value
        self.origin_alt = self.get_parameter('rtk_preprocessing.origin.altitude').value
        self.hampel_enable = self.get_parameter('rtk_preprocessing.hampel.enable').value
        self.hampel_window = self.get_parameter('rtk_preprocessing.hampel.window_size').value
        self.hampel_threshold = self.get_parameter('rtk_preprocessing.hampel.threshold').value
        self.max_speed = self.get_parameter('rtk_preprocessing.max_speed_ms').value
        self.lever_arm_x = self.get_parameter('rtk_preprocessing.lever_arm.x').value
        self.lever_arm_y = self.get_parameter('rtk_preprocessing.lever_arm.y').value
        self.lever_arm_z = self.get_parameter('rtk_preprocessing.lever_arm.z').value
        self.z_weight = self.get_parameter('rtk_preprocessing.z_weight').value
        self.map_frame = self.get_parameter('frames.map_frame').value
        self.odom_frame = self.get_parameter('frames.odom_frame').value
        
        # State
        self.origin_set = not self.origin_auto_set
        self.history_e = deque(maxlen=self.hampel_window)
        self.history_n = deque(maxlen=self.hampel_window)
        self.history_u = deque(maxlen=self.hampel_window)
        self.pose_history = deque(maxlen=50)
        
        self.last_valid_e = 0.0
        self.last_valid_n = 0.0
        self.last_valid_u = 0.0
        self.last_valid_time = None
        self.last_velocity_x = 0.0
        self.last_velocity_y = 0.0
        
        # Subscriber
        self.sub_fix = self.create_subscription(
            NavSatFix,
            'input/fix',
            self.fix_callback,
            10
        )
        
        # Publishers
        self.pub_odom = self.create_publisher(Odometry, 'output/odom_enu', 10)
        self.pub_pose = self.create_publisher(PoseStamped, 'output/pose_enu', 10)
        
        self.get_logger().info(
            f'RTK Preprocessing initialized - hampel_w={self.hampel_window}, '
            f'max_speed={self.max_speed:.1f}m/s, z_weight={self.z_weight:.1f}'
        )
    
    def lla_to_enu(self, lat: float, lon: float, alt: float,
                   lat0: float, lon0: float, alt0: float) -> tuple:
        """Convert LLA (lat/lon/alt) to ENU (East/North/Up) coordinates."""
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        lat0_rad = math.radians(lat0)
        lon0_rad = math.radians(lon0)
        
        dlat = lat_rad - lat0_rad
        dlon = lon_rad - lon0_rad
        dalt = alt - alt0
        
        sin_lat0 = math.sin(lat0_rad)
        cos_lat0 = math.cos(lat0_rad)
        N0 = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat0 * sin_lat0)
        
        e = dlon * (N0 + alt0) * cos_lat0
        n = dlat * (N0 * (1.0 - WGS84_E2) / (1.0 - WGS84_E2 * sin_lat0 * sin_lat0) + alt0)
        u = dalt
        
        return e, n, u
    
    def hampel_filter_check(self, value: float, history: deque) -> bool:
        """Hampel filter for outlier removal."""
        if len(history) < 3:
            return False
        
        sorted_vals = sorted(history)
        median = sorted_vals[len(sorted_vals) // 2]
        
        abs_devs = sorted([abs(v - median) for v in history])
        mad = abs_devs[len(abs_devs) // 2] * 1.4826
        
        if mad < 1e-6:
            return False
        
        return abs(value - median) > self.hampel_threshold * mad
    
    def fix_callback(self, msg: NavSatFix):
        """Process incoming NavSatFix message."""
        if msg.status.status < NavSatStatus.STATUS_FIX:
            self.get_logger().warn(
                f'No valid GPS fix - status: {msg.status.status}',
                throttle_duration_sec=5.0
            )
            return
        
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        
        # Check for NaN values
        if math.isnan(lat) or math.isnan(lon) or math.isnan(alt):
            return

        # Set origin on first valid fix
        if not self.origin_set and self.origin_auto_set:
            self.origin_lat = lat
            self.origin_lon = lon
            self.origin_alt = alt
            self.origin_set = True
            self.get_logger().info(
                f'RTK origin set: lat={self.origin_lat:.7f}, '
                f'lon={self.origin_lon:.7f}, alt={self.origin_alt:.2f}'
            )
        
        if not self.origin_set:
            return
        
        # LLA to ENU conversion
        e, n, u = self.lla_to_enu(lat, lon, alt, 
                                  self.origin_lat, self.origin_lon, self.origin_alt)
        
        # Hampel filter
        is_outlier = False
        if self.hampel_enable:
            is_outlier = (
                self.hampel_filter_check(e, self.history_e) or
                self.hampel_filter_check(n, self.history_n) or
                self.hampel_filter_check(u, self.history_u)
            )
        
        self.history_e.append(e)
        self.history_n.append(n)
        self.history_u.append(u)
        
        if is_outlier:
            return
        
        # Velocity check
        current_time = rclpy.time.Time.from_msg(msg.header.stamp)
        
        if self.last_valid_time is not None:
            dt = (current_time - self.last_valid_time).nanoseconds / 1e9
            if dt > 0.001:
                dx = e - self.last_valid_e
                dy = n - self.last_valid_n
                speed = math.sqrt(dx*dx + dy*dy) / dt
                
                if speed > self.max_speed:
                    return
                
                self.last_velocity_x = dx / dt
                self.last_velocity_y = dy / dt
        
        # Lever arm correction
        e += self.lever_arm_x
        n += self.lever_arm_y
        u += self.lever_arm_z
        
        self.last_valid_e = e
        self.last_valid_n = n
        self.last_valid_u = u
        self.last_valid_time = current_time
        
        # Calculate heading from motion
        heading = 0.0
        if len(self.pose_history) >= 2:
            prev = self.pose_history[-1]
            de = e - prev['x']
            dn = n - prev['y']
            if math.sqrt(de*de + dn*dn) > 0.1:
                heading = math.atan2(dn, de)
        
        self.pose_history.append({'x': e, 'y': n, 'z': u, 'heading': heading})
        
        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = 'rtk_antenna'
        
        odom_msg.pose.pose.position.x = e
        odom_msg.pose.pose.position.y = n
        odom_msg.pose.pose.position.z = u
        
        # Orientation from heading using scipy
        rot = R.from_euler('xyz', [0, 0, heading])
        q = rot.as_quat()  # Returns [x, y, z, w]
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        # Covariance
        pos_cov = 0.1
        if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            pos_cov = math.sqrt(msg.position_covariance[0])
        
        odom_msg.pose.covariance[0] = pos_cov * pos_cov
        odom_msg.pose.covariance[7] = pos_cov * pos_cov
        odom_msg.pose.covariance[14] = pos_cov * pos_cov
        odom_msg.pose.covariance[35] = 0.1
        
        odom_msg.twist.twist.linear.x = self.last_velocity_x
        odom_msg.twist.twist.linear.y = self.last_velocity_y
        
        self.pub_odom.publish(odom_msg)
        
        # Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose = odom_msg.pose.pose
        self.pub_pose.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RTKPreprocessingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
