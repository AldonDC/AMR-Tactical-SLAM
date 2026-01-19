#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
import math
import numpy as np

class PurePursuitController(Node):
    """
    Pure Pursuit Controller for Autonomous Navigation.
    Uses RTK-SLAM Odometry to follow a set of ENU Waypoints.
    """
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        
        # =========================================
        # PARAMETERS
        # =========================================
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('linear_velocity', 0.5)      # m/s
        self.declare_parameter('max_angular_velocity', 1.0) # rad/s
        self.declare_parameter('goal_tolerance', 1.5)       # m
        
        self.lookahead_dist = self.get_parameter('lookahead_distance').value
        self.v_linear = self.get_parameter('linear_velocity').value
        self.max_omega = self.get_parameter('max_angular_velocity').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        # =========================================
        # STATE
        # =========================================
        self.current_pose = None      # [x, y, yaw]
        self.path = []                # List of [x, y]
        self.target_idx = 0
        self.active = False
        
        # =========================================
        # PUBS & SUBS
        # =========================================
        # Subscriber to SLAM Odom
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
            
        # Optional: Subscriber to a specific Path topic
        self.sub_path = self.create_subscription(
            Path, '/navigation/path', self.path_callback, 10)
            
        # Publisher for robot control
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Vision/Telemetry Pub
        self.pub_lookahead = self.create_publisher(PoseStamped, '/navigation/lookahead_point', 10)
        
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('⚓ Pure Pursuit Controller Initialized')
        self.get_logger().info(f'   Lookahead: {self.lookahead_dist}m, Speed: {self.v_linear}m/s')

    def odom_callback(self, msg: Odometry):
        """Update current robot pose from SLAM."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract Yaw from Quaternion
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))
        
        self.current_pose = [x, y, yaw]

    def path_callback(self, msg: Path):
        """Update the target path."""
        self.path = []
        for p in msg.poses:
            self.path.append([p.pose.position.x, p.pose.position.y])
        
        if len(self.path) > 0:
            self.target_idx = 0
            self.active = True
            self.get_logger().info(f'📍 New Path Received: {len(self.path)} points')

    def get_distance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

    def control_loop(self):
        """Main Pure Pursuit Logic."""
        if not self.active or self.current_pose is None or not self.path:
            return

        # 1. Check if we reached final goal
        dist_to_end = self.get_distance(self.current_pose[:2], self.path[-1])
        if dist_to_end < self.goal_tolerance:
            self.get_logger().info('🎯 GOAL REACHED! Stopping.')
            self.stop_robot()
            self.active = False
            return

        # 2. Find Lookahead point
        # Search for the point on path furthest away but within lookahead distance
        target_pt = None
        for i in range(self.target_idx, len(self.path)):
            d = self.get_distance(self.current_pose[:2], self.path[i])
            if d > self.lookahead_dist:
                target_pt = self.path[i]
                self.target_idx = i
                break
        
        if target_pt is None:
            # If no point is far enough, use the last point
            target_pt = self.path[-1]

        # 3. Transform target point to robot frame
        dx = target_pt[0] - self.current_pose[0]
        dy = target_pt[1] - self.current_pose[1]
        yaw = self.current_pose[2]
        
        # Local coordinates in robot frame
        lx = dx * math.cos(yaw) + dy * math.sin(yaw)
        ly = -dx * math.sin(yaw) + dy * math.cos(yaw)
        
        # 4. Calculate Curvature and Angular Velocity
        # k = 2*dy / L^2
        L_sq = lx**2 + ly**2
        curvature = (2.0 * ly) / L_sq if L_sq > 0.01 else 0.0
        
        omega = self.v_linear * curvature
        
        # Constraint omega
        omega = max(min(omega, self.max_omega), -self.max_omega)
        
        # 5. Publish Command
        cmd = Twist()
        cmd.linear.x = self.v_linear
        cmd.angular.z = omega
        self.pub_cmd.publish(cmd)
        
        # 📊 Telemetry
        self.publish_lookahead_marker(target_pt)

    def stop_robot(self):
        cmd = Twist()
        self.pub_cmd.publish(cmd)

    def publish_lookahead_marker(self, pt):
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'odom'
        ps.pose.position.x = pt[0]
        ps.pose.position.y = pt[1]
        self.pub_lookahead.publish(ps)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
