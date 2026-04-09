#!/usr/bin/env python3
"""
follow_me_node.py  –  Smart Cart Follow-Me
==========================================
Tracks the nearest object in the front LiDAR arc and drives
the cart to maintain a 1.0m following distance.

CRITICAL: This node only drives the cart when navigation_node
is in FOLLOW mode. It subscribes to /nav/current_mode and
suppresses all output when mode is IDLE or STOP.

This means the cart will NOT move at all until the user
presses remote button 2 on the teleop window.

Topics
------
  Sub: /scan             LaserScan  – find the person
  Sub: /nav/current_mode String     – gate on follow mode only
  Pub: /cmd_vel_raw      Twist      – velocity command (→ obstacle_stop_node)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


FOLLOW_DIST_M   = 1.0
FOLLOW_ARC_DEG  = 60.0
MAX_LINEAR_VEL  = 0.8
MAX_ANGULAR_VEL = 1.0
KP_LINEAR       = 0.6
KP_ANGULAR      = 1.2
CLUSTER_POINTS  = 20
MIN_DETECT_DIST = 0.3
MAX_DETECT_DIST = 4.0


class FollowMeNode(Node):

    def __init__(self):
        super().__init__('follow_me_node')

        # Current navigation mode – start as IDLE (cart does not move)
        self._current_mode = 'IDLE'

        self.create_subscription(LaserScan, '/scan',            self._lidar_cb,   10)
        self.create_subscription(String,    '/nav/current_mode', self._mode_cb,   10)

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

        self.get_logger().info(
            'Follow-Me Node started | waiting for FOLLOW mode (press remote button 2)'
        )

    def _mode_cb(self, msg: String):
        """Track the current navigation mode."""
        new_mode = msg.data.strip().upper()
        if new_mode != self._current_mode:
            self._current_mode = new_mode
            self.get_logger().info(f'Follow-Me: mode changed to {self._current_mode}')

            # When leaving FOLLOW mode, immediately stop the cart
            if self._current_mode != 'FOLLOW':
                self._cmd_pub.publish(Twist())

    def _lidar_cb(self, msg: LaserScan):
        # ── Gate: only run when in FOLLOW mode ───────────────────────────
        if self._current_mode != 'FOLLOW':
            return

        arc_rad   = math.radians(FOLLOW_ARC_DEG)
        angle_inc = msg.angle_increment

        points = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * angle_inc
            if abs(angle) > arc_rad:
                continue
            if math.isnan(r) or math.isinf(r):
                continue
            if r < MIN_DETECT_DIST or r > MAX_DETECT_DIST:
                continue
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append((x, y, r, angle))

        cmd = Twist()

        if not points:
            self._cmd_pub.publish(cmd)   # no target → stop
            self.get_logger().info(
                'No target detected – cart stopped',
                throttle_duration_sec=2.0
            )
            return

        # Use closest cluster as person centroid
        points.sort(key=lambda p: p[2])
        cluster = points[:CLUSTER_POINTS]

        cx = sum(p[0] for p in cluster) / len(cluster)
        cy = sum(p[1] for p in cluster) / len(cluster)

        target_dist  = math.sqrt(cx**2 + cy**2)
        target_angle = math.atan2(cy, cx)

        dist_error = target_dist - FOLLOW_DIST_M

        cmd.linear.x  = max(-MAX_LINEAR_VEL,
                            min(MAX_LINEAR_VEL,  KP_LINEAR  * dist_error))
        cmd.angular.z = max(-MAX_ANGULAR_VEL,
                            min(MAX_ANGULAR_VEL, KP_ANGULAR * target_angle))

        # Don't drive into the person when already at or closer than target
        if dist_error < 0:
            cmd.linear.x = 0.0

        self._cmd_pub.publish(cmd)

        self.get_logger().debug(
            f'Target: {target_dist:.2f}m  angle: {math.degrees(target_angle):.1f}°',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = FollowMeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()