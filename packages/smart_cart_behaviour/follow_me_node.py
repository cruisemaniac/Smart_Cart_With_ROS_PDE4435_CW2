#!/usr/bin/env python3
"""
Follow-Me Node – Smart Cart
Uses the 360° LiDAR to detect the nearest object in the front arc
and drives the cart to maintain a 1.0 m following distance.

Matches CW1 Section 4.4 – Path Planning and Follow-Me Controller.
Software speed cap: 0.8 m/s near humans (CW1 Table 2).

Subscribes to : /scan   (sensor_msgs/LaserScan)
Publishes to  : /cmd_vel_raw  (geometry_msgs/Twist)
                → goes through obstacle_stop_node → /cmd_vel
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# ── Control parameters ───────────────────────────────────────────────────────
FOLLOW_DIST_M     = 1.0    # desired gap to person (metres)
FOLLOW_ARC_DEG    = 60.0   # scan arc to look for person (±60°)
MAX_LINEAR_VEL    = 0.8    # m/s – CW1 software cap near humans
MAX_ANGULAR_VEL   = 1.0    # rad/s
KP_LINEAR         = 0.6    # proportional gain – distance error
KP_ANGULAR        = 1.2    # proportional gain – angle error
CLUSTER_POINTS    = 20     # number of closest scan points to use for centroid
MIN_DETECT_DIST   = 0.3    # ignore anything closer (part of robot)
MAX_DETECT_DIST   = 4.0    # ignore background beyond this


class FollowMeNode(Node):

    def __init__(self):
        super().__init__('follow_me_node')
        self.get_logger().info('Follow-Me Node started')

        self._lidar_sub = self.create_subscription(
            LaserScan, 'scan', self._lidar_cb, 10)
        self._cmd_pub = self.create_publisher(
            Twist, 'cmd_vel_raw', 10)

    def _lidar_cb(self, msg: LaserScan):
        arc_rad   = math.radians(FOLLOW_ARC_DEG)
        angle_inc = msg.angle_increment

        # ── Collect valid points in front arc ──────────────────────────────
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
            # No target detected – publish zero to stop cart
            self._cmd_pub.publish(cmd)
            self.get_logger().info(
                'No target detected – cart stopped',
                throttle_duration_sec=2.0)
            return

        # ── Use CLUSTER_POINTS closest points as person centroid ───────────
        points.sort(key=lambda p: p[2])
        cluster = points[:CLUSTER_POINTS]

        cx = sum(p[0] for p in cluster) / len(cluster)
        cy = sum(p[1] for p in cluster) / len(cluster)

        target_dist  = math.sqrt(cx**2 + cy**2)
        target_angle = math.atan2(cy, cx)

        # ── P-controller ────────────────────────────────────────────────────
        dist_error  = target_dist - FOLLOW_DIST_M

        raw_linear  = KP_LINEAR  * dist_error
        raw_angular = KP_ANGULAR * target_angle

        # Clamp to max velocities
        cmd.linear.x  = max(-MAX_LINEAR_VEL,
                            min(MAX_LINEAR_VEL, raw_linear))
        cmd.angular.z = max(-MAX_ANGULAR_VEL,
                            min(MAX_ANGULAR_VEL, raw_angular))

        # Do not drive forward into the person when already close
        if dist_error < 0:
            cmd.linear.x = 0.0

        self._cmd_pub.publish(cmd)

        self.get_logger().debug(
            f'Target: dist={target_dist:.2f} m  '
            f'angle={math.degrees(target_angle):.1f}°  '
            f'speed_factor={cmd.linear.x:.2f}',
            throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = FollowMeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()