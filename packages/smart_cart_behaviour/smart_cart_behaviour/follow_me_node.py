#!/usr/bin/env python3
"""
follow_me_node.py  –  Smart Cart Follow-Me
==========================================
Architecture (matches CW1 spec):

  UWB  →  distance to person  →  linear speed control
  LiDAR → angle to person     →  angular steering control

  UWB gives reliable identity-locked distance measurement.
  LiDAR gives precise angle for steering.

  If UWB is unavailable (no /uwb/distance yet), falls back
  to LiDAR-only mode (360° closest cluster).

Control zones:
  dist > 1.2m   →  move forward
  dist < 0.8m   →  back off slowly
  0.8m – 1.2m   →  dead zone, just align
  always        →  angular correction toward person

Gate:
  Only active when /nav/current_mode = "FOLLOW"

Topics
------
  Sub: /scan             LaserScan   – precise angle to person
  Sub: /uwb/distance     Float32     – UWB distance measurement
  Sub: /uwb/angle        Float32     – UWB angle measurement
  Sub: /nav/current_mode String      – mode gate
  Pub: /cmd_vel_raw      Twist       – velocity command
"""

import math
import rclpy
import rclpy.clock
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32


# ── Distance control (from UWB) ───────────────────────────────────────────────
FOLLOW_DIST_M   = 1.0    # target gap between cart and person
DEAD_ZONE_M     = 0.20   # ±0.2m tolerance = [0.8m, 1.2m] comfort zone
BACKUP_SPEED    = 0.20   # m/s slow reverse when too close

# ── Speed limits ──────────────────────────────────────────────────────────────
MAX_LINEAR_VEL  = 0.8    # m/s – CW1 software cap near humans
MAX_ANGULAR_VEL = 1.2    # rad/s

# ── Controller gains ─────────────────────────────────────────────────────────
KP_LINEAR       = 0.5    # proportional gain for distance error
KP_ANGULAR      = 0.9    # proportional gain for angle error

# ── LiDAR detection params (fallback + angle refinement) ─────────────────────
MIN_DETECT_DIST = 0.15   # ignore cart's own reflections
MAX_DETECT_DIST = 6.0    # full supermarket range
CLUSTER_POINTS  = 30     # points used for centroid

# ── UWB timeout (fall back to LiDAR-only if UWB silent) ──────────────────────
UWB_TIMEOUT_SEC = 1.0


class FollowMeNode(Node):

    def __init__(self):
        super().__init__('follow_me_node')

        self._current_mode  = 'IDLE'

        # UWB data
        self._uwb_distance  = None
        self._uwb_angle     = None
        self._uwb_last_sec  = 0.0
        self._uwb_available = False

        # LiDAR data
        self._lidar_angle   = None
        self._lidar_dist    = None

        # Subscriptions
        self.create_subscription(
            String,    '/nav/current_mode', self._mode_cb,    10)
        self.create_subscription(
            Float32,   '/uwb/distance',     self._uwb_dist_cb, 10)
        self.create_subscription(
            Float32,   '/uwb/angle',        self._uwb_angle_cb, 10)
        self.create_subscription(
            LaserScan, '/scan',             self._scan_cb,    10)

        # Publisher
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

        # Control loop timer – 20 Hz, steady clock
        self.create_timer(
            0.05,
            self._control_loop,
            clock=rclpy.clock.Clock(
                clock_type=rclpy.clock.ClockType.STEADY_TIME))

        self.get_logger().info(
            'Follow-Me Node started | UWB distance + LiDAR angle | '
            'Press remote [2] to activate FOLLOW'
        )

    # ══════════════════════════════════════════════════════════════════════════
    # MODE CALLBACK
    # ══════════════════════════════════════════════════════════════════════════
    def _mode_cb(self, msg: String):
        new_mode = msg.data.strip().upper()
        if new_mode != self._current_mode:
            self._current_mode = new_mode
            self.get_logger().info(f'Follow-Me mode → {self._current_mode}')
            if self._current_mode != 'FOLLOW':
                self._cmd_pub.publish(Twist())

    # ══════════════════════════════════════════════════════════════════════════
    # UWB CALLBACKS
    # ══════════════════════════════════════════════════════════════════════════
    def _uwb_dist_cb(self, msg: Float32):
        self._uwb_distance = msg.data
        import time
        self._uwb_last_sec  = time.monotonic()
        self._uwb_available = True

    def _uwb_angle_cb(self, msg: Float32):
        self._uwb_angle = msg.data

    # ══════════════════════════════════════════════════════════════════════════
    # LIDAR CALLBACK  –  360° closest cluster for angle refinement
    # ══════════════════════════════════════════════════════════════════════════
    def _scan_cb(self, msg: LaserScan):
        angle_inc = msg.angle_increment
        points = []

        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if r < MIN_DETECT_DIST or r > MAX_DETECT_DIST:
                continue
            angle = msg.angle_min + i * angle_inc
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append((x, y, r, angle))

        if not points:
            self._lidar_angle = None
            self._lidar_dist  = None
            return

        # Closest cluster centroid
        points.sort(key=lambda p: p[2])
        cluster = points[:CLUSTER_POINTS]

        cx = sum(p[0] for p in cluster) / len(cluster)
        cy = sum(p[1] for p in cluster) / len(cluster)

        self._lidar_dist  = math.sqrt(cx * cx + cy * cy)
        self._lidar_angle = math.atan2(cy, cx)

    # ══════════════════════════════════════════════════════════════════════════
    # MAIN CONTROL LOOP  –  20 Hz
    # ══════════════════════════════════════════════════════════════════════════
    def _control_loop(self):

        # Gate – only run in FOLLOW mode
        if self._current_mode != 'FOLLOW':
            return

        cmd = Twist()

        # ── Check UWB availability ────────────────────────────────────────────
        import time
        uwb_fresh = (
            self._uwb_available and
            (time.monotonic() - self._uwb_last_sec) < UWB_TIMEOUT_SEC
        )

        # ── Get distance ──────────────────────────────────────────────────────
        # Priority: UWB (identity-locked) → LiDAR fallback
        if uwb_fresh and self._uwb_distance is not None:
            distance = self._uwb_distance
            source   = 'UWB'
        elif self._lidar_dist is not None:
            distance = self._lidar_dist
            source   = 'LiDAR'
        else:
            # No data at all – stop
            self._cmd_pub.publish(cmd)
            self.get_logger().warn(
                'No sensor data – cart stopped',
                throttle_duration_sec=2.0)
            return

        # ── Get angle ─────────────────────────────────────────────────────────
        # Priority: LiDAR (more precise) → UWB fallback
        if self._lidar_angle is not None:
            angle  = self._lidar_angle
            a_src  = 'LiDAR'
        elif uwb_fresh and self._uwb_angle is not None:
            angle  = self._uwb_angle
            a_src  = 'UWB'
        else:
            angle  = 0.0
            a_src  = 'none'

        # ── Linear control (UWB distance) ────────────────────────────────────
        dist_error = distance - FOLLOW_DIST_M

        if dist_error > DEAD_ZONE_M:
            # Too far – approach
            cmd.linear.x = min(MAX_LINEAR_VEL, KP_LINEAR * dist_error)

        elif dist_error < -DEAD_ZONE_M:
            # Too close – back off
            cmd.linear.x = -BACKUP_SPEED

        else:
            # In dead zone – stop linear
            cmd.linear.x = 0.0

        # ── Angular control (LiDAR angle) ────────────────────────────────────
        raw_angular   = -KP_ANGULAR * angle
        cmd.angular.z = max(-MAX_ANGULAR_VEL,
                            min(MAX_ANGULAR_VEL, raw_angular))

        # ── Publish ───────────────────────────────────────────────────────────
        self._cmd_pub.publish(cmd)

        self.get_logger().info(
            f'[{source}/{a_src}] dist={distance:.2f}m  '
            f'angle={math.degrees(angle):.1f}°  '
            f'lin={cmd.linear.x:.2f}  ang={cmd.angular.z:.2f}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = FollowMeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
