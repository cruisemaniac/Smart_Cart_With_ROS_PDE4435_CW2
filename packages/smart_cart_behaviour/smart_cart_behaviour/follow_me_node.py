#!/usr/bin/env python3
"""
follow_me_node.py  –  Smart Cart Follow-Me (UWB Trilateration + LiDAR)
=======================================================================
Architecture (matches CW1 spec):

  UWB anchors (4 corners) → trilaterate person XY in cart frame
  → distance from cart CENTRE (0,0) to person
  → linear speed control (approach / hold / back off)

  LiDAR 360° → precise angle to closest object
  → angular steering to face person

  Front LiDAR safety zone → emergency stop if obstacle < 0.5m

Identity lock:
  UWB trilateration always follows THIS person's tag (unique signal).
  Falls back to LiDAR-only distance if UWB times out.

Anchor positions (match smart_cart.urdf.xacro exactly):
  A0 = front-left  ( 0.32,  0.23)
  A1 = front-right ( 0.32, -0.23)
  A2 = rear-left   (-0.32,  0.23)
  A3 = rear-right  (-0.32, -0.23)

Topics
------
  Sub: /uwb/distances     Float32MultiArray  [d0,d1,d2,d3]
  Sub: /scan              LaserScan
  Sub: /nav/current_mode  String
  Pub: /cmd_vel_raw       Twist
"""

import math
import time
import numpy as np

import rclpy
import rclpy.clock
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray


# ── Anchor positions in cart body frame (must match URDF) ────────────────────
ANCHORS = np.array([
    [ 0.32,  0.23],   # A0 front-left
    [ 0.32, -0.23],   # A1 front-right
    [-0.32,  0.23],   # A2 rear-left
    [-0.32, -0.23],   # A3 rear-right
])

# ── Distance control ──────────────────────────────────────────────────────────
FOLLOW_DIST_M    = 1.0    # target gap from cart centre to person
DEAD_ZONE_M      = 0.20   # ±0.2m comfort band → [0.8m, 1.2m]
BACKUP_SPEED     = 0.20   # m/s slow reverse when too close
EMERGENCY_STOP_M = 0.50   # LiDAR front emergency stop distance

# ── Speed limits ──────────────────────────────────────────────────────────────
MAX_LINEAR_VEL   = 0.8    # m/s
MAX_ANGULAR_VEL  = 1.2    # rad/s

# ── Controller gains ─────────────────────────────────────────────────────────
KP_LINEAR        = 0.6
KP_ANGULAR       = 1.0

# ── LiDAR detection ──────────────────────────────────────────────────────────
MIN_DETECT_DIST  = 0.15   # ignore cart body
MAX_DETECT_DIST  = 6.0
CLUSTER_POINTS   = 30
FRONT_ARC_DEG    = 30.0   # front safety arc

# ── UWB timeout ───────────────────────────────────────────────────────────────
UWB_TIMEOUT_SEC  = 1.0


class FollowMeNode(Node):

    def __init__(self):
        super().__init__('follow_me_node')

        self._mode          = 'IDLE'

        # UWB
        self._uwb_distances = None
        self._uwb_last_t    = 0.0

        # LiDAR
        self._lidar_angle   = None   # angle to closest object
        self._lidar_dist    = None   # distance to closest object
        self._lidar_front   = None   # minimum distance in front arc

        # Subscriptions
        self.create_subscription(
            String,           '/nav/current_mode', self._mode_cb,    10)
        self.create_subscription(
            Float32MultiArray, '/uwb/distances',   self._uwb_cb,     10)
        self.create_subscription(
            LaserScan,        '/scan',             self._scan_cb,    10)

        # Publisher
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

        # Control loop 20Hz – steady clock
        self.create_timer(
            0.05,
            self._control_loop,
            clock=rclpy.clock.Clock(
                clock_type=rclpy.clock.ClockType.STEADY_TIME))

        self.get_logger().info(
            'Follow-Me Node started | UWB trilateration + LiDAR angle | '
            'Press remote [2] to activate FOLLOW'
        )

    # ══════════════════════════════════════════════════════════════════════════
    # MODE CALLBACK
    # ══════════════════════════════════════════════════════════════════════════
    def _mode_cb(self, msg: String):
        new_mode = msg.data.strip().upper()
        if new_mode != self._mode:
            self._mode = new_mode
            self.get_logger().info(f'Follow-Me: mode → {self._mode}')
            if self._mode != 'FOLLOW':
                self._cmd_pub.publish(Twist())

    # ══════════════════════════════════════════════════════════════════════════
    # UWB CALLBACK
    # ══════════════════════════════════════════════════════════════════════════
    def _uwb_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self._uwb_distances = np.array(msg.data[:4])
            self._uwb_last_t    = time.monotonic()

    # ══════════════════════════════════════════════════════════════════════════
    # LIDAR CALLBACK  –  360° angle + front safety
    # ══════════════════════════════════════════════════════════════════════════
    def _scan_cb(self, msg: LaserScan):
        angle_inc    = msg.angle_increment
        front_arc    = math.radians(FRONT_ARC_DEG)
        min_front    = float('inf')
        all_points   = []

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue

            angle = msg.angle_min + i * angle_inc

            # Front safety arc
            if abs(angle) < front_arc:
                if r < min_front:
                    min_front = r

            # All valid points for angle detection
            if MIN_DETECT_DIST < r < MAX_DETECT_DIST:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                all_points.append((x, y, r, angle))

        # Front safety
        self._lidar_front = min_front if min_front != float('inf') else None

        # Closest cluster centroid for angle
        if all_points:
            all_points.sort(key=lambda p: p[2])
            cluster = all_points[:CLUSTER_POINTS]
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)
            self._lidar_dist  = math.sqrt(cx * cx + cy * cy)
            self._lidar_angle = math.atan2(cy, cx)
        else:
            self._lidar_dist  = None
            self._lidar_angle = None

    # ══════════════════════════════════════════════════════════════════════════
    # UWB TRILATERATION  –  least squares from cart centre
    # ══════════════════════════════════════════════════════════════════════════
    def _trilaterate(self, distances: np.ndarray):
        """
        Compute person XY position in cart body frame using
        least-squares trilateration from 4 anchor distances.
        Returns (x, y) or (None, None) on failure.
        Distance is measured from cart centre (0, 0).
        """
        try:
            # Build linear system using anchor 0 as reference
            A = []
            b = []
            x0, y0 = ANCHORS[0]
            d0      = distances[0]

            for i in range(1, len(ANCHORS)):
                xi, yi = ANCHORS[i]
                di      = distances[i]
                A.append([2.0 * (xi - x0), 2.0 * (yi - y0)])
                b.append(d0**2 - di**2 - x0**2 + xi**2 - y0**2 + yi**2)

            A = np.array(A, dtype=float)
            b = np.array(b, dtype=float)

            # Least squares solution
            result = np.linalg.lstsq(A, b, rcond=None)
            pos    = result[0]

            return float(pos[0]), float(pos[1])

        except Exception as e:
            self.get_logger().warn(
                f'Trilateration failed: {e}',
                throttle_duration_sec=2.0)
            return None, None

    # ══════════════════════════════════════════════════════════════════════════
    # MAIN CONTROL LOOP  –  20 Hz
    # ══════════════════════════════════════════════════════════════════════════
    def _control_loop(self):

        if self._mode != 'FOLLOW':
            return

        cmd = Twist()

        # ── UWB availability check ────────────────────────────────────────────
        uwb_fresh = (
            self._uwb_distances is not None and
            (time.monotonic() - self._uwb_last_t) < UWB_TIMEOUT_SEC
        )

        # ── Get distance from cart CENTRE ─────────────────────────────────────
        if uwb_fresh:
            # Trilaterate person XY in cart frame
            px, py = self._trilaterate(self._uwb_distances)

            if px is None:
                # Trilateration failed – use LiDAR fallback
                if self._lidar_dist is not None:
                    distance = self._lidar_dist
                    src_d    = 'LiDAR'
                else:
                    self._cmd_pub.publish(cmd)
                    return
            else:
                # Distance from cart CENTRE (0,0) to trilaterated person position
                distance = math.sqrt(px * px + py * py)
                src_d    = 'UWB'

        elif self._lidar_dist is not None:
            # UWB timed out – fall back to LiDAR
            distance = self._lidar_dist
            src_d    = 'LiDAR'
        else:
            # No data
            self._cmd_pub.publish(cmd)
            self.get_logger().warn(
                'No sensor data – stopping',
                throttle_duration_sec=2.0)
            return

        # ── Get angle (LiDAR priority for precision) ──────────────────────────
        if self._lidar_angle is not None:
            angle  = self._lidar_angle
            src_a  = 'LiDAR'
        elif uwb_fresh and px is not None:
            angle  = math.atan2(py, px)
            src_a  = 'UWB'
        else:
            angle  = 0.0
            src_a  = 'none'

        # ── Emergency stop – LiDAR front safety ──────────────────────────────
        if (self._lidar_front is not None and
                self._lidar_front < EMERGENCY_STOP_M):
            self._cmd_pub.publish(Twist())   # full stop
            self.get_logger().warn(
                f'EMERGENCY STOP – obstacle at {self._lidar_front:.2f}m',
                throttle_duration_sec=0.5)
            return

        # ── Linear control ────────────────────────────────────────────────────
        dist_error = distance - FOLLOW_DIST_M

        if dist_error > DEAD_ZONE_M:
            cmd.linear.x = min(MAX_LINEAR_VEL, KP_LINEAR * dist_error)
        elif dist_error < -DEAD_ZONE_M:
            cmd.linear.x = -BACKUP_SPEED
        else:
            cmd.linear.x = 0.0

        # ── Angular control ───────────────────────────────────────────────────
        raw_ang       = -KP_ANGULAR * angle
        cmd.angular.z = max(-MAX_ANGULAR_VEL,
                            min(MAX_ANGULAR_VEL, raw_ang))

        # ── Publish ───────────────────────────────────────────────────────────
        self._cmd_pub.publish(cmd)

        self.get_logger().info(
            f'[{src_d}/{src_a}] dist={distance:.2f}m  '
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