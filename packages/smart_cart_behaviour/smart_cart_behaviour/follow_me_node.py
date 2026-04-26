#!/usr/bin/env python3
"""
Uses:
  /uwb/distances  → trilateration → distance & angle to person
  /scan           → LiDAR → obstacle avoidance steering

Topics
------
  Sub: /uwb/distances     Float32MultiArray [d0,d1,d2,d3]
  Sub: /nav/current_mode  String
  Sub: /scan              LaserScan
  Pub: /cmd_vel_raw       Twist

Obstacle avoidance strategy
---------------------------
  Front sector  ±40°   : danger zone — triggers avoidance
  Side sectors  40–80° : clearance check — pick the more open side
  Proximity blend       : at OBSTACLE_SLOW_DIST avoidance starts fading in;
                          at OBSTACLE_STOP_DIST  forward motion halted entirely
                          (prevents wheel-slip / odometry corruption in RViz)
  Once obstacle clears  : full UWB following resumes automatically
"""

import math
import time
import numpy as np

import rclpy
import rclpy.clock
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32MultiArray
from smart_cart_navigation.uwb_filter_node import Kalman2D


# ── UWB anchor positions on the cart (metres) ─────────────────────────────
ANCHORS = np.array([
    [ 0.32,  0.23],   # A0 front-left
    [ 0.32, -0.23],   # A1 front-right
    [-0.32,  0.23],   # A2 rear-left
    [-0.32, -0.23],   # A3 rear-right
])

# ── UWB following tuning ───────────────────────────────────────────────────
FOLLOW_DIST_M   = 1.0
DEAD_ZONE_M     = 0.15
MAX_LINEAR_VEL  = 0.8
MAX_ANGULAR_VEL = 1.2
KP_LINEAR       = 0.6
KP_ANGULAR      = 0.6
ANGLE_DEADBAND  = 0.15
UWB_TIMEOUT_SEC = 1.0
KALMAN_WARMUP   = 10

# ── LiDAR obstacle avoidance tuning ───────────────────────────────────────
FRONT_HALF_DEG     = 40    # ±degrees that count as "straight ahead"
SIDE_HALF_DEG      = 80    # outer edge of side clearance check
OBSTACLE_SLOW_DIST = 1.00  # m — avoidance starts blending in
OBSTACLE_STOP_DIST = 0.50  # m — forward motion stops (no wheel slip)
AVOID_ANG_GAIN     = 1.8   # avoidance angular velocity scale


class FollowMeNode(Node):

    def __init__(self):
        super().__init__('follow_me_node')

        self._mode          = 'IDLE'
        self._uwb_distances = None
        self._uwb_last_t    = 0.0
        self._uwb_was_alive = False
        self._kf            = Kalman2D()
        self._kf_cycles     = 0
        self._scan          = None   # latest LaserScan

        self.create_subscription(
            String,            '/nav/current_mode', self._mode_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/uwb/distances',    self._uwb_cb,  10)
        self.create_subscription(
            LaserScan,         '/scan',             self._scan_cb, 10)

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

        self.create_timer(
            0.05,
            self._control_loop,
            clock=rclpy.clock.Clock(
                clock_type=rclpy.clock.ClockType.STEADY_TIME))

        self.get_logger().info('Follow-Me (UWB + LiDAR avoidance) started.')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _mode_cb(self, msg: String):
        new_mode = msg.data.strip().upper()
        if new_mode != self._mode:
            self.get_logger().info(f'[MODE] {self._mode} → {new_mode}')
            self._mode = new_mode
            if self._mode != 'FOLLOW':
                self._cmd_pub.publish(Twist())
            else:
                self._kf        = Kalman2D()
                self._kf_cycles = 0
                self.get_logger().info('[UWB] Kalman reset for new FOLLOW session.')

    def _uwb_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self._uwb_distances = np.array(msg.data[:4], dtype=float)
            self._uwb_last_t    = time.monotonic()

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg

    # ── LiDAR helpers ─────────────────────────────────────────────────────

    def _sector_min(self, angle_min_deg: float, angle_max_deg: float) -> float:
        """Minimum valid range within [angle_min_deg, angle_max_deg]."""
        if self._scan is None:
            return float('inf')

        scan   = self._scan
        ranges = np.asarray(scan.ranges, dtype=float)
        n      = len(ranges)
        if n == 0:
            return float('inf')

        angles = np.linspace(scan.angle_min, scan.angle_max, n)
        a_min  = math.radians(angle_min_deg)
        a_max  = math.radians(angle_max_deg)

        mask = (
            (angles >= a_min) & (angles <= a_max) &
            np.isfinite(ranges) &
            (ranges >= scan.range_min) & (ranges <= scan.range_max)
        )

        return float(np.min(ranges[mask])) if np.any(mask) else float('inf')

    def _obstacle_sectors(self):
        """Return (min_front, min_left, min_right) in metres."""
        f = FRONT_HALF_DEG
        s = SIDE_HALF_DEG
        return (
            self._sector_min(-f,  f),   # front
            self._sector_min( f,  s),   # left side
            self._sector_min(-s, -f),   # right side
        )

    # ── UWB helpers ───────────────────────────────────────────────────────

    def _trilaterate_raw(self, d: np.ndarray):
        """Least-squares trilateration. Returns (px, py) or (None, None)."""
        try:
            x0, y0 = ANCHORS[0]
            d0      = d[0]
            A, b    = [], []
            for i in range(1, 4):
                xi, yi = ANCHORS[i]
                di      = d[i]
                A.append([2.0 * (xi - x0), 2.0 * (yi - y0)])
                b.append(d0**2 - di**2 - x0**2 + xi**2 - y0**2 + yi**2)

            sol = np.linalg.lstsq(
                np.array(A, dtype=float),
                np.array(b, dtype=float),
                rcond=None)[0]

            px, py = float(sol[0]), float(sol[1])
            if not (np.isfinite(px) and np.isfinite(py)):
                return None, None
            return px, py

        except Exception as e:
            self.get_logger().warn(
                f'[UWB] Trilateration failed: {e}',
                throttle_duration_sec=2.0)
            return None, None

    # ── Control loop ──────────────────────────────────────────────────────

    def _control_loop(self):
        if self._mode != 'FOLLOW':
            return

        uwb_fresh = (
            self._uwb_distances is not None and
            (time.monotonic() - self._uwb_last_t) < UWB_TIMEOUT_SEC
        )

        if uwb_fresh and not self._uwb_was_alive:
            self.get_logger().info('[UWB] Signal acquired.')
            self._uwb_was_alive = True
        elif not uwb_fresh and self._uwb_was_alive:
            self.get_logger().warn('[UWB] Signal lost – stopping.')
            self._uwb_was_alive = False

        if not uwb_fresh:
            self._cmd_pub.publish(Twist())
            return

        # ── Trilaterate & smooth ───────────────────────────────────────────
        px_raw, py_raw = self._trilaterate_raw(self._uwb_distances)
        if px_raw is None:
            self._cmd_pub.publish(Twist())
            return

        smoothed        = self._kf.update([px_raw, py_raw])
        self._kf_cycles += 1

        if self._kf_cycles < KALMAN_WARMUP:
            px, py = px_raw, py_raw
            self.get_logger().info(
                f'[UWB] Warming up Kalman ({self._kf_cycles}/{KALMAN_WARMUP}) '
                f'raw=({px_raw:.2f},{py_raw:.2f})',
                throttle_duration_sec=0.5)
        else:
            px, py = float(smoothed[0]), float(smoothed[1])

        # ── UWB velocity commands ──────────────────────────────────────────
        distance = math.sqrt(px**2 + py**2)
        angle    = math.atan2(py, px)

        error = distance - FOLLOW_DIST_M

        if error > DEAD_ZONE_M:
            uwb_linear = min(MAX_LINEAR_VEL, KP_LINEAR * error)
        else:
            uwb_linear = 0.0

        if abs(angle) < ANGLE_DEADBAND:
            uwb_angular = 0.0
        else:
            uwb_angular = max(-MAX_ANGULAR_VEL,
                              min(MAX_ANGULAR_VEL, KP_ANGULAR * angle))

        # ── LiDAR obstacle avoidance ───────────────────────────────────────
        min_front, min_left, min_right = self._obstacle_sectors()

        if min_front < OBSTACLE_SLOW_DIST:
            # proximity: 0.0 at OBSTACLE_SLOW_DIST, 1.0 at OBSTACLE_STOP_DIST
            proximity = 1.0 - max(0.0, min(1.0,
                (min_front - OBSTACLE_STOP_DIST) /
                (OBSTACLE_SLOW_DIST - OBSTACLE_STOP_DIST)
            ))

            # Forward motion: halt completely once inside stop distance to
            # prevent wheel slip against the obstacle (keeps odometry clean)
            if min_front <= OBSTACLE_STOP_DIST:
                final_linear = 0.0
            else:
                final_linear = uwb_linear * (1.0 - proximity)

            # Steer toward the side with more clearance
            steer_sign    = +1.0 if min_left >= min_right else -1.0
            avoid_angular = steer_sign * AVOID_ANG_GAIN * proximity

            # Blend: UWB steering fades out as proximity rises so the cart
            # steers back toward the person once it has cleared the obstacle
            final_angular = (1.0 - proximity) * uwb_angular + proximity * avoid_angular

            self.get_logger().warn(
                f'[LIDAR] Obstacle  front={min_front:.2f}m  '
                f'L={min_left:.2f}m  R={min_right:.2f}m  '
                f'steer={"LEFT" if steer_sign > 0 else "RIGHT"}  '
                f'prox={proximity:.2f}',
                throttle_duration_sec=0.3)
        else:
            final_linear  = uwb_linear
            final_angular = uwb_angular

        # ── Clamp & publish ────────────────────────────────────────────────
        cmd           = Twist()
        cmd.linear.x  = max(0.0, min(MAX_LINEAR_VEL,  final_linear))
        cmd.angular.z = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, final_angular))
        self._cmd_pub.publish(cmd)

        anchor_names = ['A0_FL', 'A1_FR', 'A2_RL', 'A3_RR']
        anchor_str   = '  '.join(
            f'{n}:{v:.2f}' for n, v in zip(anchor_names, self._uwb_distances))
        self.get_logger().info(
            f'[UWB] raw=({px_raw:.2f},{py_raw:.2f})  '
            f'smooth=({px:.2f},{py:.2f})  '
            f'dist={distance:.2f}m  '
            f'angle={math.degrees(angle):.1f}°  '
            f'vx={cmd.linear.x:.2f}  wz={cmd.angular.z:.2f}\n'
            f'      [{anchor_str}]',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = FollowMeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
