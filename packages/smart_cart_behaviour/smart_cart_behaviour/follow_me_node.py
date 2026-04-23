#!/usr/bin/env python3
"""
Uses:
  /uwb/distances  → trilateration → distance from cart centre
  /uwb/angle      → direct angle from simulator (more accurate lateral)

Topics
------
  Sub: /uwb/distances     Float32MultiArray [d0,d1,d2,d3]
  Sub: /uwb/angle         Float32
  Sub: /nav/current_mode  String
  Pub: /cmd_vel_raw       Twist

"""

import math
import time
import numpy as np

import rclpy
import rclpy.clock
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray
from smart_cart_navigation.uwb_filter_node import Kalman2D


ANCHORS = np.array([
    [ 0.32,  0.23],   # A0 front-left
    [ 0.32, -0.23],   # A1 front-right
    [-0.32,  0.23],   # A2 rear-left
    [-0.32, -0.23],   # A3 rear-right
])

FOLLOW_DIST_M   = 1.0
DEAD_ZONE_M     = 0.15
MAX_LINEAR_VEL  = 0.8
MAX_ANGULAR_VEL = 1.2
KP_LINEAR       = 0.6
KP_ANGULAR      = 0.6
ANGLE_DEADBAND  = 0.15  
UWB_TIMEOUT_SEC = 1.0
KALMAN_WARMUP   = 10     # number of cycles before trusting Kalman output


class FollowMeNode(Node):

    def __init__(self):
        super().__init__('follow_me_node')

        self._mode          = 'IDLE'
        self._uwb_distances = None
        self._uwb_last_t    = 0.0
        self._uwb_was_alive = False
        self._kf            = Kalman2D()
        self._kf_cycles     = 0      # count updates since start/reset

        self.create_subscription(
            String,            '/nav/current_mode', self._mode_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/uwb/distances',    self._uwb_cb,  10)

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

        self.create_timer(
            0.05,
            self._control_loop,
            clock=rclpy.clock.Clock(
                clock_type=rclpy.clock.ClockType.STEADY_TIME))

        self.get_logger().info('Pure UWB Follow-Me started.')

    def _mode_cb(self, msg: String):
        new_mode = msg.data.strip().upper()
        if new_mode != self._mode:
            self.get_logger().info(f'[MODE] {self._mode} → {new_mode}')
            self._mode = new_mode
            if self._mode != 'FOLLOW':
                self._cmd_pub.publish(Twist())
            else:
                # Reset Kalman when re-entering FOLLOW so cold start
                # doesn't use stale state from last session
                self._kf       = Kalman2D()
                self._kf_cycles = 0
                self.get_logger().info('[UWB] Kalman reset for new FOLLOW session.')

    def _uwb_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self._uwb_distances = np.array(msg.data[:4], dtype=float)
            self._uwb_last_t    = time.monotonic()

    def _trilaterate_raw(self, d: np.ndarray):
        """
        Pure least-squares trilateration, no Kalman.
        Returns (px, py) or (None, None).
        """
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

        # ── Trilaterate raw position ───────────────────────────────────────
        px_raw, py_raw = self._trilaterate_raw(self._uwb_distances)
        if px_raw is None:
            self._cmd_pub.publish(Twist())
            return

        # ── Kalman smooth — but use raw for first KALMAN_WARMUP cycles ─────
        smoothed       = self._kf.update([px_raw, py_raw])
        self._kf_cycles += 1

        if self._kf_cycles < KALMAN_WARMUP:
            # Use raw trilateration during warmup — Kalman is still converging
            px, py = px_raw, py_raw
            self.get_logger().info(
                f'[UWB] Warming up Kalman ({self._kf_cycles}/{KALMAN_WARMUP}) '
                f'raw=({px_raw:.2f},{py_raw:.2f})',
                throttle_duration_sec=0.5)
        else:
            px, py = float(smoothed[0]), float(smoothed[1])

        # ── Distance and angle ─────────────────────────────────────────────
        distance = math.sqrt(px**2 + py**2)
        angle    = math.atan2(py, px)

        # ── Linear: approach only, stop when at target, never reverse ──────
        error = distance - FOLLOW_DIST_M

        if error > DEAD_ZONE_M:
            linear_x = min(MAX_LINEAR_VEL, KP_LINEAR * error)
        else:
            linear_x = 0.0

        # ── Angular: turn to face person ───────────────────────────────────
        if abs(angle) < ANGLE_DEADBAND:
            angular_z = 0.0
        else:
            angular_z = max(-MAX_ANGULAR_VEL,
                            min(MAX_ANGULAR_VEL, KP_ANGULAR * angle))

        cmd           = Twist()
        cmd.linear.x  = linear_x
        cmd.angular.z = angular_z
        self._cmd_pub.publish(cmd)

        anchor_names = ['A0_FL', 'A1_FR', 'A2_RL', 'A3_RR']
        anchor_str   = '  '.join(
            f'{n}:{v:.2f}' for n, v in zip(anchor_names, self._uwb_distances))
        self.get_logger().info(
            f'[UWB] raw=({px_raw:.2f},{py_raw:.2f})  '
            f'smooth=({px:.2f},{py:.2f})  '
            f'dist={distance:.2f}m  '
            f'angle={math.degrees(angle):.1f}°  '
            f'vx={linear_x:.2f}  wz={angular_z:.2f}\n'
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