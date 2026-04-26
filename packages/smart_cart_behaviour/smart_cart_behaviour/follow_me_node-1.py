#!/usr/bin/env python3
"""
follow_me_node.py  –  UWB Follow-Me + LiDAR Obstacle Avoidance
===============================================================
Original UWB follow logic kept exactly as-is.
LiDAR added on top for obstacle avoidance only.

Behaviour:
  - Normally: follow person using UWB trilateration (unchanged)
  - If obstacle closer than OBSTACLE_DIST_M in front arc:
      → AVOID: turn toward clearer side, creep if centre clear
      → keep trying until path clears
      → brief RECOVER forward push, then resume normal follow
  - Only stops if UWB signal is lost (person truly gone)

Scan angle correction:
  Cart spawned at Y=π. LiDAR angle=0 faces cart REAR.
  Flip by π so corrected angle=0 = cart FRONT.

Topics
------
  Sub: /uwb/distances     Float32MultiArray [d0,d1,d2,d3]
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
from smart_cart_navigation.uwb_filter_node import Kalman2D


# ── Anchors ───────────────────────────────────────────────────────────────────
ANCHORS = np.array([
    [ 0.32,  0.23],
    [ 0.32, -0.23],
    [-0.32,  0.23],
    [-0.32, -0.23],
])

# ── UWB follow parameters (unchanged from working version) ────────────────────
FOLLOW_DIST_M   = 1.0
DEAD_ZONE_M     = 0.15
MAX_LINEAR_VEL  = 0.8
MAX_ANGULAR_VEL = 1.2
KP_LINEAR       = 0.6
KP_ANGULAR      = 0.6
ANGLE_DEADBAND  = 0.15
UWB_TIMEOUT_SEC = 1.0
KALMAN_WARMUP   = 10

# ── LiDAR obstacle parameters ─────────────────────────────────────────────────
OBSTACLE_DIST_M  = 0.40   # trigger avoidance closer than this
OBSTACLE_CLEAR_M = 0.60   # consider clear beyond this (hysteresis)
CENTRE_HALF_DEG  = 25.0   # ±25° centre danger cone
SIDE_INNER_DEG   = 25.0   # side sector starts at 25°
SIDE_OUTER_DEG   = 65.0   # side sector ends at 65° (before cart body at ~90°)
LIDAR_MIN_M      = 0.15   # ignore closer returns (noise)
AVOID_TURN_VEL   = 0.5    # rad/s during avoidance
AVOID_CREEP_VEL  = 0.10   # m/s slow creep when centre is clear
RECOVER_SEC      = 0.5    # seconds of forward push after clearing
SCAN_FLIP        = math.pi  # correct for 180° spawn


class FollowMeNode(Node):

    def __init__(self):
        super().__init__('follow_me_node')

        self._mode          = 'IDLE'
        self._avoiding      = False
        self._recovering    = False
        self._recover_start = 0.0
        self._last_py       = 0.0

        # LiDAR sector distances
        self._c_dist = float('inf')
        self._l_dist = float('inf')
        self._r_dist = float('inf')

        # UWB state
        self._uwb_distances = None
        self._uwb_last_t    = 0.0
        self._uwb_was_alive = False
        self._kf            = Kalman2D()
        self._kf_cycles     = 0

        self._avoid_phase = 0
        self._phase_start = 0.0

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

        self.get_logger().info('Follow-Me started | UWB + LiDAR avoidance')

    # ── Mode callback (unchanged) ─────────────────────────────────────────────
    def _mode_cb(self, msg: String):
        new_mode = msg.data.strip().upper()
        if new_mode != self._mode:
            self.get_logger().info(f'[MODE] {self._mode} → {new_mode}')
            self._mode = new_mode
            if self._mode != 'FOLLOW':
                self._cmd_pub.publish(Twist())
                self._avoiding   = False
                self._recovering = False
            else:
                self._kf        = Kalman2D()
                self._kf_cycles = 0
                self.get_logger().info('[UWB] Kalman reset for new FOLLOW session.')

    # ── UWB callback (unchanged) ──────────────────────────────────────────────
    def _uwb_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self._uwb_distances = np.array(msg.data[:4], dtype=float)
            self._uwb_last_t    = time.monotonic()

    # ── LiDAR callback ───────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        c_half  = math.radians(CENTRE_HALF_DEG)
        s_inner = math.radians(SIDE_INNER_DEG)
        s_outer = math.radians(SIDE_OUTER_DEG)
        c_list, l_list, r_list = [], [], []

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < LIDAR_MIN_M:
                continue
            raw   = msg.angle_min + i * msg.angle_increment
            angle = raw + SCAN_FLIP
            if angle > math.pi:
                angle -= 2.0 * math.pi
            elif angle < -math.pi:
                angle += 2.0 * math.pi
            if abs(angle) > s_outer:
                continue
            if abs(angle) <= c_half:
                c_list.append(r)
            elif s_inner < angle <= s_outer:
                l_list.append(r)
            elif -s_outer <= angle < -s_inner:
                r_list.append(r)

        self._c_dist = min(c_list) if c_list else float('inf')
        self._l_dist = min(l_list) if l_list else float('inf')
        self._r_dist = min(r_list) if r_list else float('inf')

    # ── Trilateration (unchanged) ─────────────────────────────────────────────
    def _trilaterate_raw(self, d):
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

    # ── Pick avoidance direction ──────────────────────────────────────────────
    # def _pick_dir(self) -> float:
    #     obs_l = self._l_dist < OBSTACLE_DIST_M
    #     obs_r = self._r_dist < OBSTACLE_DIST_M
    #     if obs_l and not obs_r:
    #         return -1.0   # left blocked → turn right
    #     if obs_r and not obs_l:
    #         return 1.0    # right blocked → turn left
    #     # Both or neither → more open side
    #     return 1.0 if self._l_dist >= self._r_dist else -1.0

    # ── Main control loop ─────────────────────────────────────────────────────
    def _control_loop(self):
        if self._mode != 'FOLLOW':
            return

        now = time.monotonic()

        # UWB freshness (unchanged)
        uwb_fresh = (
            self._uwb_distances is not None and
            (now - self._uwb_last_t) < UWB_TIMEOUT_SEC
        )
        if uwb_fresh and not self._uwb_was_alive:
            self.get_logger().info('[UWB] Signal acquired.')
            self._uwb_was_alive = True
        elif not uwb_fresh and self._uwb_was_alive:
            self.get_logger().warn('[UWB] Signal lost – stopping.')
            self._uwb_was_alive = False

        if not uwb_fresh:
            self._cmd_pub.publish(Twist())
            self._avoiding = self._recovering = False
            return

        # Trilaterate (unchanged)
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

        self._last_py = py
        distance = math.sqrt(px**2 + py**2)
        angle    = math.atan2(py, px)

        # Obstacle flags
        obs_front = self._c_dist < OBSTACLE_DIST_M
        path_clear = (self._c_dist > OBSTACLE_CLEAR_M and
                      self._l_dist > OBSTACLE_CLEAR_M and
                      self._r_dist > OBSTACLE_CLEAR_M)

        # State transitions
        if not self._avoiding and not self._recovering and obs_front:
            self._avoiding = True
            self._avoid_phase = 0
            self._phase_start = now
            self.get_logger().warn(
                f'[AVOID] Front obstacle! C={self._c_dist:.2f}m')

        elif self._recovering:
            if now - self._recover_start > RECOVER_SEC:
                self._recovering = False
                self.get_logger().info('[FOLLOW] Resumed.')
            # elif obs_any:
            elif obs_front:
                self._recovering = False
                self._avoiding   = True
                self._avoid_phase = 0
                self._phase_start = now

        # Command output
        cmd = Twist()

        if self._avoiding:
            
            # Phase 0: TURN RIGHT
            if self._avoid_phase == 0:
                cmd.linear.x = 0.0
                cmd.angular.z = -AVOID_TURN_VEL

                if now - self._phase_start > 1.0:  # tune this
                    self._avoid_phase = 1
                    self._phase_start = now

            # Phase 1: GO FORWARD
            elif self._avoid_phase == 1:
                cmd.linear.x = AVOID_CREEP_VEL
                cmd.angular.z = 0.0

                if now - self._phase_start > 1.5:
                    self._avoid_phase = 2
                    self._phase_start = now

            # Phase 2: TURN LEFT
            elif self._avoid_phase == 2:
                cmd.linear.x = 0.0
                cmd.angular.z = AVOID_TURN_VEL

                if now - self._phase_start > 1.0:
                    self._avoiding = False
                    self._avoid_phase = 0
                    self.get_logger().info('[AVOID] Done.')



        elif self._recovering:
            cmd.linear.x  = AVOID_CREEP_VEL * 2.0
            cmd.angular.z = max(-MAX_ANGULAR_VEL,
                                min(MAX_ANGULAR_VEL, KP_ANGULAR * angle))

        else:
            # ── Original UWB follow logic — completely unchanged ───────────
            error = distance - FOLLOW_DIST_M
            if error > DEAD_ZONE_M:
                cmd.linear.x = min(MAX_LINEAR_VEL, KP_LINEAR * error)
            else:
                cmd.linear.x = 0.0

            if abs(angle) < ANGLE_DEADBAND:
                cmd.angular.z = 0.0
            else:
                cmd.angular.z = max(-MAX_ANGULAR_VEL,
                                    min(MAX_ANGULAR_VEL, KP_ANGULAR * angle))

            anchor_names = ['A0_FL', 'A1_FR', 'A2_RL', 'A3_RR']
            anchor_str   = '  '.join(
                f'{n}:{v:.2f}' for n, v in
                zip(anchor_names, self._uwb_distances))
            self.get_logger().info(
                f'[UWB] raw=({px_raw:.2f},{py_raw:.2f})  '
                f'smooth=({px:.2f},{py:.2f})  '
                f'dist={distance:.2f}m  '
                f'angle={math.degrees(angle):.1f}°  '
                f'vx={cmd.linear.x:.2f}  wz={cmd.angular.z:.2f}\n'
                f'      [{anchor_str}]',
                throttle_duration_sec=0.5)

        self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FollowMeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()