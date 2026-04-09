#!/usr/bin/env python3
"""
navigation_node.py  –  Smart Cart Navigation State Machine
===========================================================
Manages operating mode and coordinates follow-me behaviour.

Modes
-----
  IDLE    Cart stationary. Waiting for remote button 2.
  FOLLOW  Follow-Me active. Tracks person via LiDAR.
  STOP    Emergency / remote button 1. Cart locked until button 2.
  MANUAL  Reserved for future teleop override.

Remote button mapping (from teleop_person_node.py)
---------------------------------------------------
  Button 1  →  STOP
  Button 2  →  FOLLOW
  Button 3  →  IDLE

Topics
------
  Sub: /scan              LaserScan   – detect person presence
  Sub: /odom              Odometry    – track distance travelled
  Sub: /nav/mode_cmd      String      – mode change commands
  Pub: /nav/current_mode  String      – current state (latched)
  Pub: /nav/status        String      – human-readable status
  Pub: /cmd_vel_raw       Twist       – zero velocity in STOP/IDLE

IMPORTANT: Cart starts in IDLE. Press remote button 2 to follow.
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String


MODE_IDLE   = 'IDLE'
MODE_FOLLOW = 'FOLLOW'
MODE_STOP   = 'STOP'
MODE_MANUAL = 'MANUAL'
VALID_MODES = {MODE_IDLE, MODE_FOLLOW, MODE_STOP, MODE_MANUAL}

PERSON_MIN_M   = 0.3
PERSON_MAX_M   = 4.0
PERSON_ARC_DEG = 60.0


class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('mode',            'idle')   # DEFAULT = idle
        self.declare_parameter('follow_distance', 1.0)
        self.declare_parameter('max_speed',       0.8)
        self.declare_parameter('target_timeout',  5.0)

        # Read startup mode – force uppercase, validate, fallback to IDLE
        raw_mode = self.get_parameter('mode').value.upper()
        self._mode = raw_mode if raw_mode in VALID_MODES else MODE_IDLE

        self._follow_d = self.get_parameter('follow_distance').value
        self._max_spd  = self.get_parameter('max_speed').value
        self._timeout  = self.get_parameter('target_timeout').value

        # ── State ────────────────────────────────────────────────────────
        self._target_visible = False
        self._last_target_t  = time.time()
        self._cart_x         = 0.0
        self._cart_y         = 0.0
        self._total_dist     = 0.0
        self._prev_x         = None
        self._prev_y         = None

        # ── Subscriptions ────────────────────────────────────────────────
        self.create_subscription(LaserScan, '/scan',         self._scan_cb,     10)
        self.create_subscription(Odometry,  '/odom',         self._odom_cb,     10)
        self.create_subscription(String,    '/nav/mode_cmd', self._mode_cmd_cb, 10)

        # ── Publishers ───────────────────────────────────────────────────
        self._mode_pub   = self.create_publisher(String, '/nav/current_mode', 10)
        self._status_pub = self.create_publisher(String, '/nav/status',       10)
        self._cmd_pub    = self.create_publisher(Twist,  '/cmd_vel_raw',      10)

        # ── Immediately publish zero velocity on startup ──────────────────
        # This ensures the cart does NOT move until commanded
        self._cmd_pub.publish(Twist())

        # ── Timers ───────────────────────────────────────────────────────
        self.create_timer(0.5, self._status_timer_cb)
        self.create_timer(1.0, self._watchdog_cb)

        self.get_logger().info(
            f'Navigation Node started | startup mode={self._mode} | '
            f'Press remote button 2 to start following'
        )

    # ════════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ════════════════════════════════════════════════════════════════════

    def _scan_cb(self, msg: LaserScan):
        arc_rad   = math.radians(PERSON_ARC_DEG)
        angle_inc = msg.angle_increment
        found = False
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * angle_inc
            if abs(angle) > arc_rad:
                continue
            if math.isnan(r) or math.isinf(r):
                continue
            if PERSON_MIN_M < r < PERSON_MAX_M:
                found = True
                break
        self._target_visible = found
        if found:
            self._last_target_t = time.time()

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._cart_x = x
        self._cart_y = y
        if self._prev_x is not None:
            dx = x - self._prev_x
            dy = y - self._prev_y
            self._total_dist += math.sqrt(dx*dx + dy*dy)
        self._prev_x = x
        self._prev_y = y

    def _mode_cmd_cb(self, msg: String):
        """
        Called when remote button is pressed (via teleop_person_node).
        Button 1 → STOP, Button 2 → FOLLOW, Button 3 → IDLE
        """
        requested = msg.data.strip().upper()
        if requested not in VALID_MODES:
            self.get_logger().warn(f'Unknown mode: "{msg.data}"')
            return

        old = self._mode
        self._set_mode(requested)

        self.get_logger().info(
            f'[REMOTE] Mode: {old} → {self._mode}'
        )

    # ════════════════════════════════════════════════════════════════════
    # TIMERS
    # ════════════════════════════════════════════════════════════════════

    def _status_timer_cb(self):
        # Publish current mode
        mode_msg = String()
        mode_msg.data = self._mode
        self._mode_pub.publish(mode_msg)

        # Publish status string
        tgt = 'visible' if self._target_visible else 'not visible'
        status = String()
        status.data = (
            f'mode={self._mode} | target={tgt} | '
            f'pos=({self._cart_x:.2f},{self._cart_y:.2f}) | '
            f'dist={self._total_dist:.2f}m'
        )
        self._status_pub.publish(status)

        # In STOP or IDLE: keep publishing zero so cart cannot drift
        if self._mode in (MODE_STOP, MODE_IDLE):
            self._cmd_pub.publish(Twist())

    def _watchdog_cb(self):
        """If in FOLLOW and target lost for timeout → go IDLE."""
        if self._mode != MODE_FOLLOW:
            return
        elapsed = time.time() - self._last_target_t
        if elapsed > self._timeout:
            self.get_logger().warn(
                f'[WATCHDOG] Target lost for {elapsed:.1f}s → IDLE'
            )
            self._set_mode(MODE_IDLE)

    # ════════════════════════════════════════════════════════════════════
    # HELPERS
    # ════════════════════════════════════════════════════════════════════

    def _set_mode(self, new_mode: str):
        if new_mode == self._mode:
            return
        self._mode = new_mode
        # Always stop immediately when entering STOP or IDLE
        if new_mode in (MODE_STOP, MODE_IDLE):
            self._cmd_pub.publish(Twist())
            self.get_logger().info(f'Cart stopped (mode={new_mode})')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()