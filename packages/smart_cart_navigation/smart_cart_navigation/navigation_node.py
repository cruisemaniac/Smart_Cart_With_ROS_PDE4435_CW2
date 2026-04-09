#!/usr/bin/env python3
"""
navigation_node.py  –  Smart Cart Navigation State Machine
===========================================================
Manages the cart's operating mode and coordinates between
follow-me behaviour and manual/override control.

Modes
-----
  IDLE      Cart is stationary. Waiting for a target.
  FOLLOW    Follow-Me active. Relies on follow_me_node via /cmd_vel_raw.
  STOP      Emergency or manual stop override.
  MANUAL    Teleop input takes priority over follow-me.

Topic graph
-----------
  Subscribes:
    /scan               – LaserScan  (monitors for people / obstacles)
    /odom               – Odometry   (tracks cart position)
    /cmd_vel_raw        – Twist      (from follow_me_node)
    /nav/mode_cmd       – String     (external mode change: "follow","idle","stop","manual")

  Publishes:
    /nav/current_mode   – String     (current state, latched)
    /nav/status         – String     (human-readable status string)
    /cmd_vel_raw        – Twist      (re-publishes zeros when in STOP/IDLE)

Parameters
----------
  mode            (str,   default 'follow')  – startup mode
  follow_distance (float, default 1.0)       – target gap in metres
  max_speed       (float, default 0.8)       – max linear velocity m/s
  target_timeout  (float, default 3.0)       – seconds before "lost target" → IDLE
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# ── Mode constants ────────────────────────────────────────────────────────────
MODE_IDLE   = 'IDLE'
MODE_FOLLOW = 'FOLLOW'
MODE_STOP   = 'STOP'
MODE_MANUAL = 'MANUAL'

VALID_MODES = {MODE_IDLE, MODE_FOLLOW, MODE_STOP, MODE_MANUAL}

# Detection range for "is there a person in front?"
PERSON_MIN_M = 0.3
PERSON_MAX_M = 4.0
PERSON_ARC_DEG = 60.0


class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')

        # ── Declare parameters ────────────────────────────────────────────
        self.declare_parameter('mode',            'follow')
        self.declare_parameter('follow_distance', 1.0)
        self.declare_parameter('max_speed',       0.8)
        self.declare_parameter('target_timeout',  3.0)

        startup_mode    = self.get_parameter('mode').value.upper()
        self._follow_d  = self.get_parameter('follow_distance').value
        self._max_spd   = self.get_parameter('max_speed').value
        self._timeout   = self.get_parameter('target_timeout').value

        # ── State ─────────────────────────────────────────────────────────
        self._mode            = startup_mode if startup_mode in VALID_MODES else MODE_FOLLOW
        self._last_target_t   = time.time()
        self._target_visible  = False
        self._cart_x          = 0.0
        self._cart_y          = 0.0
        self._total_distance  = 0.0
        self._prev_x          = None
        self._prev_y          = None

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(LaserScan, '/scan',        self._scan_cb,     10)
        self.create_subscription(Odometry,  '/odom',        self._odom_cb,     10)
        self.create_subscription(String,    '/nav/mode_cmd',self._mode_cmd_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────
        self._mode_pub   = self.create_publisher(String, '/nav/current_mode', 10)
        self._status_pub = self.create_publisher(String, '/nav/status',       10)
        self._cmd_pub    = self.create_publisher(Twist,  '/cmd_vel_raw',      10)

        # ── Status timer – publishes mode + status at 2 Hz ───────────────
        self.create_timer(0.5, self._status_timer_cb)

        # ── Watchdog timer – checks for lost target at 1 Hz ──────────────
        self.create_timer(1.0, self._watchdog_cb)

        self.get_logger().info(
            f'Navigation Node started  |  mode={self._mode}  '
            f'follow_dist={self._follow_d}m  max_speed={self._max_spd}m/s'
        )

    # ══════════════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ══════════════════════════════════════════════════════════════════════════

    def _scan_cb(self, msg: LaserScan):
        """
        Check the front arc for a detectable target (the person cylinder).
        Updates _target_visible and _last_target_t.
        """
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

        # If we're in IDLE and a target appears, auto-switch to FOLLOW
        if self._mode == MODE_IDLE and found:
            self._set_mode(MODE_FOLLOW)
            self.get_logger().info('Target detected – switching IDLE → FOLLOW')

    def _odom_cb(self, msg: Odometry):
        """Track cart position and accumulate distance travelled."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self._cart_x = x
        self._cart_y = y

        if self._prev_x is not None and self._prev_y is not None:
            dx = x - self._prev_x
            dy = y - self._prev_y
            self._total_distance += math.sqrt(dx*dx + dy*dy)

        self._prev_x = x
        self._prev_y = y

    def _mode_cmd_cb(self, msg: String):
        """
        External mode change command.
        Publish to /nav/mode_cmd with one of: follow, idle, stop, manual
        """
        requested = msg.data.strip().upper()
        if requested not in VALID_MODES:
            self.get_logger().warn(
                f'Unknown mode command: "{msg.data}"  '
                f'(valid: {", ".join(VALID_MODES)})'
            )
            return
        old = self._mode
        self._set_mode(requested)
        self.get_logger().info(f'Mode change command: {old} → {self._mode}')

    # ══════════════════════════════════════════════════════════════════════════
    # TIMERS
    # ══════════════════════════════════════════════════════════════════════════

    def _status_timer_cb(self):
        """Publish current mode and a human-readable status string."""
        mode_msg        = String()
        mode_msg.data   = self._mode
        self._mode_pub.publish(mode_msg)

        target_str = 'visible' if self._target_visible else 'not visible'
        status_str = (
            f'mode={self._mode} | '
            f'target={target_str} | '
            f'pos=({self._cart_x:.2f},{self._cart_y:.2f}) | '
            f'dist_travelled={self._total_distance:.2f}m'
        )
        status_msg      = String()
        status_msg.data = status_str
        self._status_pub.publish(status_msg)

        # In STOP or IDLE mode, ensure zero velocity is published
        if self._mode in (MODE_STOP, MODE_IDLE):
            self._cmd_pub.publish(Twist())

    def _watchdog_cb(self):
        """
        If in FOLLOW mode and no target has been seen for target_timeout
        seconds, switch to IDLE and stop the cart.
        """
        if self._mode != MODE_FOLLOW:
            return

        elapsed = time.time() - self._last_target_t
        if elapsed > self._timeout:
            self.get_logger().warn(
                f'Target lost for {elapsed:.1f}s – switching FOLLOW → IDLE'
            )
            self._set_mode(MODE_IDLE)
            self._cmd_pub.publish(Twist())  # stop immediately

    # ══════════════════════════════════════════════════════════════════════════
    # HELPERS
    # ══════════════════════════════════════════════════════════════════════════

    def _set_mode(self, new_mode: str):
        if new_mode == self._mode:
            return
        self.get_logger().info(f'Mode: {self._mode} → {new_mode}')
        self._mode = new_mode

        # Immediately stop motion when switching to STOP or IDLE
        if new_mode in (MODE_STOP, MODE_IDLE):
            self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()