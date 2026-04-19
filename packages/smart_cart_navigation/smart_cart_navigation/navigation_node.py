#!/usr/bin/env python3
"""
navigation_node.py  –  Smart Cart Navigation State Machine

IMPORTANT: Uses self.get_clock().now() throughout – NOT time.time().
This is required for use_sim_time=True compatibility. Mixing wall
clock with sim time causes the watchdog to fire immediately on startup
and silently breaks the node.

Modes
-----
  IDLE    Cart stationary. Waiting for remote button 2.
  FOLLOW  Follow-Me active – follow_me_node drives the cart.
  STOP    Remote button 1. Cart locked until button 2 pressed.
  MANUAL  Reserved for future use.

Remote buttons (from teleop_person_node)
-----------------------------------------
  1 → STOP
  2 → FOLLOW
  3 → IDLE

Topics
------
  Sub: /scan              LaserScan  – detect person in front arc
  Sub: /odom              Odometry   – track cart distance
  Sub: /nav/mode_cmd      String     – remote button commands
  Pub: /nav/current_mode  String     – current mode (2 Hz)
  Pub: /nav/status        String     – human-readable status (2 Hz)
  Pub: /cmd_vel_raw       Twist      – zero velocity in STOP/IDLE
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.clock import ClockType
import rclpy.clock
from std_msgs.msg import String, Float32MultiArray


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
        self.declare_parameter('mode',            'idle')
        self.declare_parameter('follow_distance', 1.0)
        self.declare_parameter('max_speed',       0.8)
        self.declare_parameter('target_timeout',  5.0)

        raw_mode       = self.get_parameter('mode').value.upper()
        self._mode     = raw_mode if raw_mode in VALID_MODES else MODE_IDLE
        self._follow_d = self.get_parameter('follow_distance').value
        self._max_spd  = self.get_parameter('max_speed').value
        self._timeout  = self.get_parameter('target_timeout').value

        # ── State  (use ROS clock – compatible with use_sim_time) ────────
        self._target_visible  = False
        self._last_target_sec = self.get_clock().now().nanoseconds / 1e9
        self._cart_x          = 0.0
        self._cart_y          = 0.0
        self._total_dist      = 0.0
        self._prev_x          = None
        self._prev_y          = None

        # ── Subscriptions ────────────────────────────────────────────────
        self.create_subscription(LaserScan, '/scan',         self._scan_cb,     10)
        self.create_subscription(Odometry,  '/odom',         self._odom_cb,     10)
        self.create_subscription(String,    '/nav/mode_cmd', self._mode_cmd_cb, 10)
        self.create_subscription(Float32MultiArray, '/uwb/distances', self._uwb_nav_cb, 10)

        # ── Publishers ───────────────────────────────────────────────────
        self._mode_pub   = self.create_publisher(String, '/nav/current_mode', 10)
        self._status_pub = self.create_publisher(String, '/nav/status',       10)
        self._cmd_pub    = self.create_publisher(Twist,  '/cmd_vel_raw',      10)

        # Immediately publish zero so cart cannot drift on startup
        self._cmd_pub.publish(Twist())

        # ── Timers ───────────────────────────────────────────────────────
        self.create_timer(
            0.5, self._status_timer_cb,
            clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
        self.create_timer(
            1.0, self._watchdog_cb,
            clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))

        self.get_logger().info(
            f'Navigation Node ready | mode={self._mode} | '
            f'Press remote button [2] to start following'
        )
    # ════════════════════════════════════════════════════════════════════
    # SCAN CALLBACK  –  detect person presence with uwb
    # ════════════════════════════════════════════════════════════════════
    def _uwb_nav_cb(self, msg):
        """Keep watchdog alive as long as UWB tag is visible."""
        if len(msg.data) >= 4:
            self._last_target_sec = self.get_clock().now().nanoseconds / 1e9
            self._target_visible  = True


    # ════════════════════════════════════════════════════════════════════
    # SCAN CALLBACK  –  detect person presence in front arc
    # ════════════════════════════════════════════════════════════════════
    def _scan_cb(self, msg: LaserScan):
        arc_rad   = math.radians(PERSON_ARC_DEG)
        angle_inc = msg.angle_increment
        found     = False

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
            # Use ROS clock – works correctly with use_sim_time=True
            self._last_target_sec = self.get_clock().now().nanoseconds / 1e9

    # ════════════════════════════════════════════════════════════════════
    # ODOMETRY CALLBACK  –  track distance travelled
    # ════════════════════════════════════════════════════════════════════
    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._cart_x = x
        self._cart_y = y
        if self._prev_x is not None:
            dx = x - self._prev_x
            dy = y - self._prev_y
            self._total_dist += math.sqrt(dx * dx + dy * dy)
        self._prev_x = x
        self._prev_y = y

    # ════════════════════════════════════════════════════════════════════
    # MODE COMMAND CALLBACK  –  remote button presses
    # ════════════════════════════════════════════════════════════════════
    def _mode_cmd_cb(self, msg: String):
        requested = msg.data.strip().upper()
        if requested not in VALID_MODES:
            self.get_logger().warn(f'Unknown mode command: "{msg.data}"')
            return
        old = self._mode
        self._set_mode(requested)
        self.get_logger().info(f'[REMOTE] {old} → {self._mode}')

    # ════════════════════════════════════════════════════════════════════
    # STATUS TIMER  –  publish mode + status, hold zero in STOP/IDLE
    # ════════════════════════════════════════════════════════════════════
    def _status_timer_cb(self):
        mode_msg      = String()
        mode_msg.data = self._mode
        self._mode_pub.publish(mode_msg)

        tgt           = 'visible' if self._target_visible else 'not visible'
        status_msg    = String()
        status_msg.data = (
            f'mode={self._mode} | target={tgt} | '
            f'pos=({self._cart_x:.2f},{self._cart_y:.2f}) | '
            f'dist={self._total_dist:.2f}m'
        )
        self._status_pub.publish(status_msg)

        # Keep publishing zero in STOP/IDLE so cart cannot drift
        if self._mode in (MODE_STOP, MODE_IDLE):
            self._cmd_pub.publish(Twist())

    # ════════════════════════════════════════════════════════════════════
    # WATCHDOG  –  if target lost for timeout → go IDLE
    # ════════════════════════════════════════════════════════════════════
    def _watchdog_cb(self):
        if self._mode != MODE_FOLLOW:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        elapsed = now_sec - self._last_target_sec

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