#!/usr/bin/env python3
"""
teleop_person_node.py  –  Smart Cart Person Teleop + Remote Control
====================================================================
Hold-to-move: movement starts on key-press and stops on key-release.

Two-phase key-timeout to handle the OS key-repeat initial delay (~500 ms):
  - First press  → KEY_FIRST_TIMEOUT (0.60 s) so the person keeps moving
                   while waiting for terminal key-repeat to kick in.
  - Repeat active → KEY_HOLD_TIMEOUT (0.10 s) so movement stops within
                    100 ms of release once repeats are flowing.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  MOVEMENT  (hold key)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  W / ↑   Forward
  S / ↓   Backward
  A / ←   Turn left
  D / →   Turn right
  SPACE   Stop

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  REMOTE BUTTONS  (single press)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  1   STOP      cart stops immediately
  2   FOLLOW    cart enters follow-me mode
  3   IDLE      cart standby

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  SPEED
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  +   Speed up
  -   Speed down
  R   Reset speed

  ESC / Ctrl+C   Quit
"""

import sys
import select
import tty
import termios
import threading
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32


# ── Speed settings ─────────────────────────────────────────────────────────
DEFAULT_LINEAR_SPEED  = 0.6
DEFAULT_ANGULAR_SPEED = 0.5
SPEED_STEP            = 0.1
MAX_LINEAR_SPEED      = 1.5
MIN_LINEAR_SPEED      = 0.1

# Two-phase hold-detection timeouts.
# KEY_FIRST_TIMEOUT bridges the OS key-repeat initial delay (~500 ms on most
# Linux desktops).  Once rapid repeats are detected, KEY_HOLD_TIMEOUT gives a
# fast 100 ms stop after the key is released.
KEY_FIRST_TIMEOUT     = 0.60   # linear keys (W/S) — bridges 500 ms OS repeat delay
KEY_FIRST_TIMEOUT_ANG = 0.20   # angular keys (A/D) — short to avoid yaw overshoot
KEY_HOLD_TIMEOUT      = 0.10   # both — once repeat is active, fast stop on release

# ── Person spawn position (must match launch file -x / -y arguments) ──────
PERSON_SPAWN_X = 2.0
PERSON_SPAWN_Y = 0.0

# ── Map-based collision avoidance — AABBs derived from supermarket.sdf ────
# Format: (x_min, x_max, y_min, y_max)
OBSTACLES = [
    # Outer walls
    ( -7.00,  14.00,   4.90,   5.10),  # left_wall        pose=(3.5, 5.0)  size=(21,0.2)
    ( -7.00,  14.00,  -5.10,  -4.90),  # right_wall       pose=(3.5,-5.0)
    (  13.90,  14.10, -5.10,   5.10),  # back_wall        pose=(14.0, 0.0) size=(0.2,10.2)
    (  -7.10,  -6.90,  2.00,   5.00),  # front_wall_left  pose=(-7.0, 3.5) size=(0.2,3.0)
    (  -7.10,  -6.90, -5.00,  -2.00),  # front_wall_right pose=(-7.0,-3.5)
    # Perimeter shelves
    (  -3.01,  11.01,  4.295,  4.705), # perim_shelf_left  pose=(4.0, 4.5) size=(14.02,0.41)
    (  -3.01,  11.01, -4.705, -4.295), # perim_shelf_right pose=(4.0,-4.5)
    # Main aisle shelves
    (   1.49,   5.51,  1.27,   1.73),  # shelf_A_left  pose=(3.5, 1.5) size=(4.02,0.46)
    (   1.49,   5.51, -1.73,  -1.27),  # shelf_A_right pose=(3.5,-1.5)
    (   6.99,  11.01,  1.27,   1.73),  # shelf_B_left  pose=(9.0, 1.5)
    (   6.99,  11.01, -1.73,  -1.27),  # shelf_B_right pose=(9.0,-1.5)
    # Checkout counters
    (  -4.25,  -2.75,  2.85,   3.55),  # checkout_left  pose=(-3.5, 3.2) size=(1.5,0.7)
    (  -4.25,  -2.75, -3.55,  -2.85),  # checkout_right pose=(-3.5,-3.2)
    # Cooler units
    (  12.10,  12.90,  1.25,   4.75),  # cooler_left  pose=(12.5, 3.0) size=(0.8,3.5)
    (  12.10,  12.90, -4.75,  -1.25),  # cooler_right pose=(12.5,-3.0)
    # Small obstacle box
    (   1.85,   2.15,  0.45,   0.75),  # obstacle_box pose=(2.0, 0.6) size=(0.3,0.3)
]
PERSON_RADIUS = 0.35

# ── Remote button map ──────────────────────────────────────────────────────
REMOTE_BUTTONS = {
    '1': ('STOP',   1, 'STOP'),
    '2': ('FOLLOW', 2, 'FOLLOW'),
    '3': ('IDLE',   3, 'IDLE'),
}

BANNER = """
+------------------------------------------------------+
|       SMART CART  -  Person Remote Control           |
+------------------------------------------------------+
|  MOVEMENT (hold key)  |  REMOTE BUTTONS              |
|  W/Up  Forward        |  1 -> STOP  (cart stops)     |
|  S/Dn  Backward       |  2 -> FOLLOW-ME              |
|  A/Lt  Turn left      |  3 -> IDLE  (standby)        |
|  D/Rt  Turn right     |                              |
|  SPACE Stop (instant) |  SPEED:  + / -  adjust       |
|                       |          R      reset         |
+------------------------------------------------------+
|  ESC / Ctrl+C  Quit                                  |
+------------------------------------------------------+
"""


def cprint(text: str):
    sys.stdout.write('\r' + text + '\r\n')
    sys.stdout.flush()


class TeleopPersonNode(Node):

    def __init__(self):
        super().__init__('teleop_person_node')

        self._linear_speed  = DEFAULT_LINEAR_SPEED
        self._angular_speed = DEFAULT_ANGULAR_SPEED
        self._current_mode  = 'IDLE'
        self._running       = True

        # Two-phase hold-detection state
        self._held_twist    = Twist()
        self._last_key_t    = 0.0       # time of most recent movement key char
        self._repeat_active = False     # True once rapid repeats detected
        self._is_angular    = False     # True when last movement key was A/D

        # Person pose in world frame, calibrated from /person/odom
        self._odom_x      = PERSON_SPAWN_X
        self._odom_y      = PERSON_SPAWN_Y
        self._odom_yaw    = 0.0
        self._odom_init_x = None
        self._odom_init_y = None
        self._odom_ready  = False
        self._blocked     = False

        self._cmd_pub   = self.create_publisher(Twist,  '/person/cmd_vel',    10)
        self._btn_pub   = self.create_publisher(String, '/remote/button',     10)
        self._btn_i_pub = self.create_publisher(Int32,  '/remote/button_int', 10)
        self._nav_pub   = self.create_publisher(String, '/nav/mode_cmd',      10)

        self.create_subscription(Odometry, '/person/odom', self._odom_cb, 10)

        self._cmd_pub.publish(Twist())

        self.create_timer(0.05, self._publish_cb)   # 20 Hz command loop
        self.create_timer(1.0,  self._status_cb)    # 1 Hz status line

    # ── Odom callback ───────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        if self._odom_init_x is None:
            self._odom_init_x = raw_x
            self._odom_init_y = raw_y

        self._odom_x = PERSON_SPAWN_X + (raw_x - self._odom_init_x)
        self._odom_y = PERSON_SPAWN_Y + (raw_y - self._odom_init_y)

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw   = math.atan2(siny_cosp, cosy_cosp)
        self._odom_ready = True

    # ── Map-based collision check ────────────────────────────────────────────

    def _is_blocked_in_direction(self, twist: Twist) -> bool:
        if twist.linear.x == 0.0 or not self._odom_ready:
            return False

        sign    = 1.0 if twist.linear.x > 0 else -1.0
        move_dx = math.cos(self._odom_yaw) * sign
        move_dy = math.sin(self._odom_yaw) * sign
        px, py  = self._odom_x, self._odom_y

        for (x_min, x_max, y_min, y_max) in OBSTACLES:
            nx   = max(x_min, min(px, x_max))
            ny   = max(y_min, min(py, y_max))
            to_x = nx - px
            to_y = ny - py
            dist = math.sqrt(to_x * to_x + to_y * to_y)
            if dist < PERSON_RADIUS and to_x * move_dx + to_y * move_dy > 0.0:
                return True

        return False

    # ── Timers ──────────────────────────────────────────────────────────────

    def _publish_cb(self):
        """20 Hz — two-phase hold-to-move with obstacle blocking."""
        now     = time.monotonic()
        elapsed = now - self._last_key_t

        if self._repeat_active:
            timeout = KEY_HOLD_TIMEOUT
        elif self._is_angular:
            timeout = KEY_FIRST_TIMEOUT_ANG   # short — prevents yaw overshoot
        else:
            timeout = KEY_FIRST_TIMEOUT
        key_held = elapsed <= timeout

        if not key_held:
            if self._held_twist.linear.x != 0.0 or self._held_twist.angular.z != 0.0:
                self._held_twist    = Twist()
                self._repeat_active = False
                self._blocked       = False
                self._cmd_pub.publish(Twist())
            return

        out = Twist()
        out.linear.x  = self._held_twist.linear.x
        out.angular.z = self._held_twist.angular.z

        if self._is_blocked_in_direction(self._held_twist):
            if not self._blocked:
                self._blocked = True
                cprint('[BLOCKED] Obstacle — linear suppressed, turn still works')
            out.linear.x = 0.0
        else:
            self._blocked = False

        self._cmd_pub.publish(out)

    def _status_cb(self):
        if self._repeat_active:
            timeout = KEY_HOLD_TIMEOUT
        elif self._is_angular:
            timeout = KEY_FIRST_TIMEOUT_ANG
        else:
            timeout = KEY_FIRST_TIMEOUT
        elapsed = time.monotonic() - self._last_key_t
        moving  = elapsed <= timeout and (
            self._held_twist.linear.x != 0.0 or self._held_twist.angular.z != 0.0)
        cprint(
            f'[Status] speed={self._linear_speed:.1f}m/s  '
            f'mode={self._current_mode}  '
            f'pos=({self._odom_x:.2f},{self._odom_y:.2f})  '
            f'yaw={math.degrees(self._odom_yaw):.0f}°'
            f'{"  [MOVING]"  if moving  else ""}  '
            f'{"  [BLOCKED]" if self._blocked else ""}  '
        )

    # ── Key handling ────────────────────────────────────────────────────────

    def _key_to_twist(self, key: str):
        cmd = Twist()
        if key in ('w', '\x1b[A'):
            cmd.linear.x  =  self._linear_speed
        elif key in ('s', '\x1b[B'):
            cmd.linear.x  = -self._linear_speed
        elif key in ('a', '\x1b[D'):
            cmd.angular.z =  self._angular_speed
        elif key in ('d', '\x1b[C'):
            cmd.angular.z = -self._angular_speed
        elif key == ' ':
            pass  # zero twist = stop
        else:
            return None
        return cmd

    def process_key(self, key: str):

        # Remote buttons — single press
        if key in REMOTE_BUTTONS:
            label, number, nav_mode = REMOTE_BUTTONS[key]
            btn_msg   = String(); btn_msg.data   = label
            btn_i_msg = Int32();  btn_i_msg.data = number
            nav_msg   = String(); nav_msg.data   = nav_mode
            self._btn_pub.publish(btn_msg)
            self._btn_i_pub.publish(btn_i_msg)
            self._nav_pub.publish(nav_msg)
            self._current_mode = nav_mode
            cprint(f'[REMOTE] Button {number} pressed -> {label}')
            return

        # Speed adjust — single press
        if key in ('+', '='):
            self._linear_speed  = min(self._linear_speed  + SPEED_STEP, MAX_LINEAR_SPEED)
            self._angular_speed = min(self._angular_speed + SPEED_STEP * 2, 3.0)
            cprint(f'[SPEED] Linear: {self._linear_speed:.1f} m/s')
            return

        if key in ('-', '_'):
            self._linear_speed  = max(self._linear_speed  - SPEED_STEP, MIN_LINEAR_SPEED)
            self._angular_speed = max(self._angular_speed - SPEED_STEP * 2, 0.3)
            cprint(f'[SPEED] Linear: {self._linear_speed:.1f} m/s')
            return

        if key == 'r':
            self._linear_speed  = DEFAULT_LINEAR_SPEED
            self._angular_speed = DEFAULT_ANGULAR_SPEED
            cprint(f'[SPEED] Reset to {DEFAULT_LINEAR_SPEED:.1f} m/s')
            return

        # Movement — two-phase hold-to-move
        twist = self._key_to_twist(key)
        if twist is not None:
            now = time.monotonic()
            gap = now - self._last_key_t
            is_ang = key in ('a', 'd', '\x1b[D', '\x1b[C')

            if gap <= KEY_HOLD_TIMEOUT:
                self._repeat_active = True
            else:
                self._repeat_active = False

            self._is_angular = is_ang
            self._last_key_t = now
            self._held_twist = twist

    def stop(self):
        self._running = False
        self._cmd_pub.publish(Twist())


# ── Non-blocking terminal key reader ───────────────────────────────────────

def _read_key(fd) -> str | None:
    """
    Read one keypress from raw stdin using select (non-blocking, 20 ms poll).
    Returns the key string, or None if no input was available.
    Handles ESC sequences (arrow keys).
    """
    r, _, _ = select.select([sys.stdin], [], [], 0.020)
    if not r:
        return None

    ch = sys.stdin.read(1)
    if ch != '\x1b':
        return ch

    # ESC — try to read the rest of an escape sequence within 20 ms
    r2, _, _ = select.select([sys.stdin], [], [], 0.020)
    if not r2:
        return '\x1b'          # bare ESC key

    ch2 = sys.stdin.read(1)
    if ch2 != '[':
        return '\x1b'

    r3, _, _ = select.select([sys.stdin], [], [], 0.020)
    if not r3:
        return '\x1b'

    ch3 = sys.stdin.read(1)
    return '\x1b[' + ch3      # e.g. '\x1b[A' = Up arrow


def main(args=None):
    rclpy.init(args=args)
    node = TeleopPersonNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    settings = termios.tcgetattr(sys.stdin)

    for line in BANNER.strip().split('\n'):
        sys.stdout.write(line + '\r\n')
    sys.stdout.write(f'  Initial speed: {DEFAULT_LINEAR_SPEED} m/s\r\n\r\n')
    sys.stdout.flush()

    fd = sys.stdin.fileno()
    try:
        tty.setraw(fd)
        while rclpy.ok() and node._running:
            key = _read_key(fd)
            if key is None:
                continue
            if key in ('\x1b', '\x03'):   # ESC or Ctrl+C
                break
            node.process_key(key.lower() if key.isalpha() else key)

    except Exception as e:
        cprint(f'Error: {e}')

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        sys.stdout.write('\r\n[Teleop] Shutdown complete.\r\n')
        sys.stdout.flush()


if __name__ == '__main__':
    main()
