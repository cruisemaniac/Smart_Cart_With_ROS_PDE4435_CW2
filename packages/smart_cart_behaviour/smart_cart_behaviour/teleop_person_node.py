#!/usr/bin/env python3
"""
teleop_person_node.py  –  Smart Cart Person Teleop + Remote Control
====================================================================
Hold-to-move: WASD commands are sent only while the key is held down.
Releasing the key stops the person within one key-repeat interval (~0.12 s).

Map-based collision avoidance
------------------------------
The person's world position is read from /person/odom.  Before sending any
forward/backward command, a probe point PERSON_RADIUS ahead of the person in
the commanded direction is checked against the static obstacle bounding boxes
extracted from supermarket.sdf.  If the probe lands inside an obstacle the
linear command is suppressed while angular (turning) is still passed through
so the person can turn away from the wall.

This prevents the person from ever physically contacting a wall, which in
turn prevents the diff-drive odometry from drifting (drift happens when wheel
joints keep spinning while the body is blocked).

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
DEFAULT_ANGULAR_SPEED = 1.2
SPEED_STEP            = 0.1
MAX_LINEAR_SPEED      = 1.5
MIN_LINEAR_SPEED      = 0.1

# Hold-to-move: key-repeat fires every ~30–50 ms; timeout must be longer
KEY_TIMEOUT_SEC = 0.12

# ── Person spawn position (must match launch file -x / -y arguments) ──────
# The diff-drive odometry resets to (0, 0) at spawn, not the world position.
# We calibrate on the first odom message so world_pos = SPAWN + (odom - odom0).
PERSON_SPAWN_X = 2.0
PERSON_SPAWN_Y = 0.0

# ── Map-based collision avoidance — from supermarket.sdf ───────────────────
# Each entry: (x_min, x_max, y_min, y_max) exact AABB in WORLD frame.
# Derived directly from SDF pose ± size/2.  No inflation — PERSON_RADIUS
# provides the clearance from the surface.
#
# left_wall:   pose=(0, 2.5, 1)      size=(14, 0.15, 2)
# right_wall:  pose=(0,-2.5, 1)      size=(14, 0.15, 2)
# back_wall:   pose=(6.5, 0, 1)      size=(0.15, 5.3, 2)
# shelf_left:  pose=(3.5, 1.5, .75)  size=(1.8, 0.45, 1.5)
# shelf_right: pose=(3.5,-1.5, .75)  size=(1.8, 0.45, 1.5)
# obstacle_box:pose=(2.0, 0.6, .25)  size=(0.3, 0.3, 0.5)
OBSTACLES = [
    (-7.000,  7.000,  2.425,  2.575),   # left_wall
    (-7.000,  7.000, -2.575, -2.425),   # right_wall
    ( 6.425,  6.575, -2.650,  2.650),   # back_wall
    ( 2.600,  4.400,  1.275,  1.725),   # shelf_left
    ( 2.600,  4.400, -1.725, -1.275),   # shelf_right
    ( 1.850,  2.150,  0.450,  0.750),   # obstacle_box
]

# The person stops this far (metres) from any obstacle surface.
# Set to at least body_radius + braking_margin.
# At default speed 0.6 m/s, max_accel 2.0 m/s²: braking dist = 0.09 m.
# 0.35 m gives comfortable margin for all normal approach speeds.
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

        # Hold-to-move state
        self._held_twist = Twist()
        self._last_key_t = 0.0

        # Person pose in world frame, calibrated from /person/odom.
        # _odom_init_* captures the first odom reading (which Gazebo may set
        # to 0,0 or to the spawn world position depending on plugin version).
        # We subtract it and add the known spawn position so world coords are
        # always correct regardless of how Gazebo initialises the odom.
        self._odom_x      = PERSON_SPAWN_X
        self._odom_y      = PERSON_SPAWN_Y
        self._odom_yaw    = 0.0
        self._odom_init_x = None
        self._odom_init_y = None
        self._odom_ready  = False

        self._blocked = False

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
            # Calibrate once: whatever Gazebo gives as the first odom reading
            # (could be 0,0 or the spawn world position), treat it as SPAWN.
            self._odom_init_x = raw_x
            self._odom_init_y = raw_y

        # World position = spawn + displacement from first odom reading
        self._odom_x = PERSON_SPAWN_X + (raw_x - self._odom_init_x)
        self._odom_y = PERSON_SPAWN_Y + (raw_y - self._odom_init_y)

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw   = math.atan2(siny_cosp, cosy_cosp)
        self._odom_ready = True

    # ── Map-based collision check ────────────────────────────────────────────

    def _is_blocked_in_direction(self, twist: Twist) -> bool:
        """
        For each obstacle AABB, find the nearest point on its surface to the
        person.  If that distance is less than PERSON_RADIUS AND the person is
        moving toward that obstacle, block the linear command.

        This handles all approach angles, including diagonal approaches to
        shelf corners that a single forward-probe would miss.  Angular (turn)
        commands are never blocked so the person can always turn away.
        """
        if twist.linear.x == 0.0 or not self._odom_ready:
            return False

        sign = 1.0 if twist.linear.x > 0 else -1.0
        move_dx = math.cos(self._odom_yaw) * sign
        move_dy = math.sin(self._odom_yaw) * sign
        px = self._odom_x
        py = self._odom_y

        for (x_min, x_max, y_min, y_max) in OBSTACLES:
            # Nearest point on this AABB to the person's current position
            nx = max(x_min, min(px, x_max))
            ny = max(y_min, min(py, y_max))

            # Vector from person to that nearest point
            to_x = nx - px
            to_y = ny - py
            dist = math.sqrt(to_x * to_x + to_y * to_y)

            if dist < PERSON_RADIUS:
                # Only block if the commanded direction has a component
                # toward this obstacle (positive dot product).
                # This allows backing away from an already-close obstacle.
                if to_x * move_dx + to_y * move_dy > 0.0:
                    return True

        return False

    # ── Timers ──────────────────────────────────────────────────────────────

    def _publish_cb(self):
        """20 Hz — hold-to-move with map-based obstacle blocking."""
        key_held = (time.monotonic() - self._last_key_t) <= KEY_TIMEOUT_SEC

        if not key_held:
            if self._held_twist.linear.x != 0.0 or self._held_twist.angular.z != 0.0:
                self._held_twist = Twist()
                self._blocked    = False
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
        cprint(
            f'[Status] speed={self._linear_speed:.1f}m/s  '
            f'mode={self._current_mode}  '
            f'pos=({self._odom_x:.2f},{self._odom_y:.2f})  '
            f'yaw={math.degrees(self._odom_yaw):.0f}°'
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

        # Movement — hold-to-move
        twist = self._key_to_twist(key)
        if twist is not None:
            self._held_twist = twist
            self._last_key_t = time.monotonic()

    def stop(self):
        self._running = False
        self._cmd_pub.publish(Twist())


# ── Terminal key reader ─────────────────────────────────────────────────────

def get_key(settings) -> str:
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    if key == '\x1b':
        key2 = sys.stdin.read(1)
        if key2 == '[':
            key3 = sys.stdin.read(1)
            key  = '\x1b[' + key3
        else:
            key = '\x1b'
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


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

    try:
        while rclpy.ok() and node._running:
            key = get_key(settings)
            if key in ('\x1b', '\x03'):
                break
            node.process_key(key.lower() if key.isalpha() else key)

    except Exception as e:
        cprint(f'Error: {e}')

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        sys.stdout.write('\r\n[Teleop] Shutdown complete.\r\n')
        sys.stdout.flush()


if __name__ == '__main__':
    main()
