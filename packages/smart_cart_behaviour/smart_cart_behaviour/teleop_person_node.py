#!/usr/bin/env python3
"""
teleop_person_node.py  –  Smart Cart Person Teleop + Remote Control
════════════════════════════════════════════════════════════════════
Controls the keyboard-driven person model in Gazebo AND sends
remote button signals to the navigation state machine.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  MOVEMENT (person walks around the supermarket)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  W / ↑   Forward
  S / ↓   Backward
  A / ←   Turn left
  D / →   Turn right
  Q        Strafe left   (if using holonomic mode)
  E        Strafe right
  SPACE    Stop person immediately

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  REMOTE BUTTONS (signal to smart cart)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  1   Button 1 → STOP      cart stops immediately, ignores follow
  2   Button 2 → FOLLOW    cart enters follow-me mode, tracks person
  3   Button 3 → IDLE      cart enters idle (empty button / standby)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  SPEED ADJUST
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  +   Increase person speed
  -   Decrease person speed
  R   Reset speed to default

  ESC / Ctrl+C   Quit

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Topics published:
  /person/cmd_vel      geometry_msgs/Twist   → person movement in Gazebo
  /remote/button       std_msgs/String       → remote button (STOP/FOLLOW/IDLE)
  /remote/button_int   std_msgs/Int32        → same as integer (1/2/3)
  /nav/mode_cmd        std_msgs/String       → directly commands navigation node
"""

import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32


# ── Speed settings ────────────────────────────────────────────────────────────
DEFAULT_LINEAR_SPEED  = 0.6    # m/s  person walk speed
DEFAULT_ANGULAR_SPEED = 1.2    # rad/s person turn speed
SPEED_STEP            = 0.1    # increment per +/- press
MAX_LINEAR_SPEED      = 1.5
MIN_LINEAR_SPEED      = 0.1

# ── Remote button map ─────────────────────────────────────────────────────────
REMOTE_BUTTONS = {
    '1': ('STOP',   1, 'STOP'),
    '2': ('FOLLOW', 2, 'FOLLOW'),
    '3': ('IDLE',   3, 'IDLE'),
}

# ── Terminal display ──────────────────────────────────────────────────────────
BANNER = """
╔══════════════════════════════════════════════════════╗
║         SMART CART  –  Person Remote Control         ║
╠══════════════════════════════════════════════════════╣
║  MOVEMENT          │  REMOTE BUTTONS                 ║
║  W/↑  Forward      │  1 → STOP  (cart stops)         ║
║  S/↓  Backward     │  2 → FOLLOW-ME (cart follows)   ║
║  A/←  Turn left    │  3 → IDLE  (standby)            ║
║  D/→  Turn right   │                                 ║
║  SPACE  Stop       │  SPEED                          ║
║                    │  + / -  adjust speed            ║
║                    │  R      reset speed             ║
╠══════════════════════════════════════════════════════╣
║  ESC / Ctrl+C  Quit                                  ║
╚══════════════════════════════════════════════════════╝
"""


class TeleopPersonNode(Node):

    def __init__(self):
        super().__init__('teleop_person_node')
        self.get_logger().info('Teleop Person Node started')

        # ── Publishers ────────────────────────────────────────────────────
        self._cmd_pub    = self.create_publisher(Twist,  '/person/cmd_vel',    10)
        self._btn_pub    = self.create_publisher(String, '/remote/button',     10)
        self._btn_i_pub  = self.create_publisher(Int32,  '/remote/button_int', 10)
        self._nav_pub    = self.create_publisher(String, '/nav/mode_cmd',      10)

        # ── State ─────────────────────────────────────────────────────────
        self._linear_speed  = DEFAULT_LINEAR_SPEED
        self._angular_speed = DEFAULT_ANGULAR_SPEED
        self._current_mode  = 'IDLE'
        self._running       = True

        # ── Publish zero on startup ───────────────────────────────────────
        self._cmd_pub.publish(Twist())

        # ── Status timer ─────────────────────────────────────────────────
        self.create_timer(1.0, self._status_cb)

    # ── Status printout ───────────────────────────────────────────────────────
    def _status_cb(self):
        self.get_logger().info(
            f'Person speed: {self._linear_speed:.1f} m/s  |  '
            f'Cart mode: {self._current_mode}',
            throttle_duration_sec=1.0
        )

    # ── Key → velocity ────────────────────────────────────────────────────────
    def _key_to_twist(self, key: str) -> Twist | None:
        cmd = Twist()

        if key in ('w', '\x1b[A'):      # W or Up arrow
            cmd.linear.x = self._linear_speed
        elif key in ('s', '\x1b[B'):    # S or Down arrow
            cmd.linear.x = -self._linear_speed
        elif key in ('a', '\x1b[D'):    # A or Left arrow
            cmd.angular.z = -self._angular_speed
        elif key in ('d', '\x1b[C'):    # D or Right arrow
            cmd.angular.z = self._angular_speed
        elif key == 'q':                # strafe left
            cmd.linear.y = self._linear_speed
        elif key == 'e':                # strafe right
            cmd.linear.y = -self._linear_speed
        elif key == ' ':                # SPACE – stop
            pass                        # zero Twist = stop
        else:
            return None                 # not a movement key

        return cmd

    # ── Process key press ─────────────────────────────────────────────────────
    def process_key(self, key: str):

        # ── Remote buttons ────────────────────────────────────────────────
        if key in REMOTE_BUTTONS:
            label, number, nav_mode = REMOTE_BUTTONS[key]

            btn_msg       = String()
            btn_msg.data  = label
            self._btn_pub.publish(btn_msg)

            btn_i_msg      = Int32()
            btn_i_msg.data = number
            self._btn_i_pub.publish(btn_i_msg)

            nav_msg       = String()
            nav_msg.data  = nav_mode
            self._nav_pub.publish(nav_msg)

            self._current_mode = nav_mode
            print(f'\n  [REMOTE] Button {number} pressed → {label}', flush=True)
            return

        # ── Speed adjust ──────────────────────────────────────────────────
        if key == '+' or key == '=':
            self._linear_speed  = min(self._linear_speed  + SPEED_STEP, MAX_LINEAR_SPEED)
            self._angular_speed = min(self._angular_speed + SPEED_STEP * 2, 3.0)
            print(f'\n  [SPEED] Linear: {self._linear_speed:.1f} m/s', flush=True)
            return

        if key == '-' or key == '_':
            self._linear_speed  = max(self._linear_speed  - SPEED_STEP, MIN_LINEAR_SPEED)
            self._angular_speed = max(self._angular_speed - SPEED_STEP * 2, 0.3)
            print(f'\n  [SPEED] Linear: {self._linear_speed:.1f} m/s', flush=True)
            return

        if key == 'r':
            self._linear_speed  = DEFAULT_LINEAR_SPEED
            self._angular_speed = DEFAULT_ANGULAR_SPEED
            print(f'\n  [SPEED] Reset to {DEFAULT_LINEAR_SPEED:.1f} m/s', flush=True)
            return

        # ── Movement ──────────────────────────────────────────────────────
        twist = self._key_to_twist(key)
        if twist is not None:
            self._cmd_pub.publish(twist)

    def stop(self):
        self._running = False
        self._cmd_pub.publish(Twist())  # zero velocity on exit


def get_key_raw(settings) -> str:
    """Read a single keypress from stdin, including arrow keys."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)

    # Handle escape sequences (arrow keys = ESC [ A/B/C/D)
    if key == '\x1b':
        key2 = sys.stdin.read(1)
        if key2 == '[':
            key3 = sys.stdin.read(1)
            key = '\x1b[' + key3   # e.g. '\x1b[A' = Up
        else:
            key = '\x1b'           # bare ESC

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = TeleopPersonNode()

    # Run ROS spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)

    print(BANNER, flush=True)
    print(f'  Initial speed: {DEFAULT_LINEAR_SPEED} m/s\n', flush=True)

    try:
        while rclpy.ok() and node._running:
            key = get_key_raw(settings)

            # Quit on ESC or Ctrl+C
            if key in ('\x1b', '\x03'):
                break

            node.process_key(key.lower() if key.isalpha() else key)

    except Exception as e:
        print(f'\nError: {e}', flush=True)

    finally:
        # Restore terminal + stop person
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        print('\n[Teleop] Shutdown complete.', flush=True)


if __name__ == '__main__':
    main()
