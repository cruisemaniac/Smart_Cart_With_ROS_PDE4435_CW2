#!/usr/bin/env python3
"""
teleop_person_node.py  –  Smart Cart Person Teleop + Remote Control
====================================================================
Controls the keyboard-driven person model in Gazebo AND sends
remote button signals to the navigation state machine.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  MOVEMENT
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  W / ↑   Forward
  S / ↓   Backward
  A / ←   Turn left
  D / →   Turn right
  SPACE   Stop person

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  REMOTE BUTTONS
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
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32


# ── Speed settings ────────────────────────────────────────────────────────────
DEFAULT_LINEAR_SPEED  = 0.6
DEFAULT_ANGULAR_SPEED = 1.2
SPEED_STEP            = 0.1
MAX_LINEAR_SPEED      = 1.5
MIN_LINEAR_SPEED      = 0.1

# ── Remote button map ─────────────────────────────────────────────────────────
REMOTE_BUTTONS = {
    '1': ('STOP',   1, 'STOP'),
    '2': ('FOLLOW', 2, 'FOLLOW'),
    '3': ('IDLE',   3, 'IDLE'),
}

BANNER = """
+------------------------------------------------------+
|       SMART CART  -  Person Remote Control           |
+------------------------------------------------------+
|  MOVEMENT          |  REMOTE BUTTONS                 |
|  W/Up  Forward     |  1 -> STOP  (cart stops)        |
|  S/Dn  Backward    |  2 -> FOLLOW-ME                 |
|  A/Lt  Turn left   |  3 -> IDLE  (standby)           |
|  D/Rt  Turn right  |                                 |
|  SPACE Stop        |  SPEED:  + / -  adjust          |
|                    |          R      reset            |
+------------------------------------------------------+
|  ESC / Ctrl+C  Quit                                  |
+------------------------------------------------------+
"""


def cprint(text: str):
    """Print a line cleanly even in raw terminal mode."""
    sys.stdout.write('\r' + text + '\r\n')
    sys.stdout.flush()


class TeleopPersonNode(Node):

    def __init__(self):
        super().__init__('teleop_person_node')

        self._linear_speed  = DEFAULT_LINEAR_SPEED
        self._angular_speed = DEFAULT_ANGULAR_SPEED
        self._current_mode  = 'IDLE'
        self._running       = True

        self._cmd_pub   = self.create_publisher(Twist,  '/person/cmd_vel',    10)
        self._btn_pub   = self.create_publisher(String, '/remote/button',     10)
        self._btn_i_pub = self.create_publisher(Int32,  '/remote/button_int', 10)
        self._nav_pub   = self.create_publisher(String, '/nav/mode_cmd',      10)

        self._cmd_pub.publish(Twist())

        # Status timer – 1 Hz, clean single-line output
        self.create_timer(1.0, self._status_cb)

    def _status_cb(self):
        cprint(
            f'[Status] Speed: {self._linear_speed:.1f} m/s  |  '
            f'Cart mode: {self._current_mode}          '
        )

    def _key_to_twist(self, key: str):
        cmd = Twist()
        if key in ('w', '\x1b[A'):
            cmd.linear.x = self._linear_speed   # forward in Gazebo
        elif key in ('s', '\x1b[B'):
            cmd.linear.x =  -self._linear_speed   # backward
        elif key in ('a', '\x1b[D'):
            cmd.angular.z = self._angular_speed  # turn left
        elif key in ('d', '\x1b[C'):
            cmd.angular.z =  -self._angular_speed  # turn right
        elif key == ' ':
            pass   # zero twist = stop
        else:
            return None
        return cmd

    def process_key(self, key: str):

        # Remote buttons
        if key in REMOTE_BUTTONS:
            label, number, nav_mode = REMOTE_BUTTONS[key]
            btn_msg      = String(); btn_msg.data = label
            btn_i_msg    = Int32();  btn_i_msg.data = number
            nav_msg      = String(); nav_msg.data = nav_mode

            self._btn_pub.publish(btn_msg)
            self._btn_i_pub.publish(btn_i_msg)
            self._nav_pub.publish(nav_msg)

            self._current_mode = nav_mode
            cprint(f'[REMOTE] Button {number} pressed -> {label}')
            return

        # Speed adjust
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

        # Movement
        twist = self._key_to_twist(key)
        if twist is not None:
            self._cmd_pub.publish(twist)

    def stop(self):
        self._running = False
        self._cmd_pub.publish(Twist())


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

    # Print banner cleanly
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