#!/usr/bin/env python3
"""
cart_teleop_node.py  –  Direct cart teleop for testing
=======================================================
Publishes directly to /cmd_vel (bypasses cmd_vel_raw and obstacle_stop_node)
to verify raw cart movement in Gazebo.

Controls:
  W   Forward
  S   Backward
  A   Turn left
  D   Turn right
  Q   Forward + turn left
  E   Forward + turn right
  Z   Backward + turn left
  C   Backward + turn right
  SPACE  Stop
  +/-  Speed up/down
  ESC  Quit
"""

import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


LINEAR_SPEED  = 0.3
ANGULAR_SPEED = 0.8
SPEED_STEP    = 0.1
MAX_LINEAR    = 1.0
MIN_LINEAR    = 0.1

BANNER = """
╔══════════════════════════════════════╗
║     CART DIRECT TELEOP TEST          ║
║  publishes → /cmd_vel directly       ║
╠══════════════════════════════════════╣
║  W = Forward      S = Backward       ║
║  A = Turn left    D = Turn right     ║
║  Q = Fwd+Left     E = Fwd+Right      ║
║  Z = Bck+Left     C = Bck+Right      ║
║  SPACE = Stop                        ║
║  + / -  Speed up / down              ║
║  ESC = Quit                          ║
╚══════════════════════════════════════╝
"""


def cprint(text):
    sys.stdout.write('\r' + text + '\r\n')
    sys.stdout.flush()


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    if key == '\x1b':
        extra = sys.stdin.read(2)
        key = '\x1b' + extra
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class CartTeleopNode(Node):

    def __init__(self):
        super().__init__('cart_teleop_node')

        self._linear  = LINEAR_SPEED
        self._angular = ANGULAR_SPEED
        self._pub     = self.create_publisher(Twist, '/cmd_vel', 10)

        # Stop cart on startup
        self._pub.publish(Twist())

        # Status timer
        self.create_timer(1.0, self._status)
        self.get_logger().info('Cart Teleop Node started → /cmd_vel')

    def _status(self):
        cprint(f'[CartTeleop] linear={self._linear:.1f}  angular={self._angular:.1f}')

    def send(self, lx=0.0, az=0.0):
        cmd           = Twist()
        cmd.linear.x  = lx
        cmd.angular.z = az
        self._pub.publish(cmd)

    def process_key(self, key):
        k = key.lower() if len(key) == 1 else key

        if k == 'w':
            self.send(lx=self._linear);         cprint(f'FORWARD  {self._linear:.1f}')
        elif k == 's':
            self.send(lx=-self._linear);        cprint(f'BACKWARD {self._linear:.1f}')
        elif k == 'a':
            self.send(az=self._angular);        cprint(f'LEFT     {self._angular:.1f}')
        elif k == 'd':
            self.send(az=-self._angular);       cprint(f'RIGHT    {self._angular:.1f}')
        elif k == 'q':
            self.send(self._linear,  self._angular);  cprint('FWD+LEFT')
        elif k == 'e':
            self.send(self._linear, -self._angular);  cprint('FWD+RIGHT')
        elif k == 'z':
            self.send(-self._linear,  self._angular); cprint('BCK+LEFT')
        elif k == 'c':
            self.send(-self._linear, -self._angular); cprint('BCK+RIGHT')
        elif k == ' ':
            self.send();                        cprint('STOP')
        elif k in ('+', '='):
            self._linear  = min(self._linear  + SPEED_STEP, MAX_LINEAR)
            self._angular = min(self._angular + SPEED_STEP, 2.0)
            cprint(f'SPEED UP → linear={self._linear:.1f}')
        elif k in ('-', '_'):
            self._linear  = max(self._linear  - SPEED_STEP, MIN_LINEAR)
            self._angular = max(self._angular - SPEED_STEP, 0.2)
            cprint(f'SPEED DN → linear={self._linear:.1f}')

    def stop(self):
        self._pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CartTeleopNode()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    settings = termios.tcgetattr(sys.stdin)

    for line in BANNER.strip().split('\n'):
        sys.stdout.write(line + '\r\n')
    sys.stdout.flush()

    try:
        while rclpy.ok():
            key = get_key(settings)
            if key in ('\x1b', '\x03'):   # ESC or Ctrl+C
                break
            node.process_key(key)

    except Exception as e:
        cprint(f'Error: {e}')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        sys.stdout.write('\r\n[CartTeleop] Shutdown.\r\n')
        sys.stdout.flush()


if __name__ == '__main__':
    main()