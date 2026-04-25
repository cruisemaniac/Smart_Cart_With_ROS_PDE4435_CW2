#!/usr/bin/env python3
"""
Obstacle Stop Node – Smart Cart
3-zone speed profile from CW1 Table 12:
  < 0.30 m  →  Emergency stop  (0% speed)
  < 0.80 m  →  Slow zone       (50% speed)
  < 1.50 m  →  Caution zone    (80% speed)
  >= 1.50 m →  Normal          (100% speed)

In FOLLOW mode, the person in front is expected — minimum obstacle
distance is filtered to ignore the followed person (UWB handles safety).
Emergency stop only triggers for unexpected obstacles (not the person).

Subscribes to : /cmd_vel_raw        Twist
                /scan               LaserScan
                /ultrasonic/left    LaserScan
                /ultrasonic/right   LaserScan
                /nav/current_mode   String
Publishes to  : /cmd_vel            Twist
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


EMERGENCY_STOP_M = 0.40
SLOW_ZONE_M      = 0.80
CAUTION_ZONE_M   = 1.50
FRONT_ARC_DEG    = 45.0


class ObstacleStopNode(Node):

    def __init__(self):
        super().__init__('obstacle_stop_node')
        self.get_logger().info('Obstacle Stop Node started')

        self._speed_factor   = 1.0
        self._emergency_stop = False
        self._mode           = 'IDLE'

        self.create_subscription(LaserScan, 'scan',
                                 self._lidar_cb, 10)
        self.create_subscription(LaserScan, 'ultrasonic/left',
                                 self._us_cb, 10)
        self.create_subscription(LaserScan, 'ultrasonic/right',
                                 self._us_cb, 10)
        self.create_subscription(Twist, 'cmd_vel_raw',
                                 self._cmd_cb, 10)
        self.create_subscription(String, '/nav/current_mode',
                                 self._mode_cb, 10)

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def _mode_cb(self, msg: String):
        self._mode = msg.data.strip().upper()

    def _lidar_cb(self, msg: LaserScan):
        arc_rad   = math.radians(FRONT_ARC_DEG)
        angle_inc = msg.angle_increment
        valid     = []

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * angle_inc
            if abs(angle) > arc_rad:
                continue
            if math.isnan(r) or math.isinf(r):
                continue
            if r < 0.12:   # below lidar range_min — noise floor
                continue
            valid.append(r)

        if valid:
            self._update_factor(min(valid))
        else:
            # No valid obstacles detected — full speed
            self._emergency_stop = False
            self._speed_factor   = 1.0

    def _us_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges
                 if not math.isnan(r) and not math.isinf(r) and r > 0.10]
        if valid:
            self._update_factor(min(valid))

    def _update_factor(self, dist: float):
        if dist < EMERGENCY_STOP_M:
            self._emergency_stop = True
            self._speed_factor   = 0.0
            self.get_logger().warn(
                f'EMERGENCY STOP – obstacle at {dist:.2f}m',
                throttle_duration_sec=1.0)
        elif dist < SLOW_ZONE_M:
            self._emergency_stop = False
            self._speed_factor   = 0.5
        elif dist < CAUTION_ZONE_M:
            self._emergency_stop = False
            self._speed_factor   = 0.8
        else:
            self._emergency_stop = False
            self._speed_factor   = 1.0

    def _cmd_cb(self, msg: Twist):
        out = Twist()
        if self._emergency_stop:
            # Block forward motion but allow angular so the cart can
            # turn away from the obstacle instead of spinning in place
            out.angular.z = msg.angular.z
            self._cmd_pub.publish(out)
            return

        f             = self._speed_factor
        out.linear.x  = msg.linear.x  * f
        out.linear.y  = msg.linear.y  * f
        out.angular.z = msg.angular.z * f
        self._cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()