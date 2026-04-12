#!/usr/bin/env python3
"""
uwb_simulator_node.py  –  UWB Trilateration Simulator
======================================================
Simulates the UWB (Ultra-Wideband) positioning system described in CW1.

Real system:
  - 4 UWB anchors mounted at cart corners
  - 1 UWB tag on person's remote
  - Anchors measure time-of-flight to tag → trilaterate position
  - Output: distance + XY position of tag relative to cart

Simulation:
  - Uses /odom (cart) and /person/odom (person) to get true positions
  - Computes true distance and angle between cart and person
  - Adds realistic Gaussian noise (±0.10m) to simulate UWB error
  - Publishes simulated UWB measurements at 20 Hz

This node gives the follow_me_node a reliable, identity-locked
distance measurement — the cart always follows THIS person's tag,
not just the nearest LiDAR reflection.

Topics
------
  Sub: /odom           nav_msgs/Odometry   – cart position
  Sub: /person/odom    nav_msgs/Odometry   – person position
  Pub: /uwb/distance   std_msgs/Float32    – distance to person (m)
  Pub: /uwb/angle      std_msgs/Float32    – angle to person (rad)
  Pub: /uwb/position   geometry_msgs/Point – XY position of person
                                             in cart frame
"""

import math
import random
import rclpy
import rclpy.clock
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Point


# ── UWB noise parameters (realistic DW1000 module) ───────────────────────────
UWB_NOISE_STDDEV  = 0.08   # ±0.08m standard deviation (typical UWB accuracy)
UWB_NOISE_BIAS    = 0.02   # small positive bias (multipath effect)
UWB_UPDATE_HZ     = 20.0   # UWB update rate

# ── Maximum reliable UWB range ────────────────────────────────────────────────
UWB_MAX_RANGE_M   = 15.0   # beyond this, signal too weak


class UWBSimulatorNode(Node):

    def __init__(self):
        super().__init__('uwb_simulator_node')

        # Cart and person positions (world frame)
        self._cart_x   = 0.0
        self._cart_y   = 0.0
        self._cart_yaw = 0.0

        self._person_x = None
        self._person_y = None

        self._person_received = False

        # Subscriptions
        self.create_subscription(
            Odometry, '/odom', self._cart_odom_cb, 10)
        self.create_subscription(
            Odometry, '/person/odom', self._person_odom_cb, 10)

        # Publishers
        self._dist_pub = self.create_publisher(Float32, '/uwb/distance', 10)
        self._angle_pub = self.create_publisher(Float32, '/uwb/angle',   10)
        self._pos_pub  = self.create_publisher(Point,   '/uwb/position', 10)

        # Timer – publish at UWB_UPDATE_HZ using steady clock
        # (independent of sim time so it always fires)
        self.create_timer(
            1.0 / UWB_UPDATE_HZ,
            self._publish_uwb,
            clock=rclpy.clock.Clock(
                clock_type=rclpy.clock.ClockType.STEADY_TIME))

        self.get_logger().info(
            f'UWB Simulator Node started | '
            f'noise={UWB_NOISE_STDDEV}m | rate={UWB_UPDATE_HZ}Hz'
        )

    # ── Cart odometry ─────────────────────────────────────────────────────────
    def _cart_odom_cb(self, msg: Odometry):
        self._cart_x = msg.pose.pose.position.x
        self._cart_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._cart_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ── Person odometry ───────────────────────────────────────────────────────
    def _person_odom_cb(self, msg: Odometry):
        self._person_x = msg.pose.pose.position.x
        self._person_y = msg.pose.pose.position.y
        self._person_received = True

    # ── Publish simulated UWB measurements ───────────────────────────────────
    def _publish_uwb(self):

        if not self._person_received:
            return

        # ── True distance and angle (world frame) ────────────────────────────
        dx = self._person_x - self._cart_x
        dy = self._person_y - self._cart_y

        true_dist  = math.sqrt(dx * dx + dy * dy)
        true_angle_world = math.atan2(dy, dx)

        # ── Transform angle to cart body frame ───────────────────────────────
        # angle relative to cart's current heading
        true_angle_body = true_angle_world - self._cart_yaw

        # Normalise to [-π, π]
        while true_angle_body >  math.pi: true_angle_body -= 2 * math.pi
        while true_angle_body < -math.pi: true_angle_body += 2 * math.pi

        # ── Add UWB noise ─────────────────────────────────────────────────────
        noisy_dist = true_dist + UWB_NOISE_BIAS + random.gauss(0, UWB_NOISE_STDDEV)
        noisy_dist = max(0.0, noisy_dist)   # distance can't be negative

        # Angle noise is much smaller (sub-degree level for UWB)
        noisy_angle = true_angle_body + random.gauss(0, 0.02)

        # ── Out of range check ────────────────────────────────────────────────
        if noisy_dist > UWB_MAX_RANGE_M:
            self.get_logger().warn(
                f'UWB: person out of range ({noisy_dist:.1f}m)',
                throttle_duration_sec=2.0
            )
            return

        # ── Publish distance ──────────────────────────────────────────────────
        dist_msg      = Float32()
        dist_msg.data = float(noisy_dist)
        self._dist_pub.publish(dist_msg)

        # ── Publish angle ─────────────────────────────────────────────────────
        angle_msg      = Float32()
        angle_msg.data = float(noisy_angle)
        self._angle_pub.publish(angle_msg)

        # ── Publish XY position in cart frame ─────────────────────────────────
        pos_msg   = Point()
        pos_msg.x = float(noisy_dist * math.cos(noisy_angle))
        pos_msg.y = float(noisy_dist * math.sin(noisy_angle))
        pos_msg.z = 0.0
        self._pos_pub.publish(pos_msg)

        self.get_logger().debug(
            f'UWB: dist={noisy_dist:.2f}m  '
            f'angle={math.degrees(noisy_angle):.1f}°  '
            f'pos=({pos_msg.x:.2f}, {pos_msg.y:.2f})',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = UWBSimulatorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
