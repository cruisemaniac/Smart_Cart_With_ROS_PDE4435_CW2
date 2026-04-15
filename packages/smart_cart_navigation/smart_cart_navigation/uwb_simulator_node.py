#!/usr/bin/env python3
"""
uwb_simulator_node.py  –  UWB Trilateration Simulator
======================================================
Simulates the 4-anchor UWB system described in CW1.

Real system:
  - 4 UWB anchors at cart corners measure time-of-flight to person's tag
  - Each anchor gives one range measurement
  - Trilateration computes person XY position in cart frame

Simulation:
  - Uses /odom (cart) + /person/odom (person) for true positions
  - Computes true distance from each anchor to person tag
  - Adds per-anchor Gaussian noise (realistic UWB ±0.08m)
  - Publishes all 4 distances as Float32MultiArray on /uwb/distances
  - Also publishes convenience topics /uwb/distance and /uwb/angle

Anchor layout (matches smart_cart.urdf.xacro exactly):
  A0 = front-left  ( 0.32,  0.23)
  A1 = front-right ( 0.32, -0.23)
  A2 = rear-left   (-0.32,  0.23)
  A3 = rear-right  (-0.32, -0.23)

Topics
------
  Sub: /odom              nav_msgs/Odometry
  Sub: /person/odom       nav_msgs/Odometry
  Pub: /uwb/distances     std_msgs/Float32MultiArray  [d0,d1,d2,d3] metres
  Pub: /uwb/distance      std_msgs/Float32            centre distance
  Pub: /uwb/angle         std_msgs/Float32            angle in cart frame
"""

import math
import random
import numpy as np

import rclpy
import rclpy.clock
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Point


# ── Anchor positions in cart body frame (metres) ──────────────────────────────
# Must match smart_cart.urdf.xacro uwb_anchor origins exactly
ANCHORS = np.array([
    [ 0.32,  0.23],   # A0 front-left
    [ 0.32, -0.23],   # A1 front-right
    [-0.32,  0.23],   # A2 rear-left
    [-0.32, -0.23],   # A3 rear-right
])

# ── UWB noise (DW1000 module realistic values) ────────────────────────────────
UWB_NOISE_STDDEV = 0.08   # ±0.08m per anchor
UWB_NOISE_BIAS   = 0.02   # small positive bias (multipath)
UWB_UPDATE_HZ    = 20.0
UWB_MAX_RANGE_M  = 15.0


class UWBSimulatorNode(Node):

    def __init__(self):
        super().__init__('uwb_simulator_node')

        # State
        self._cart_x   = 0.0
        self._cart_y   = 0.0
        self._cart_yaw = 0.0

        self._person_x = 0.0
        self._person_y = 0.0
        self._person_ready = False
        self.kalman = Kalman2D()

        # Subscriptions
        self.create_subscription(
            Odometry, '/odom',        self._cart_odom_cb,   10)
        self.create_subscription(
            Odometry, '/person/odom', self._person_odom_cb, 10)

        # Publishers
        self._distances_pub = self.create_publisher(
            Float32MultiArray, '/uwb/distances', 10)
        self._distance_pub  = self.create_publisher(
            Float32, '/uwb/distance', 10)
        self._angle_pub     = self.create_publisher(
            Float32, '/uwb/angle',    10)

        # Publish timer – steady clock (always fires, sim-time independent)
        self.create_timer(
            1.0 / UWB_UPDATE_HZ,
            self._publish_uwb,
            clock=rclpy.clock.Clock(
                clock_type=rclpy.clock.ClockType.STEADY_TIME))

        self.get_logger().info(
            f'UWB Simulator started | '
            f'4 anchors | noise={UWB_NOISE_STDDEV}m | {UWB_UPDATE_HZ}Hz'
        )

    # ── Cart odometry ─────────────────────────────────────────────────────────
    def _cart_odom_cb(self, msg: Odometry):
        self._cart_x = msg.pose.pose.position.x
        self._cart_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._cart_yaw = math.atan2(siny, cosy)

    # ── Person odometry ───────────────────────────────────────────────────────
    def _person_odom_cb(self, msg: Odometry):
        self._person_x = msg.pose.pose.position.x
        self._person_y = msg.pose.pose.position.y
        self._person_ready = True

    # ── Main publish loop ─────────────────────────────────────────────────────
    def _publish_uwb(self):

        if not self._person_ready:
            return

        # ── Transform person position into cart body frame ────────────────────
        # World frame vector from cart centre to person
        dx_world = self._person_x - self._cart_x
        dy_world = self._person_y - self._cart_y

        # Rotate into cart frame using cart yaw
        cos_y = math.cos(-self._cart_yaw)
        sin_y = math.sin(-self._cart_yaw)
        px = cos_y * dx_world - sin_y * dy_world   # forward (+x = front of cart)
        py = sin_y * dx_world + cos_y * dy_world   # lateral (+y = left of cart)

        raw_xy = np.array([px, py])
        smooth_xy = self.kalman.update(raw_xy)

        px, py = smooth_xy

        person_cart = np.array([px, py])

        # ── Compute per-anchor distances with noise ───────────────────────────
        noisy_distances = []
        for anchor in ANCHORS:
            diff      = person_cart - anchor
            true_dist = float(np.linalg.norm(diff))
            noisy     = true_dist + UWB_NOISE_BIAS + random.gauss(0, UWB_NOISE_STDDEV)
            noisy     = max(0.0, noisy)
            noisy_distances.append(noisy)

        # ── Publish Float32MultiArray [d0, d1, d2, d3] ───────────────────────
        dist_array      = Float32MultiArray()
        dist_array.data = [float(d) for d in noisy_distances]
        self._distances_pub.publish(dist_array)

        # ── Convenience: centre distance (from cart origin 0,0) ──────────────
        centre_dist  = float(np.linalg.norm(person_cart))
        noisy_centre = centre_dist + random.gauss(0, UWB_NOISE_STDDEV * 0.5)
        noisy_centre = max(0.0, noisy_centre)

        dist_msg      = Float32()
        dist_msg.data = noisy_centre
        self._distance_pub.publish(dist_msg)

        # ── Convenience: angle in cart frame ─────────────────────────────────
        true_angle  = math.atan2(py, px)
        noisy_angle = true_angle + random.gauss(0, 0.02)

        angle_msg      = Float32()
        angle_msg.data = float(noisy_angle)
        self._angle_pub.publish(angle_msg)

        self.get_logger().debug(
            f'UWB: person_cart=({px:.2f},{py:.2f})  '
            f'dist={noisy_centre:.2f}m  angle={math.degrees(noisy_angle):.1f}°  '
            f'anchors={[f"{d:.2f}" for d in noisy_distances]}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = UWBSimulatorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()