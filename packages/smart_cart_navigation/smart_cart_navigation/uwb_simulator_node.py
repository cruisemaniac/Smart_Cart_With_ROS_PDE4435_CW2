#!/usr/bin/env python3
"""
uwb_simulator_node.py  -  UWB Trilateration Simulator
Publishes RViz markers: line cart→person + distance text label.
"""

import math
import random
import numpy as np

import rclpy
import rclpy.clock
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

ANCHORS = np.array([
    [ 0.32,  0.23],
    [ 0.32, -0.23],
    [-0.32,  0.23],
    [-0.32, -0.23],
])

UWB_NOISE_STDDEV   = 0.08
UWB_NOISE_BIAS     = 0.02
UWB_UPDATE_HZ      = 20.0
PERSON_SPAWN_X     = 2.0
PERSON_SPAWN_Y     = 0.0


class UWBSimulatorNode(Node):

    def __init__(self):
        super().__init__('uwb_simulator_node')

        self._cart_x   = 0.0
        self._cart_y   = 0.0
        self._cart_yaw = 0.0

        self._person_world_x       = PERSON_SPAWN_X
        self._person_world_y       = PERSON_SPAWN_Y
        self._person_ready         = False
        self._person_odom_origin_x = None
        self._person_odom_origin_y = None

        self.create_subscription(Odometry, '/odom',        self._cart_odom_cb,   10)
        self.create_subscription(Odometry, '/person/odom', self._person_odom_cb, 10)

        self._distances_pub = self.create_publisher(Float32MultiArray, '/uwb/distances', 10)
        self._distance_pub  = self.create_publisher(Float32,           '/uwb/distance',  10)
        self._angle_pub     = self.create_publisher(Float32,           '/uwb/angle',     10)

        # RViz marker publisher
        self._marker_pub = self.create_publisher(MarkerArray, '/uwb/markers', 10)

        self.create_timer(
            1.0 / UWB_UPDATE_HZ,
            self._publish_uwb,
            clock=rclpy.clock.Clock(
                clock_type=rclpy.clock.ClockType.STEADY_TIME))

        self.get_logger().info('UWB Simulator started - publishing RViz markers on /uwb/markers')

    def _cart_odom_cb(self, msg: Odometry):
        self._cart_x = msg.pose.pose.position.x
        self._cart_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._cart_yaw = math.atan2(siny, cosy)

    def _person_odom_cb(self, msg: Odometry):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        if self._person_odom_origin_x is None:
            self._person_odom_origin_x = PERSON_SPAWN_X - ox
            self._person_odom_origin_y = PERSON_SPAWN_Y - oy
            self.get_logger().info(
                f'Person odom calibrated: offset=('
                f'{self._person_odom_origin_x:.3f},'
                f'{self._person_odom_origin_y:.3f})')
        self._person_world_x = ox + self._person_odom_origin_x
        self._person_world_y = oy + self._person_odom_origin_y
        self._person_ready   = True

    def _publish_uwb(self):
        if not self._person_ready:
            return

        dx_world = self._person_world_x - self._cart_x
        dy_world = self._person_world_y - self._cart_y

        cos_y = math.cos(-self._cart_yaw)
        sin_y = math.sin(-self._cart_yaw)
        px = cos_y * dx_world - sin_y * dy_world
        py = sin_y * dx_world + cos_y * dy_world

        person_cart  = np.array([px, py])
        centre_dist  = float(np.linalg.norm(person_cart))

        # ── Noisy anchor distances ─────────────────────────────────────────
        noisy_distances = []
        for anchor in ANCHORS:
            true_dist = float(np.linalg.norm(person_cart - anchor))
            noisy     = true_dist + UWB_NOISE_BIAS + random.gauss(0, UWB_NOISE_STDDEV)
            noisy_distances.append(max(0.0, noisy))

        dist_array      = Float32MultiArray()
        dist_array.data = [float(d) for d in noisy_distances]
        self._distances_pub.publish(dist_array)

        noisy_centre  = max(0.0, centre_dist + random.gauss(0, UWB_NOISE_STDDEV * 0.5))
        dist_msg      = Float32()
        dist_msg.data = noisy_centre
        self._distance_pub.publish(dist_msg)

        noisy_angle   = math.atan2(py, px) + random.gauss(0, 0.02)
        angle_msg     = Float32()
        angle_msg.data = float(noisy_angle)
        self._angle_pub.publish(angle_msg)

        # ── RViz markers ───────────────────────────────────────────────────
        self._publish_markers(centre_dist)

        self.get_logger().info(
            f'UWB: world_person=({self._person_world_x:.2f},{self._person_world_y:.2f}) '
            f'cart=({self._cart_x:.2f},{self._cart_y:.2f}) '
            f'cart_frame=({px:.2f},{py:.2f}) '
            f'dist={centre_dist:.2f}m '
            f'yaw={math.degrees(self._cart_yaw):.1f}°',
            throttle_duration_sec=1.0
        )

    def _publish_markers(self, dist: float):
        markers = MarkerArray()
        now     = self.get_clock().now().to_msg()

        # ── Marker 1: LINE from cart to person ────────────────────────────
        line                 = Marker()
        line.header.frame_id = 'odom'
        line.header.stamp    = now
        line.ns              = 'uwb'
        line.id              = 0
        line.type            = Marker.LINE_STRIP
        line.action          = Marker.ADD
        line.scale.x         = 0.03   # line width metres

        # Green line
        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 0.0
        line.color.a = 1.0

        # Cart position (z=0.3 so visible above ground)
        cart_pt   = Point()
        cart_pt.x = self._cart_x
        cart_pt.y = self._cart_y
        cart_pt.z = 0.3

        # Person position in world frame
        person_pt   = Point()
        person_pt.x = self._person_world_x
        person_pt.y = self._person_world_y
        person_pt.z = 0.3

        line.points = [cart_pt, person_pt]
        markers.markers.append(line)

        # ── Marker 2: TEXT showing distance at midpoint ────────────────────
        text                 = Marker()
        text.header.frame_id = 'odom'
        text.header.stamp    = now
        text.ns              = 'uwb'
        text.id              = 1
        text.type            = Marker.TEXT_VIEW_FACING
        text.action          = Marker.ADD

        # Midpoint between cart and person
        text.pose.position.x = (self._cart_x + self._person_world_x) / 2.0
        text.pose.position.y = (self._cart_y + self._person_world_y) / 2.0
        text.pose.position.z = 0.8   # float above the line

        text.pose.orientation.w = 1.0
        text.scale.z            = 0.25   # text height metres
        text.color.r            = 1.0
        text.color.g            = 1.0
        text.color.b            = 0.0
        text.color.a            = 1.0
        text.text               = f'{dist:.2f} m'
        markers.markers.append(text)

        # ── Marker 3: SPHERE at person position ───────────────────────────
        sphere                 = Marker()
        sphere.header.frame_id = 'odom'
        sphere.header.stamp    = now
        sphere.ns              = 'uwb'
        sphere.id              = 2
        sphere.type            = Marker.SPHERE
        sphere.action          = Marker.ADD
        sphere.pose.position.x = self._person_world_x
        sphere.pose.position.y = self._person_world_y
        sphere.pose.position.z = 0.3
        sphere.pose.orientation.w = 1.0
        sphere.scale.x         = 0.2
        sphere.scale.y         = 0.2
        sphere.scale.z         = 0.2
        sphere.color.r         = 0.0
        sphere.color.g         = 0.5
        sphere.color.b         = 1.0
        sphere.color.a         = 0.8
        markers.markers.append(sphere)

        # ── Marker 4: CYLINDER at person position (visible person shape) ──────
        person_shape                 = Marker()
        person_shape.header.frame_id = 'odom'
        person_shape.header.stamp    = now
        person_shape.ns              = 'uwb'
        person_shape.id              = 3
        person_shape.type            = Marker.CYLINDER
        person_shape.action          = Marker.ADD
        person_shape.pose.position.x = self._person_world_x
        person_shape.pose.position.y = self._person_world_y
        person_shape.pose.position.z = 0.9   # half height
        person_shape.pose.orientation.w = 1.0
        person_shape.scale.x         = 0.4   # diameter
        person_shape.scale.y         = 0.4
        person_shape.scale.z         = 1.8   # height (person ~1.8m)
        person_shape.color.r         = 1.0
        person_shape.color.g         = 0.5
        person_shape.color.b         = 0.0
        person_shape.color.a         = 0.8
        markers.markers.append(person_shape)

        self._marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = UWBSimulatorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()