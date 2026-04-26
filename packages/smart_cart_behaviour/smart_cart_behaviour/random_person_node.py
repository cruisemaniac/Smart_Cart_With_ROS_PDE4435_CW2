#!/usr/bin/env python3
"""
random_person_node.py
=====================
Parameterised autonomous random-walking pedestrian.
One binary, launched three times with different person_ns values.

Avoidance:
  Static walls/shelves  — directed AABB check (3-ray cone in FORWARD, 1-ray in escape)
  Dynamic entities       — pure distance check (NO direction check, avoids oscillation)
"""

import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# ── Supermarket static obstacle AABBs (world frame) ──────────────────────────
# Format: (x_min, x_max, y_min, y_max)
# World: X∈[-7,14]  Y∈[-5,5]
OBSTACLES = [
    # Boundary walls
    (-7.000, 14.000,  4.900,  5.100),   # left_wall  (y=+5.0)
    (-7.000, 14.000, -5.100, -4.900),   # right_wall (y=-5.0)
    (13.900, 14.100, -5.100,  5.100),   # back_wall  (x=14.0)
    (-7.100, -6.900,  2.000,  5.000),   # front_wall_left
    (-7.100, -6.900, -5.000, -2.000),   # front_wall_right
    (-7.500, -6.500, -2.000,  2.000),   # virtual entrance stop (gap)
    # Main aisle shelves
    ( 1.500,  5.500,  1.275,  1.725),   # shelf_A_left
    ( 1.500,  5.500, -1.725, -1.275),   # shelf_A_right
    ( 7.000, 11.000,  1.275,  1.725),   # shelf_B_left
    ( 7.000, 11.000, -1.725, -1.275),   # shelf_B_right
    # Perimeter wall shelves (y=±4.5, x∈[-3,11])
    (-3.000, 11.000,  4.300,  4.700),   # perim_shelf_left
    (-3.000, 11.000, -4.700, -4.300),   # perim_shelf_right
    # Checkout counters
    (-4.250, -2.750,  2.850,  3.550),   # checkout_left
    (-4.250, -2.750, -3.550, -2.850),   # checkout_right
    # Produce display tables
    (-6.500, -4.500,  2.900,  4.100),   # produce_table_left
    (-6.500, -4.500, -4.100, -2.900),   # produce_table_right
    # Refrigerated coolers (near back wall)
    (12.100, 12.900,  1.250,  4.750),   # cooler_left
    (12.100, 12.900, -4.750, -1.250),   # cooler_right
    # Mid-aisle obstacle box (preserved)
    ( 1.850,  2.150,  0.450,  0.750),   # obstacle_box
]

# ── All random pedestrian world spawn positions ───────────────────────────────
# Must match launch file -x/-y arguments exactly
ALL_RANDOM_PEOPLE = {
    'random_person':   ( 0.0, -3.5),
    'random_person_2': (-3.0,  4.0),
    'random_person_3': ( 4.8,  0.0),
}

# Fixed entity spawns (cart and main person)
CART_SPAWN_X,   CART_SPAWN_Y   =  0.0,  0.0
PERSON_SPAWN_X, PERSON_SPAWN_Y =  2.0,  0.0

# ── Tuning ────────────────────────────────────────────────────────────────────
FORWARD_SPEED    = 0.35      # m/s — slow enough to react in time
TURN_SPEED       = 0.80      # rad/s
SAFETY_DIST      = 0.80      # m — static obstacle (wall/shelf) clearance
DYNAMIC_DIST     = 0.85      # m — dynamic entity clearance (pure distance, no direction)
CONE_OFFSETS     = [0.0, math.radians(15), math.radians(-15)]
MIN_TURN_RAD     = math.pi * 2 / 3   # 120°
MAX_TURN_RAD     = math.pi           # 180°
MIN_WALK_TIME    = 2.0
MAX_WALK_TIME    = 6.0


class RandomPersonNode(Node):

    _STATE_FORWARD = 'forward'
    _STATE_TURN    = 'turn'

    def __init__(self):
        super().__init__('random_person_node')

        self.declare_parameter('person_ns', 'random_person')
        ns = self.get_parameter('person_ns').get_parameter_value().string_value
        self._my_ns = ns

        sx, sy = ALL_RANDOM_PEOPLE.get(ns, (0.0, 0.0))

        self._cmd_pub = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
        self.create_subscription(Odometry, f'/{ns}/odom',    self._odom_cb,        10)
        self.create_subscription(Odometry, '/odom',          self._cart_odom_cb,   10)
        self.create_subscription(Odometry, '/person/odom',   self._person_odom_cb, 10)

        # Track other random pedestrians
        self._others = {}
        for other_ns, (osx, osy) in ALL_RANDOM_PEOPLE.items():
            if other_ns == ns:
                continue
            self._others[other_ns] = {'x': osx, 'y': osy, 'sx': osx, 'sy': osy}
            self.create_subscription(
                Odometry, f'/{other_ns}/odom',
                lambda msg, n=other_ns: self._other_odom_cb(msg, n), 10)

        # Self pose — world = spawn + raw_odom (odom always starts at 0,0 at spawn)
        self._spawn_x = sx
        self._spawn_y = sy
        self._x   = sx
        self._y   = sy
        self._yaw = 0.0

        # Dynamic entity positions
        self._cart_x,   self._cart_y   = CART_SPAWN_X,   CART_SPAWN_Y
        self._person_x, self._person_y = PERSON_SPAWN_X, PERSON_SPAWN_Y

        # State machine
        self._state          = self._STATE_FORWARD
        self._turn_remaining = 0.0
        self._turn_sign      = 1.0
        self._walk_timer     = self._random_walk_time()

        self.create_timer(0.1, self._control_cb)
        self.get_logger().info(f'[{ns}] started at spawn ({sx},{sy})')

    # ── Odom callbacks (world = spawn + raw_odom) ────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._x = self._spawn_x + msg.pose.pose.position.x
        self._y = self._spawn_y + msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def _cart_odom_cb(self, msg: Odometry):
        self._cart_x = CART_SPAWN_X + msg.pose.pose.position.x
        self._cart_y = CART_SPAWN_Y + msg.pose.pose.position.y

    def _person_odom_cb(self, msg: Odometry):
        self._person_x = PERSON_SPAWN_X + msg.pose.pose.position.x
        self._person_y = PERSON_SPAWN_Y + msg.pose.pose.position.y

    def _other_odom_cb(self, msg: Odometry, ns: str):
        e = self._others[ns]
        e['x'] = e['sx'] + msg.pose.pose.position.x
        e['y'] = e['sy'] + msg.pose.pose.position.y

    # ── Geometry helpers ─────────────────────────────────────────────────────

    def _nearest_static_dist(self, px, py, hdx, hdy):
        min_dist = float('inf')
        for (x0, x1, y0, y1) in OBSTACLES:
            nx = max(x0, min(px, x1))
            ny = max(y0, min(py, y1))
            tx, ty = nx - px, ny - py
            d = math.hypot(tx, ty)
            if d == 0.0:
                return 0.0          # inside obstacle — always blocked
            if tx * hdx + ty * hdy > 0.0 and d < min_dist:
                min_dist = d
        return min_dist

    def _any_dynamic_too_close(self):
        """Pure distance check — no direction. Avoids oscillation."""
        entities = [
            (self._cart_x,   self._cart_y),
            (self._person_x, self._person_y),
        ] + [(e['x'], e['y']) for e in self._others.values()]
        for (ex, ey) in entities:
            if math.hypot(ex - self._x, ey - self._y) < DYNAMIC_DIST:
                return True
        return False

    def _path_clear(self, yaw, strict=True):
        """
        strict=True  (walking): 3-ray cone against static obstacles
        strict=False (escape after turn): single center ray only
        Both use same pure-distance dynamic check.
        """
        offsets = CONE_OFFSETS if strict else [0.0]
        for off in offsets:
            dx, dy = math.cos(yaw + off), math.sin(yaw + off)
            if self._nearest_static_dist(self._x, self._y, dx, dy) < SAFETY_DIST:
                return False
        return not self._any_dynamic_too_close()

    # ── State helpers ────────────────────────────────────────────────────────

    def _start_turn(self, force_angle=0.0):
        angle = force_angle if force_angle > 0.0 else random.uniform(MIN_TURN_RAD, MAX_TURN_RAD)
        # Prefer the direction that leads to a clearer heading
        left_clear  = self._path_clear(self._yaw + angle, strict=False)
        right_clear = self._path_clear(self._yaw - angle, strict=False)
        if left_clear and not right_clear:
            sign = 1.0
        elif right_clear and not left_clear:
            sign = -1.0
        else:
            sign = random.choice([-1.0, 1.0])
        self._turn_remaining = angle
        self._turn_sign      = sign
        self._state          = self._STATE_TURN
        self._walk_timer     = self._random_walk_time()

    def _random_walk_time(self):
        return random.uniform(MIN_WALK_TIME, MAX_WALK_TIME)

    # ── Control loop ─────────────────────────────────────────────────────────

    def _control_cb(self):
        dt  = 0.1
        cmd = Twist()

        if self._state == self._STATE_FORWARD:
            if not self._path_clear(self._yaw, strict=True):
                self._start_turn(force_angle=random.uniform(MIN_TURN_RAD, MAX_TURN_RAD))
            else:
                self._walk_timer -= dt
                if self._walk_timer <= 0.0:
                    self._start_turn()
                else:
                    cmd.linear.x = FORWARD_SPEED

        elif self._state == self._STATE_TURN:
            step = TURN_SPEED * dt
            if self._turn_remaining <= step:
                self._turn_remaining = 0.0
                if self._path_clear(self._yaw, strict=False):
                    self._state      = self._STATE_FORWARD
                    self._walk_timer = self._random_walk_time()
                else:
                    # Still blocked — keep turning 30° at a time
                    self._turn_remaining = math.radians(30)
                    cmd.angular.z = self._turn_sign * TURN_SPEED
            else:
                self._turn_remaining -= step
                cmd.angular.z = self._turn_sign * TURN_SPEED

        self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RandomPersonNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
