#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Teleop(Node):

    def __init__(self):
        super().__init__('teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def move(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Teleop()

    print("Controls:")
    print("w = forward | s = reverse | a = left | d = right")

    try:
        while True:
            key = input("Command: ")

            if key == 'w':
                node.move(1.0, 0.0)
            elif key == 's':
                node.move(-1.0, 0.0)
            elif key == 'a':
                node.move(0.0, 1.0)
            elif key == 'd':
                node.move(0.0, -1.0)
            else:
                node.move(0.0, 0.0)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
