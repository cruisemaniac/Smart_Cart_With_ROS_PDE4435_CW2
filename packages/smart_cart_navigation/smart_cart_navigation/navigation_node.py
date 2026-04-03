import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Navigation node started')

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Subscribe to cmd_vel_raw from follow_me / obstacle_stop
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.cmd_callback,
            10
        )

        # Publish final cmd_vel to the robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Navigation node ready — listening on /cmd_vel_raw')

    def odom_callback(self, msg):
        # Log position occasionally
        pos = msg.pose.pose.position
        self.get_logger().debug(f'Position: x={pos.x:.2f} y={pos.y:.2f}')

    def cmd_callback(self, msg):
        # Forward velocity commands to the robot
        self.cmd_pub.publish(msg)
        self.get_logger().debug(
            f'cmd_vel → linear={msg.linear.x:.2f} angular={msg.angular.z:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)       
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()