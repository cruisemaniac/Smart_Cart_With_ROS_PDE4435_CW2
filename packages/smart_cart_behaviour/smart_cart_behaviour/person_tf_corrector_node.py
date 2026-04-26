#!/usr/bin/env python3
"""
person_tf_corrector_node.py
===========================
Subscribes to /world_dynamic_poses, which is bridged from the built-in
Gazebo Harmonic topic /world/supermarket/dynamic_pose/info.  That topic
publishes true physics-state world poses for every dynamic entity in the
scene — no extra plugins required.

We filter for the "person" entity, relabel its parent frame from the
Gazebo world name ("supermarket") to "odom", and re-broadcast it as the
canonical  odom → person  TF.

Why this exists
---------------
The diff-drive plugin derives odom from wheel joint positions.  When the
person body is blocked by a wall, the wheel motors keep applying torque;
the joints rotate, positions accumulate, and the odom TF drifts — so
RViz shows the person walking through the wall.  The dynamic_pose/info
stream never drifts because it reads the rigid-body transform directly
from the Gazebo physics state.
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import tf2_ros


class PersonTFCorrectorNode(Node):

    def __init__(self):
        super().__init__('person_tf_corrector_node')
        self._br = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(
            TFMessage, '/world_dynamic_poses', self._poses_cb, 10)
        self.get_logger().info(
            'Person TF corrector started — ground-truth odom→person TF via world_dynamic_poses.')

    def _poses_cb(self, msg: TFMessage):
        for tf in msg.transforms:
            if tf.child_frame_id == 'person':
                # dynamic_pose/info sets frame_id to the Gazebo world name.
                # Relabel to "odom" so it slots into the existing TF tree.
                tf.header.frame_id = 'odom'
                self._br.sendTransform(tf)
                return  # only one person


def main(args=None):
    rclpy.init(args=args)
    node = PersonTFCorrectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
