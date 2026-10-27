"""Forward ideal simulated odometry and own odom -> base_link TF."""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def valid_odometry(message):
    p = message.pose.pose.position
    q = message.pose.pose.orientation
    return (
        message.header.frame_id == "odom" and
        message.child_frame_id == "base_link" and
        all(math.isfinite(v) for v in [p.x, p.y, p.z, q.x, q.y, q.z, q.w]) and
        abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) <= .01
    )


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("localization")
        self.publisher = self.create_publisher(Odometry, "state/odom", 1)
        self.tf = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry, "sensors/odom", self.receive, 1)

    def receive(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        if not valid_odometry(message):
            return
        self.publisher.publish(message)
        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = message.child_frame_id
        transform.transform.translation.x = p.x
        transform.transform.translation.y = p.y
        transform.transform.translation.z = p.z
        transform.transform.rotation = q
        self.tf.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
