"""Synthetic test card, not a rendered onboard camera."""
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DemoCamera(Node):
    def __init__(self):
        super().__init__("demo_camera")
        self.publisher = self.create_publisher(
            Image, "camera/image_raw", qos_profile_sensor_data)
        self.bridge = CvBridge()
        self.timer = self.create_timer(.1, self.publish)

    def publish(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(image, (260, 180), (380, 300), (0, 255, 0), -1)
        cv2.putText(image, "SYNTHETIC TEST CARD", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
        message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "camera_optical_frame"
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = DemoCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
