"""Bounded latest-image inference; image results are not navigation goals."""
from pathlib import Path
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from mzj_interfaces.msg import SemanticDetection, SemanticDetections
from mzj_perception.detector import detect_marker
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception")
        self.backend = self.declare_parameter("backend", "color_demo").value
        weights = self.declare_parameter("weights", "").value
        self.model = None
        if self.backend == "yolo":
            if not Path(weights).is_file():
                raise ValueError("YOLO requires an existing local weights file")
            from ultralytics import YOLO
            self.model = YOLO(weights)
        elif self.backend != "color_demo":
            raise ValueError("backend must be color_demo or yolo")
        self.bridge = CvBridge()
        self.output = self.create_publisher(
            SemanticDetections, "perception/detections", 1)
        self.pending = None
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.stopping = threading.Event()
        self.subscription = self.create_subscription(
            Image, "camera/image_raw", self.receive, qos_profile_sensor_data)
        self.worker = threading.Thread(target=self.infer, daemon=True)
        self.worker.start()

    def receive(self, message):
        with self.lock:
            self.pending = message
        self.wake.set()

    def infer(self):
        while not self.stopping.is_set():
            self.wake.wait(.2)
            self.wake.clear()
            with self.lock:
                message, self.pending = self.pending, None
            if message is None:
                continue
            try:
                image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                if self.model is None:
                    results = detect_marker(image)
                else:
                    prediction = self.model.predict(source=image, verbose=False)[0]
                    results = []
                    for box in prediction.boxes:
                        x, y, w, h = box.xywh[0].cpu().tolist()
                        results.append((prediction.names[int(box.cls[0])],
                                        float(box.conf[0]), x, y, w, h))
                output = SemanticDetections()
                output.header = message.header
                for label, confidence, x, y, w, h in results:
                    output.detections.append(SemanticDetection(
                        class_name=label, confidence=confidence, center_x=x,
                        center_y=y, width=w, height=h))
                if not self.stopping.is_set() and rclpy.ok():
                    self.output.publish(output)
            except Exception as error:
                if not self.stopping.is_set():
                    self.get_logger().error(f"Inference failed: {error}")

    def close(self):
        self.stopping.set()
        self.wake.set()
        self.worker.join()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
