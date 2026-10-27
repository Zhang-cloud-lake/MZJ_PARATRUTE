"""Gate actuator requests; fallback is trim glide, not a flight termination."""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from mzj_interfaces.msg import ControlCommand
from mzj_system.validation import command_valid
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


class SystemNode(Node):
    def __init__(self):
        super().__init__("system")
        self.enabled = self.declare_parameter("enabled", True).value
        self.require_link = self.declare_parameter("require_mcu_link", False).value
        self.request = None
        self.request_time = 0.0
        self.state = None
        self.state_time = 0.0
        self.link_time = 0.0
        self.sequence = 0
        self.last_status = ""
        self.output = self.create_publisher(ControlCommand, "actuator/command", 1)
        self.status = self.create_publisher(String, "system/status", 1)
        self.request_sub = self.create_subscription(
            ControlCommand, "control/request", self.on_request, 1)
        self.state_sub = self.create_subscription(
            Odometry, "state/odom", self.on_state, 1)
        self.link_sub = self.create_subscription(
            Bool, "mcu/connected", self.on_link, 1)
        self.service = self.create_service(SetBool, "system/enable", self.set_enable)
        self.timer = self.create_timer(.02, self.tick)

    def on_request(self, message):
        self.request = message
        self.request_time = time.monotonic()

    def on_state(self, message):
        self.state = message
        self.state_time = time.monotonic()

    def on_link(self, message):
        if message.data:
            self.link_time = time.monotonic()
        else:
            self.link_time = 0.0

    def set_enable(self, request, response):
        self.enabled = request.data
        self.request = None  # Never reuse a command from before a mode change.
        response.success = True
        response.message = "enabled" if self.enabled else "trim glide"
        return response

    def tick(self):
        steady = time.monotonic()
        now = self.get_clock().now()
        seconds = now.nanoseconds / 1e9
        state_valid = False
        if self.state is not None:
            stamp = self.state.header.stamp
            age = seconds - (stamp.sec + stamp.nanosec * 1e-9)
            state_valid = (
                self.state.header.frame_id == "odom" and
                steady - self.state_time < .3 and 0 <= age < .3 and
                self.state.pose.pose.position.z > 0)
        if not self.enabled:
            reason = "DISABLED_TRIM"
        elif not state_valid:
            reason = "NO_AIRBORNE_STATE_TRIM"
        elif self.require_link and steady - self.link_time >= .5:
            reason = "MCU_OFFLINE_TRIM"
        elif (self.request is None or steady - self.request_time >= .2 or
              not command_valid(self.request, seconds)):
            reason = "COMMAND_EXPIRED_TRIM"
        else:
            reason = "ACTIVE"
        out = ControlCommand()
        out.header.stamp = now.to_msg()
        out.header.frame_id = "base_link"
        self.sequence = (self.sequence + 1) & 0xffffffff
        out.sequence = self.sequence
        out.valid_for = Duration(seconds=.15).to_msg()
        if reason == "ACTIVE":
            out.left_brake = self.request.left_brake
            out.right_brake = self.request.right_brake
        self.output.publish(out)
        if reason != self.last_status:
            self.get_logger().info(reason)
            self.last_status = reason
        self.status.publish(String(data=reason))


def main(args=None):
    rclpy.init(args=args)
    node = SystemNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
