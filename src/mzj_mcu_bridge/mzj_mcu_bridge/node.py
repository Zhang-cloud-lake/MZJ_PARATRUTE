"""Nonblocking POSIX serial adapter. Dry-run never opens a device."""
import errno
import os
import struct
import termios
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from mzj_interfaces.msg import ControlCommand
from mzj_mcu_bridge.protocol import Decoder, STATUS, encode_command
from mzj_system.validation import command_valid
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt8MultiArray


class BridgeNode(Node):
    def __init__(self):
        super().__init__("mcu_bridge")
        self.dry_run = self.declare_parameter("dry_run", True).value
        self.device = self.declare_parameter("device", "").value
        self.baud = self.declare_parameter("baud", 115200).value
        self.fd = None
        self.pending = b""
        self.pending_since = 0.0
        self.decoder = Decoder()
        self.command = None
        self.received = 0.0
        self.feedback_time = 0.0
        self.last_ack = None
        self.wire_sequence = 0
        self.recent_sequences = {}
        self.frames = self.create_publisher(UInt8MultiArray, "mcu/tx_frame", 1)
        self.connected = self.create_publisher(Bool, "mcu/connected", 1)
        self.status = self.create_publisher(String, "mcu/status", 1)
        self.subscription = self.create_subscription(
            ControlCommand, "actuator/command", self.receive, 1)
        if not self.dry_run:
            if not self.device:
                raise ValueError("serial device required when dry_run=false")
            if self.baud != 115200:
                raise ValueError("example transport currently supports 115200 baud")
            self.fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0
            attrs[4] = termios.B115200
            attrs[5] = termios.B115200
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        self.timer = self.create_timer(.02, self.tick)
        self.get_logger().info("dry-run" if self.dry_run else f"serial: {self.device}")

    def receive(self, message):
        self.command = message
        self.received = time.monotonic()

    def tick(self):
        now = time.monotonic()
        seconds = self.get_clock().now().nanoseconds / 1e9
        left = right = 0.
        ttl_ms = 100
        if (self.command is not None and now - self.received < .2 and
                command_valid(self.command, seconds)):
            left, right = self.command.left_brake, self.command.right_brake
            stamp = self.command.header.stamp
            duration = self.command.valid_for
            expiry = (stamp.sec + stamp.nanosec * 1e-9 +
                      duration.sec + duration.nanosec * 1e-9)
            ttl_ms = max(1, min(500, int((expiry - seconds) * 1000)))
        self.wire_sequence = (self.wire_sequence + 1) & 0xffffffff
        frame = encode_command(self.wire_sequence, left, right, ttl_ms)
        self.frames.publish(UInt8MultiArray(data=list(frame)))
        self.recent_sequences = {
            seq: sent for seq, sent in self.recent_sequences.items()
            if now - sent < .5}
        if self.fd is not None:
            try:
                if self.pending and now - self.pending_since > .05:
                    raise OSError(errno.ETIMEDOUT, "serial write stalled")
                if not self.pending:
                    self.pending = frame
                    self.pending_since = now
                    self.recent_sequences[self.wire_sequence] = now
                try:
                    written = os.write(self.fd, self.pending)
                    self.pending = self.pending[written:]
                except BlockingIOError:
                    pass
                try:
                    incoming = os.read(self.fd, 1024)
                except BlockingIOError:
                    incoming = b""
                for kind, sequence, payload in self.decoder.feed(incoming):
                    if (kind == STATUS and len(payload) == 5 and
                            sequence in self.recent_sequences and sequence != self.last_ack):
                        flags, actual_left, actual_right = struct.unpack("<BHH", payload)
                        if flags == 0 and actual_left <= 10000 and actual_right <= 10000:
                            self.feedback_time = now
                            self.last_ack = sequence
                        else:
                            self.feedback_time = 0.
            except OSError as error:
                self.get_logger().error(f"Serial disabled after I/O error: {error}")
                os.close(self.fd)
                self.fd = None
                self.feedback_time = 0.
        linked = self.fd is not None and now - self.feedback_time < .5
        self.connected.publish(Bool(data=linked))
        mode = "DRY_RUN" if self.dry_run else ("CONNECTED" if linked else "OFFLINE")
        self.status.publish(String(data=mode))

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
