#!/usr/bin/env python3
"""Exercise the actual POSIX transport against a pseudo-terminal MCU."""
import argparse
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import time

import rclpy
from mzj_interfaces.msg import ControlCommand
from mzj_mcu_bridge.protocol import COMMAND, STATUS, Decoder, encode
from rclpy.duration import Duration
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain", type=int, default=82)
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    device = os.ttyname(slave)
    root = Path(__file__).resolve().parents[1]
    log_path = root / "log" / "serial_integration.log"
    log_path.parent.mkdir(exist_ok=True)
    log = log_path.open("w")
    process = subprocess.Popen([
        "ros2", "run", "mzj_mcu_bridge", "mcu_bridge", "--ros-args",
        "-p", "dry_run:=false", "-p", f"device:={device}"],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    node = None
    try:
        rclpy.init()
        node = rclpy.create_node("mzj_serial_probe")
        publisher = node.create_publisher(ControlCommand, "actuator/command", 1)
        seen = {"linked": False, "active": False, "expired": False, "invalid": False}
        subscription = node.create_subscription(
            Bool, "mcu/connected", lambda m: seen.update(linked=seen["linked"] or m.data), 1)
        assert subscription is not None
        decoder = Decoder()
        start = time.monotonic()
        sequence = 0
        while time.monotonic() - start < 5:
            assert process.poll() is None, "bridge exited"
            elapsed = time.monotonic() - start
            if elapsed < 2 or elapsed >= 3.5:
                command = ControlCommand()
                command.header.stamp = node.get_clock().now().to_msg()
                command.header.frame_id = "base_link"
                command.valid_for = Duration(seconds=.15).to_msg()
                sequence += 1
                command.sequence = sequence
                command.left_brake = .3 if elapsed < 2 else float("nan")
                command.right_brake = .1
                publisher.publish(command)
            try:
                data = os.read(master, 4096)
            except BlockingIOError:
                data = b""
            for kind, frame_sequence, payload in decoder.feed(data):
                if kind != COMMAND:
                    continue
                left, right, ttl = struct.unpack("<HHH", payload)
                assert 1 <= ttl <= 500
                if elapsed < 2:
                    seen["active"] |= (left, right) == (3000, 1000)
                elif 2.5 < elapsed < 3.5:
                    assert (left, right) == (0, 0), "expired command was executed"
                    seen["expired"] = True
                elif elapsed > 4:
                    assert (left, right) == (0, 0), "NaN command was executed"
                    seen["invalid"] = True
                response = encode(STATUS, frame_sequence, struct.pack("<BHH", 0, left, right))
                os.write(master, response)
            rclpy.spin_once(node, timeout_sec=.02)
        assert all(seen.values()), seen
        print("PASS: pseudo-MCU command, CRC feedback, stale-command trim and NaN rejection",
              flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        os.close(master)
        os.close(slave)
        log.close()
        print(f"Serial log: {log_path}", flush=True)


if __name__ == "__main__":
    main()
