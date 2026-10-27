#!/usr/bin/env python3
"""Run the default flight in an isolated ROS domain; no physical device access."""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from mzj_interfaces.msg import ControlCommand, SemanticDetections, Trajectory
from nav_msgs.msg import Odometry
from std_msgs.msg import String, UInt8MultiArray
from std_srvs.srv import SetBool


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain", type=int, default=81)
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    root = Path(__file__).resolve().parents[1]
    log_path = root / "log" / "smoke_test.log"
    log_path.parent.mkdir(exist_ok=True)
    log = log_path.open("w")
    process = subprocess.Popen(
        ["ros2", "launch", "mzj_bringup", "system.launch.py"],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    node = None
    try:
        rclpy.init()
        node = rclpy.create_node("mzj_smoke_probe")
        observed = {"odom": None, "vision": False, "plan": False,
                    "frame": False, "command": None, "active": False, "turn": False}

        def odom(message):
            observed["odom"] = message

        def vision(message):
            observed["vision"] |= any(
                d.class_name == "demo_green_marker" for d in message.detections)

        def plan(message):
            observed["plan"] |= message.feasible and len(message.points) > 1

        def frame(message):
            observed["frame"] |= bytes(message.data[:2]) == b"MZ"

        def command(message):
            assert 0 <= message.left_brake <= 1 and 0 <= message.right_brake <= 1
            observed["command"] = message
            observed["turn"] |= max(message.left_brake, message.right_brake) > .01

        def status(message):
            observed["active"] |= message.data == "ACTIVE"

        subscriptions = [
            node.create_subscription(Odometry, "state/odom", odom, 1),
            node.create_subscription(SemanticDetections, "perception/detections", vision, 1),
            node.create_subscription(Trajectory, "planning/trajectory", plan, 1),
            node.create_subscription(UInt8MultiArray, "mcu/tx_frame", frame, 1),
            node.create_subscription(ControlCommand, "actuator/command", command, 1),
            node.create_subscription(String, "system/status", status, 1),
        ]
        assert len(subscriptions) == 6
        enable = node.create_client(SetBool, "system/enable")
        start = time.monotonic()
        disabled = None
        reenabled = None
        gated = False
        landed = False
        while time.monotonic() - start < 35:
            assert process.poll() is None, "launch exited; see log"
            rclpy.spin_once(node, timeout_sec=.05)
            elapsed = time.monotonic() - start
            if elapsed > 6 and disabled is None and enable.service_is_ready():
                disabled = enable.call_async(SetBool.Request(data=False))
            if disabled is not None and disabled.done():
                assert disabled.result().success
                if 6.4 < elapsed < 6.9:
                    out = observed["command"]
                    gated |= out is not None and out.left_brake == 0 and out.right_brake == 0
                if elapsed >= 7 and reenabled is None:
                    reenabled = enable.call_async(SetBool.Request(data=True))
            state = observed["odom"]
            if state is not None and state.pose.pose.position.z == 0:
                landed = True
                break
        assert landed, "simulation did not touch down within 35 seconds"
        assert gated and reenabled is not None and reenabled.done(), "enable gate failed"
        assert reenabled.result().success
        assert all(observed[k] for k in ("vision", "plan", "frame", "active", "turn")), observed
        state = observed["odom"]
        assert state.header.frame_id == "odom" and state.child_frame_id == "base_link"
        p = state.pose.pose.position
        error = math.hypot(p.x - 120, p.y - 30)
        assert error < 8, f"landing error {error:.3f} m"
        print(f"PASS: landing=({p.x:.3f}, {p.y:.3f}), error={error:.3f} m; "
              "vision, trajectory, command gate and dry-run frames verified", flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process.poll() is None:
            # Launch propagates SIGINT to its children; signaling the process
            # group as well would interrupt their cleanup a second time.
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        log.close()
        print(f"Launch log: {log_path}", flush=True)
    contents = log_path.read_text()
    assert "Traceback" not in contents and "process has died" not in contents, contents[-4000:]


if __name__ == "__main__":
    main()
