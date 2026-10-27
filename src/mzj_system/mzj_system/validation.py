"""Pure command validation shared with the communication bridge."""
import math


def command_valid(message, now_seconds):
    stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
    ttl = message.valid_for.sec + message.valid_for.nanosec * 1e-9
    return (
        message.header.frame_id == "base_link" and
        all(math.isfinite(v) and 0 <= v <= 1
            for v in (message.left_brake, message.right_brake)) and
        0 < ttl <= .5 and 0 <= now_seconds - stamp < ttl
    )
