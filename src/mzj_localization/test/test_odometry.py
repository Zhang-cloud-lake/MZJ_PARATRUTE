from nav_msgs.msg import Odometry
from mzj_localization.node import valid_odometry


def test_reject_invalid_frames_nan_and_unnormalized_attitude():
    message = Odometry()
    message.header.frame_id = "odom"
    message.child_frame_id = "base_link"
    assert valid_odometry(message)
    message.header.frame_id = "map"
    assert not valid_odometry(message)
    message.header.frame_id = "odom"
    message.pose.pose.position.x = float("nan")
    assert not valid_odometry(message)
    message.pose.pose.position.x = 0.
    message.pose.pose.orientation.w = 2.
    assert not valid_odometry(message)
