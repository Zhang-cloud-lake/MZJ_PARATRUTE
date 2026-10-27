from types import SimpleNamespace as NS
import pytest
from mzj_system.validation import command_valid


def command(left=0., right=0., ttl=100000000, frame="base_link"):
    return NS(header=NS(frame_id=frame, stamp=NS(sec=10, nanosec=0)),
              valid_for=NS(sec=0, nanosec=ttl), left_brake=left, right_brake=right)


def test_age_boundaries():
    assert command_valid(command(), 10.05)
    assert not command_valid(command(), 9.99)
    assert not command_valid(command(), 10.11)


@pytest.mark.parametrize("message", [
    command(left=float("nan")), command(right=1.1), command(ttl=0),
    command(ttl=600000000), command(frame="odom")])
def test_malformed_command(message):
    assert not command_valid(message, 10.01)
