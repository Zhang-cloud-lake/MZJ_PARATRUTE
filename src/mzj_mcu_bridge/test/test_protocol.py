import struct
import pytest
from mzj_mcu_bridge.protocol import Decoder, encode, encode_command, STATUS


def test_fragmentation_noise_crc_and_back_to_back():
    good = encode_command(42, .2, .8, 100)
    bad = bytearray(good)
    bad[-1] ^= 0xff
    decoder = Decoder()
    assert decoder.feed(b"noise" + bad + good[:5]) == []
    result = decoder.feed(good[5:] + good)
    assert len(result) == 2
    assert result[0][:2] == (1, 42)
    assert struct.unpack("<HHH", result[0][2]) == (2000, 8000, 100)


def test_status_and_bounded_decoder():
    payload = struct.pack("<BHH", 0, 1000, 2000)
    assert Decoder().feed(encode(STATUS, 7, payload)) == [(STATUS, 7, payload)]
    decoder = Decoder()
    decoder.feed(b"x" * 10000)
    assert len(decoder.buffer) < 10


@pytest.mark.parametrize("left,right,ttl", [(float("nan"), 0, 100), (-1, 0, 100),
                                          (0, 2, 100), (0, 0, 0), (0, 0, 501)])
def test_invalid_commands(left, right, ttl):
    with pytest.raises(ValueError):
        encode_command(1, left, right, ttl)
