"""Example protocol v1; all integers little-endian. No hardware standard implied."""
import binascii
import math
import struct

MAGIC = b"MZ"
HEADER = struct.Struct("<2sBBHI")
MAX_PAYLOAD = 32
COMMAND = 1
STATUS = 2


def encode(kind, sequence, payload):
    if kind not in (COMMAND, STATUS) or len(payload) > MAX_PAYLOAD:
        raise ValueError("invalid frame")
    frame = HEADER.pack(MAGIC, 1, kind, len(payload), sequence) + payload
    return frame + struct.pack("<H", binascii.crc_hqx(frame, 0xffff))


def encode_command(sequence, left, right, ttl_ms):
    if (not all(math.isfinite(v) and 0 <= v <= 1 for v in (left, right)) or
            not 1 <= ttl_ms <= 500):
        raise ValueError("invalid actuator command")
    payload = struct.pack("<HHH", round(left * 10000), round(right * 10000), ttl_ms)
    return encode(COMMAND, sequence, payload)


class Decoder:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        output = []
        while len(self.buffer) >= HEADER.size:
            if self.buffer[:2] != MAGIC:
                del self.buffer[0]
                continue
            _, version, kind, length, sequence = HEADER.unpack_from(self.buffer)
            if version != 1 or kind not in (COMMAND, STATUS) or length > MAX_PAYLOAD:
                del self.buffer[0]
                continue
            size = HEADER.size + length + 2
            if len(self.buffer) < size:
                break
            expected = struct.unpack_from("<H", self.buffer, size - 2)[0]
            actual = binascii.crc_hqx(self.buffer[:size - 2], 0xffff)
            if actual != expected:
                del self.buffer[0]
                continue
            output.append((kind, sequence, bytes(self.buffer[HEADER.size:size - 2])))
            del self.buffer[:size]
        return output
