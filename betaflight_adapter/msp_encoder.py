#!/usr/bin/env python3
"""Offline MSPv2 encoder for Betaflight MT optical-flow/rangefinder inputs.

No serial port is opened. No hardware commands are sent.
This is a protocol/packing building block, NOT a flight-ready bridge.
"""
import argparse
import math
import struct

MSP2_SENSOR_RANGEFINDER_LIDARMT = 0x1F01
MSP2_SENSOR_OPTICALFLOW_MT = 0x1F02


def crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0xD5 if crc & 0x80 else 0)) & 0xFF
    return crc


def msp_v2_frame(command: int, payload: bytes, flags: int = 0) -> bytes:
    if not 0 <= command <= 0xFFFF or not 0 <= flags <= 0xFF:
        raise ValueError('Invalid command or flags')
    if len(payload) > 0xFFFF:
        raise ValueError('Payload too long')
    body = struct.pack('<BHH', flags, command, len(payload)) + payload
    return b'$X<' + body + bytes([crc8_dvb_s2(body)])


def rangefinder_packet(distance_m: float, quality: int = 255) -> bytes:
    if not math.isfinite(distance_m) or not 0 <= distance_m <= 2147483.647:
        raise ValueError('Invalid distance')
    if not 0 <= quality <= 255:
        raise ValueError('Invalid quality')
    payload = struct.pack('<Bi', quality, round(distance_m * 1000))
    return msp_v2_frame(MSP2_SENSOR_RANGEFINDER_LIDARMT, payload)


def opticalflow_packet(motion_x: int, motion_y: int, quality: int = 255) -> bytes:
    if not 0 <= quality <= 255:
        raise ValueError('Invalid quality')
    if not all(-2147483648 <= v <= 2147483647 for v in (motion_x, motion_y)):
        raise ValueError('Motion outside int32 range')
    payload = struct.pack('<Bii', quality, motion_x, motion_y)
    return msp_v2_frame(MSP2_SENSOR_OPTICALFLOW_MT, payload)


class AngularMotionQuantizer:
    """Accumulate fractional MT motion counts to reduce quantization bias.

    Assumes externally verified axis orientation, raw/compensated flow semantics,
    and synchronized emission intervals. Do not connect to an FC without them.
    """
    def __init__(self):
        self.residual_x = 0.0
        self.residual_y = 0.0

    def encode(self, rate_x_rad_s: float, rate_y_rad_s: float, dt_s: float):
        if not all(map(math.isfinite, (rate_x_rad_s, rate_y_rad_s, dt_s))):
            raise ValueError('Non-finite flow input')
        if not 0 < dt_s < 1:
            raise ValueError('Invalid dt')
        counts_x = rate_x_rad_s * dt_s * 200.0 + self.residual_x
        counts_y = rate_y_rad_s * dt_s * 200.0 + self.residual_y
        def quantize(value):
            nearest = round(value)
            if abs(value - nearest) < 1e-10:
                value = float(nearest)
            return math.trunc(value)

        ix, iy = quantize(counts_x), quantize(counts_y)
        if not all(-2147483648 <= v <= 2147483647 for v in (ix, iy)):
            raise ValueError('Motion overflow')
        self.residual_x = counts_x - ix
        self.residual_y = counts_y - iy
        return ix, iy

    def reset(self):
        self.residual_x = self.residual_y = 0.0


def decode_frame(frame: bytes):
    if len(frame) < 9 or frame[:3] != b'$X<':
        raise ValueError('Invalid MSPv2 header')
    flags, command, size = struct.unpack_from('<BHH', frame, 3)
    if len(frame) != 9 + size:
        raise ValueError('Invalid frame length')
    if crc8_dvb_s2(frame[3:-1]) != frame[-1]:
        raise ValueError('Invalid CRC')
    return flags, command, frame[8:-1]


def self_test():
    rf = rangefinder_packet(1.234)
    of = opticalflow_packet(12, -34, 200)
    _, rf_id, rf_payload = decode_frame(rf)
    _, of_id, of_payload = decode_frame(of)
    assert rf_id == 0x1F01 and struct.unpack('<Bi', rf_payload) == (255, 1234)
    assert of_id == 0x1F02 and struct.unpack('<Bii', of_payload) == (200, 12, -34)
    assert len(rf) == 14 and len(of) == 18
    q = AngularMotionQuantizer()
    xs = [q.encode(0.1, 0, 0.02)[0] for _ in range(100)]
    assert sum(xs) == 40, sum(xs)
    try:
        decode_frame(of[:-1] + bytes([of[-1] ^ 1]))
        raise AssertionError('Corrupted CRC accepted')
    except ValueError:
        pass
    print('PASS: MSPv2 framing, CRC, payload, and fractional motion accumulation')
    print('RANGEFINDER:', rf.hex(' '))
    print('OPTICALFLOW:', of.hex(' '))
    print('NOTE: No UART access. Axis/gyro/timing compatibility NOT validated.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
    else:
        parser.print_help()
