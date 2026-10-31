#!/usr/bin/env python3
"""JT-Zero extended optical-flow MSPv2 payload."""

import math
import struct

PAYLOAD_FORMAT = "<BiiI"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FORMAT)

MIN_DT_US = 1
MAX_DT_US = 999_999
FLOW_SCALE = 200


def encode_payload(quality, motion_x, motion_y, integration_us):
    if not isinstance(quality, int) or not 0 <= quality <= 255:
        raise ValueError("quality must be uint8")

    for name, value in (("motion_x", motion_x), ("motion_y", motion_y)):
        if not isinstance(value, int) or not -(2**31) <= value < 2**31:
            raise ValueError(f"{name} must be int32")

    if not isinstance(integration_us, int):
        raise ValueError("integration_us must be integer")

    if not MIN_DT_US <= integration_us <= MAX_DT_US:
        raise ValueError("invalid integration interval")

    return struct.pack(
        PAYLOAD_FORMAT,
        quality,
        motion_x,
        motion_y,
        integration_us,
    )


def decode_payload(payload):
    if len(payload) != PAYLOAD_SIZE:
        raise ValueError("invalid payload length")

    quality, motion_x, motion_y, integration_us = struct.unpack(
        PAYLOAD_FORMAT, payload
    )

    if not MIN_DT_US <= integration_us <= MAX_DT_US:
        raise ValueError("invalid integration interval")

    return quality, motion_x, motion_y, integration_us


def flow_rates(payload):
    quality, motion_x, motion_y, integration_us = decode_payload(payload)

    dt = integration_us * 1e-6

    return (
        quality,
        motion_x / (FLOW_SCALE * dt),
        motion_y / (FLOW_SCALE * dt),
    )
