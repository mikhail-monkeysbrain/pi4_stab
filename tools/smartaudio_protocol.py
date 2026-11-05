#!/usr/bin/env python3
"""SmartAudio V2.1 response decoder. No hardware access."""

def crc8(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ (0xD5 if crc & 0x80 else 0)) & 0xFF
    return crc


def decode_settings(raw):
    if len(raw) < 5 or raw[:2] != b'\xAA\x55':
        raise ValueError("Invalid SmartAudio header")

    command = raw[2]
    length = raw[3]
    total = 4 + length

    if length < 1 or len(raw) < total:
        raise ValueError("Incomplete SmartAudio frame")

    frame = raw[:total]
    if crc8(frame[2:-1]) != frame[-1]:
        raise ValueError("SmartAudio CRC mismatch")

    if command != 0x11:
        raise ValueError(f"Unexpected command: {command:02X}")

    payload = frame[4:-1]

    if len(payload) < 8:
        raise ValueError("Short V2.1 settings payload")

    frequency = int.from_bytes(payload[3:5], "big")
    count = payload[6]

    if count > 16 or len(payload) != 8 + count:
        raise ValueError("Invalid power table length")

    power_dbm = list(payload[8:8 + count])

    if not 5000 <= frequency <= 6000:
        raise ValueError("Unexpected frequency")

    return {
        "ok": True,
        "protocol": "SMARTAUDIO_V21",
        "transport": "GPIO12_ONEWIRE",
        "frequency_mhz": frequency,
        "channel_raw": payload[0],
        "power_raw": payload[1],
        "mode_raw": payload[2],
        "current_power_dbm": payload[5],
        "power_levels_dbm": power_dbm,
        "power_level_count": count,
        "raw_hex": frame.hex(" ").upper(),
        "crc_ok": True,
    }


if __name__ == "__main__":
    import json

    raw = bytes.fromhex(
        "AA 55 11 0D 00 00 10 16 E9 0E 04 00 "
        "0E 1B 1E 22 DA 00 00"
    )
    print(json.dumps(decode_settings(raw), indent=2))
