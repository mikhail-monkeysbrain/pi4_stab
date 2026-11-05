#!/usr/bin/env python3
"""SmartAudio command encoding. No hardware access."""

from smartaudio_protocol import crc8


def set_frequency_packet(frequency_mhz):
    if type(frequency_mhz) is not int:
        raise TypeError("Frequency must be integer")

    if not 5000 <= frequency_mhz <= 6000:
        raise ValueError("Frequency outside 5000..6000 MHz")

    packet = bytes([
        0xAA, 0x55,
        0x09, 0x02,
        (frequency_mhz >> 8) & 0xFF,
        frequency_mhz & 0xFF,
    ])

    return packet + bytes([crc8(packet)])


if __name__ == "__main__":
    for frequency in (5865, 5845):
        packet = set_frequency_packet(frequency)
        print(f"{frequency} MHz: {packet.hex(' ').upper()}")
