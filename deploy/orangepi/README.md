# Orange Pi Zero 2W deployment snapshot

Date: 2026-11-26/27
Branch: OrangePiZero2w-baro-height

## Status

Development snapshot, not a flight-validated release.

- Barometric virtual rangefinder and ARM zero changes:
  final validation pending.
- IRC Tramp V4: confirmed frequency and power read/write.
- VTX OSD DATA96 type 78: display confirmed.
- RC10/RC11: final integrated validation pending.

## VTX hardware

Foxeer Reaper Extreme, IRC Tramp.
Physical pin 15, PI5, UART2 TX.
Device: /dev/ttyS2.
Single-wire half-duplex.

## Build and install Tramp V4

mkdir -p build
g++ -std=c++17 -O2 tools/tramp_orangepi_v4.cpp -o build/tramp_orangepi_v4
sudo install -o root -g root -m 0755 build/tramp_orangepi_v4 /usr/local/sbin/monkeysstab-tramp-orangepi

## OSD sender

Install tools/jtzero_osd_sender_v2.py to:
 /home/vio/jtzero_osd_sender_v2.py

Expected Python:
 /home/vio/mavlink-test-venv/bin/python

On the original Orange Pi this environment is provided
by uart-test-venv with pymavlink installed.

VTX state:
 /home/vio/jtzero_vtx_state.json

MAVLink endpoint:
 tcp:127.0.0.1:5760

## Configuration

Review monkeys_orangepi.env.example before installing.
Set MONKEYS_VTX_PROTOCOL=TRAMP.

Review boot configuration before installing.
Preserve the destination root filesystem UUID.
Required overlays: pi-uart4 ph-uart5 pi-uart2.
Custom overlay: apb2-75mhz.

## Services

monkeys-router.service
monkeys-web.service

The web service manages the flight runtime.
Do not start a second independent runtime service.

## Wi-Fi

wifi-dom and wifi-flight use existing NetworkManager profiles.
Replace profile UUIDs on a different installation.
Wi-Fi passwords are not stored in this repository.

## Experimental archive

archive/orangepi/20261126 contains historical source backups
and optical-flow runtime configurations.
These are not production configuration files.
