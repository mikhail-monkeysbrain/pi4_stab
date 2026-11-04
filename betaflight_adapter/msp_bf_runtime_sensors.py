#!/usr/bin/env python3
"""Betaflight MSP IMU + ATTITUDE + real TF-Luna."""

import argparse
import atexit
import time

import serial

from msp_encoder import rangefinder_packet, opticalflow_packet, AngularMotionQuantizer
from bf_imu_ipc import BfImuPublisher
from bf_flow_ipc import BfFlowReceiver
from msp_bf_runtime import TfLunaParser
from msp_sensor_reader import (
    MspSensorReader,
    MSP_RAW_IMU,
    MSP_ATTITUDE,
)

IMU_PERIOD_NS = 10_000_000
ATT_PERIOD_NS = 50_000_000
RESPONSE_TIMEOUT_NS = 30_000_000


def run(luna_port, msp_port, baud):
    parser = TfLunaParser()
    ipc = BfImuPublisher()
    flow_ipc = BfFlowReceiver()
    atexit.register(flow_ipc.close)
    flow_ipc_rx = 0
    flow_ipc_last_ns = None

    flow_quantizer = AngularMotionQuantizer()
    flow_prev_ns = None
    flow_motion_x = 0
    flow_motion_y = 0
    flow_discontinuities = 0
    flow_duplicates = 0

    # Diagnostic 50 ms buckets; no UART OF transmission.
    flow_bucket_start_ns = None
    flow_bucket_dt_s = 0.0
    flow_bucket_x = 0
    flow_bucket_y = 0
    flow_bucket_count = 0
    flow_buckets_ready = 0
    flow_buckets_discarded = 0
    flow_msp_built = 0
    flow_msp_errors = 0
    flow_bucket_total_dt_s = 0.0
    flow_bucket_min_dt_s = float("inf")
    flow_bucket_max_dt_s = 0.0
    ipc_tx = 0
    ipc_fail = 0
    ipc_eagain = 0

    imu_rx = 0
    att_rx = 0
    range_tx = 0
    timeouts = 0

    pending = None
    request_ns = 0

    now = time.monotonic_ns()
    next_imu_ns = now
    next_att_ns = now
    last_report_ns = now

    with serial.Serial(
        luna_port, 115200, timeout=0
    ) as luna, serial.Serial(
        msp_port, baud,
        timeout=0,
        write_timeout=0.1,
    ) as fc:

        sensors = MspSensorReader(fc)

        print("JT-Zero BF sensor runtime")
        print("IMU target: 100 Hz")
        print("ATT target: 20 Hz")
        print("TF-Luna: real sensor", flush=True)

        while True:
            now = time.monotonic_ns()

            flow_samples = flow_ipc.poll()
            for flow_sample in flow_samples:
                timestamp_ns, dt_s, rx, ry, quality = flow_sample

                flow_ipc_rx += 1

                if flow_prev_ns is not None:
                    elapsed_ns = timestamp_ns - flow_prev_ns

                    if elapsed_ns <= 0:
                        flow_duplicates += 1
                        continue

                    expected_ns = dt_s * 1e9

                    if abs(elapsed_ns - expected_ns) > max(
                        5_000_000, expected_ns * 0.5
                    ):
                        flow_discontinuities += 1
                        if flow_bucket_count:
                            flow_buckets_discarded += 1
                        flow_bucket_start_ns = None
                        flow_bucket_dt_s = 0.0
                        flow_bucket_x = 0
                        flow_bucket_y = 0
                        flow_bucket_count = 0
                        flow_quantizer.reset()
                        flow_prev_ns = timestamp_ns
                        flow_ipc_last_ns = timestamp_ns
                        continue

                flow_prev_ns = timestamp_ns
                flow_ipc_last_ns = timestamp_ns

                mx, my = flow_quantizer.encode(rx, ry, dt_s)
                flow_motion_x += mx
                flow_motion_y += my

                if flow_bucket_start_ns is None:
                    flow_bucket_start_ns = timestamp_ns

                flow_bucket_x += mx
                flow_bucket_y += my
                flow_bucket_count += 1
                flow_bucket_dt_s += dt_s

                if flow_bucket_dt_s >= 0.05:
                    # Build MSPv2 in memory only. NEVER write OF to UART.
                    try:
                        packet = opticalflow_packet(
                            flow_bucket_x, flow_bucket_y, 255
                        )
                        if len(packet) != 18:
                            raise ValueError("Unexpected OF packet length")
                        flow_msp_built += 1
                        flow_bucket_total_dt_s += flow_bucket_dt_s
                        flow_bucket_min_dt_s = min(
                            flow_bucket_min_dt_s, flow_bucket_dt_s
                        )
                        flow_bucket_max_dt_s = max(
                            flow_bucket_max_dt_s, flow_bucket_dt_s
                        )
                    except (ValueError, TypeError, OverflowError):
                        flow_msp_errors += 1

                    flow_buckets_ready += 1
                    flow_bucket_start_ns = None
                    flow_bucket_dt_s = 0.0
                    flow_bucket_x = 0
                    flow_bucket_y = 0
                    flow_bucket_count = 0

            if pending is not None:
                if now - request_ns > RESPONSE_TIMEOUT_NS:
                    pending = None
                    timeouts += 1
                    sensors.buffer.clear()
                    fc.reset_input_buffer()

            sample = sensors.poll()

            if sample is not None:
                if pending == sample.command:
                    if sample.command == MSP_RAW_IMU:
                        imu_rx += 1
                    elif sample.command == MSP_ATTITUDE:
                        att_rx += 1

                    if ipc.publish(sample):
                        ipc_tx += 1
                    else:
                        ipc_fail += 1
                        if ipc.last_error in (11, 35):
                            ipc_eagain += 1
                    pending = None

            now = time.monotonic_ns()

            if pending is None:
                command = None

                if now >= next_att_ns:
                    command = MSP_ATTITUDE
                    next_att_ns = now + ATT_PERIOD_NS
                elif now >= next_imu_ns:
                    command = MSP_RAW_IMU

                if command is not None:
                    sensors.request(command)
                    pending = command
                    request_ns = time.monotonic_ns()

                    if command == MSP_RAW_IMU:
                        next_imu_ns = request_ns + IMU_PERIOD_NS

            available = luna.in_waiting

            if available:
                data = luna.read(min(available, 512))

                for distance_m, strength in parser.feed(data):
                    packet = rangefinder_packet(
                        distance_m,
                        quality=255,
                    )

                    written = fc.write(packet)

                    if written != len(packet):
                        raise IOError("Incomplete range write")

                    range_tx += 1

            now = time.monotonic_ns()

            if now - last_report_ns >= 1_000_000_000:
                print(
                    f"IMU_RX={imu_rx} "
                    f"ATT_RX={att_rx} "
                    f"RANGE_TX={range_tx} "
                    f"TIMEOUT={timeouts} "
                    f"IPC_TX={ipc_tx} "
                    f"IPC_FAIL={ipc_fail} "
                    f"IPC_EAGAIN={ipc_eagain} "
                    f"FLOW_IPC_RX={flow_ipc_rx} "
                    f"FLOW_MOTION_X={flow_motion_x} "
                    f"FLOW_MOTION_Y={flow_motion_y} "
                    f"FLOW_GAPS={flow_discontinuities} "
                    f"FLOW_DUP={flow_duplicates} "
                    f"FLOW_BUCKETS={flow_buckets_ready} "
                    f"FLOW_BUCKET_DROP={flow_buckets_discarded} "
                    f"FLOW_MSP_BUILT={flow_msp_built} "
                    f"FLOW_MSP_ERRORS={flow_msp_errors} "
                    f"FLOW_BUCKET_DT_AVG_MS="
                    f"{(1000.0 * flow_bucket_total_dt_s / flow_msp_built) if flow_msp_built else 0:.2f} "
                    f"FLOW_BUCKET_DT_MIN_MS="
                    f"{(1000.0 * flow_bucket_min_dt_s) if flow_msp_built else 0:.2f} "
                    f"FLOW_BUCKET_DT_MAX_MS="
                    f"{1000.0 * flow_bucket_max_dt_s:.2f} "
                    f"FLOW_IPC_AGE_MS="
                    f"{((now - flow_ipc_last_ns) / 1e6) if flow_ipc_last_ns is not None else -1:.1f}",
                    flush=True,
                )
                last_report_ns = now

            time.sleep(0.001)


if __name__ == "__main__":
    cli = argparse.ArgumentParser()
    cli.add_argument("--luna-port", default="/dev/serial0")
    cli.add_argument("--msp-port", default="/dev/ttyAMA4")
    cli.add_argument("--baud", type=int, default=115200)
    args = cli.parse_args()

    try:
        run(args.luna_port, args.msp_port, args.baud)
    except KeyboardInterrupt:
        print("\nStopped")
