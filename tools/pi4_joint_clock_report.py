#!/usr/bin/env python3
"""Offline joint camera/FC timestamp analysis. No FC transmission.

Fits FC boot milliseconds to Pi CLOCK_MONOTONIC receive time. UART delivery
latency is unknown; this is NOT a validated IMU sample/exposure alignment.
"""
import argparse
import csv
from pathlib import Path
from statistics import median


def percentile(values, q):
    s = sorted(values)
    return s[round((len(s)-1)*q)] if s else float("nan")


def fit(points):
    x0, y0 = points[0]
    xs = [(x-x0)*1e-9 for x, _ in points]
    ys = [(y-y0)*1e-9 for _, y in points]
    xm = sum(xs)/len(xs)
    ym = sum(ys)/len(ys)
    den = sum((x-xm)**2 for x in xs)
    if den == 0:
        raise ValueError("no time span")
    slope = sum((x-xm)*(y-ym) for x, y in zip(xs, ys))/den
    intercept = ym-slope*xm
    residuals = [(y-intercept-slope*x)*1000 for x, y in zip(xs, ys)]
    return slope, residuals


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    with (args.directory/"camera_timestamps.csv").open(newline="") as f:
        camera = list(csv.DictReader(f))
    with (args.directory/"fc.csv").open(newline="") as f:
        fc = list(csv.DictReader(f))
    offsets = []
    latency = []
    cam_sensor = []
    cam_rx = []
    for row in camera:
        if not row.get("sensor_ts_ns"):
            continue
        sensor = int(row["sensor_ts_ns"])
        rx = int(row["recv_mono_ns"])
        steady = int(row["recv_steady_ns"])
        cam_sensor.append(sensor)
        cam_rx.append(rx)
        offsets.append((steady-rx)/1e6)
        latency.append((rx-sensor)/1e6)
    if not cam_sensor:
        raise SystemExit("ERROR: no valid camera sensor timestamps")
    print(f"CAMERA_CLOCK frames={len(cam_sensor)} "
          f"steady_minus_mono_median_ms={median(offsets):.6f} "
          f"steady_minus_mono_range_ms={max(offsets)-min(offsets):.6f} "
          f"sensor_to_rx_median_ms={median(latency):.3f} "
          f"sensor_to_rx_p95_ms={percentile(latency,.95):.3f} "
          f"sensor_to_rx_max_ms={max(latency):.3f} "
          f"sensor_nonmonotonic={sum(b<=a for a,b in zip(cam_sensor,cam_sensor[1:]))}")
    groups = {"ATTITUDE": [], "SCALED_IMU": []}
    raw = []
    for row in fc:
        kind = row.get("msg_type")
        try:
            rx = int(row["recv_mono_ns"])
            if kind in groups and row.get("fc_time_boot_ms"):
                groups[kind].append((rx, int(row["fc_time_boot_ms"])*1000000))
            if kind == "RAW_IMU" and row.get("fc_time_usec"):
                raw.append(int(row["fc_time_usec"]))
        except (ValueError, KeyError):
            continue
    for kind, points in groups.items():
        if len(points)<30:
            print(f"FC_CLOCK type={kind} insufficient_rows={len(points)}")
            continue
        slope, residual = fit(points)
        center = median(residual)
        mad = median(abs(x-center) for x in residual)
        threshold = max(5.0, 6*1.4826*mad)
        inliers = [p for p,r in zip(points,residual) if abs(r-center)<=threshold]
        robust, remaining = fit(inliers)
        print(f"FC_CLOCK type={kind} rows={len(points)} inliers={len(inliers)} "
              f"ols_ppm={(slope-1)*1e6:.1f} filtered_ppm={(robust-1)*1e6:.1f} "
              f"residual_abs_p95_ms={percentile([abs(x) for x in remaining],.95):.3f} "
              f"outliers={len(points)-len(inliers)}")
    if raw:
        print(f"RAW_IMU_CLOCK rows={len(raw)} first_usec={raw[0]} last_usec={raw[-1]} "
              f"nonmonotonic={sum(b<=a for a,b in zip(raw,raw[1:]))}")
    print("LIMITATION: FC clock fit uses UART reception times; camera sensor/IMU sample alignment is unverified.")


if __name__ == "__main__":
    main()
