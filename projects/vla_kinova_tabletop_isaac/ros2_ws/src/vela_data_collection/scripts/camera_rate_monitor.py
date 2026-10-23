#!/usr/bin/env python3
"""Live camera rate monitor: prints per-window rate and gaps for each camera.

Subscribes to camera_info, not image_raw: the drivers publish one per frame with
the same header stamp, so the rate is identical, but it doesn't add another full
copy of every raw frame to DDS (that made teleop lag). Measures the camera side;
delivery drops to the recorder show up as its 'Frame gap' / 'has no data' warnings.
Gaps = intervals > 1.5x the median.

    python3 camera_rate_monitor.py                 # wrist + both externals, 5 s windows
    python3 camera_rate_monitor.py --window 10 /realsense/d435/color/camera_info
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

DEFAULT_TOPICS = [
    "/camera/color/camera_info",
    "/realsense/d435/color/camera_info",
    "/realsense/d435_2/color/camera_info",
]


def summary(ts: list) -> str:
    if len(ts) < 2:
        return "  NO DATA"
    d = np.diff(ts)
    gaps = int((d > 1.5 * np.median(d)).sum())
    return f"{len(d) / (ts[-1] - ts[0]):6.2f} Hz  max {d.max() * 1e3:6.1f} ms  gaps {gaps:3d}"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("topics", nargs="*", default=DEFAULT_TOPICS)
    p.add_argument("--window", type=float, default=5.0, help="report period [s]")
    args = p.parse_args()

    rclpy.init()
    node = rclpy.create_node("camera_rate_monitor")
    window = {t: [] for t in args.topics}
    total = {t: [] for t in args.topics}

    def cb(topic):
        def f(msg):
            s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            window[topic].append(s)
            total[topic].append(s)
        return f

    for t in args.topics:
        node.create_subscription(CameraInfo, t, cb(t), qos_profile_sensor_data)

    w = max(len(t) for t in args.topics)
    next_report = time.monotonic() + args.window
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() < next_report:
                continue
            next_report += args.window
            print(time.strftime("%H:%M:%S"))
            for t in args.topics:
                print(f"  {t:{w}s} {summary(window[t])}")
                window[t] = window[t][-1:]  # keep last stamp so window intervals chain
    except KeyboardInterrupt:
        pass
    print("TOTAL")
    for t in args.topics:
        print(f"  {t:{w}s} {summary(total[t])}")


if __name__ == "__main__":
    main()
