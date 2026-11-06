#!/usr/bin/env python3
"""Grab one frame per camera topic and save it as PNG, to document and rebuild a trial scene.

Run it between trials only, with the arm idle: it subscribes to raw image topics for a
moment, and raw image subscribers during execution lag the arm. Using the policy cameras
gives the exact viewpoint the policy saw, so the scene can be rebuilt for a later eval.
Existing files are never overwritten.

    python3 trial_photo.py <trial_id> --tag before --out ~/eval/v1/photos \
        /realsense/d435/color/image_raw /camera/color/image_raw
    # -> ~/eval/v1/photos/<trial_id>_before_realsense_d435.png, <trial_id>_before_camera.png
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def slug(topic: str) -> str:
    # /realsense/d435/color/image_raw -> realsense_d435
    parts = [p for p in topic.strip("/").split("/") if p not in ("color", "image_raw", "image")]
    return "_".join(parts) or "image"


def to_bgr(msg: Image) -> np.ndarray:
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    if msg.encoding in ("rgb8", "bgr8"):
        img = img[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else img
    if msg.encoding == "mono8":
        return img[:, :msg.width]
    raise ValueError(f"unsupported encoding {msg.encoding}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("trial", help="trial id, used as file prefix")
    p.add_argument("topics", nargs="+", help="sensor_msgs/Image topics")
    p.add_argument("--tag", default="before", help="e.g. before / after")
    p.add_argument("--out", default=".", help="output directory (created if missing)")
    p.add_argument("--timeout", type=float, default=5.0, help="per-run wait for frames [s]")
    args = p.parse_args()

    out = os.path.expanduser(args.out)
    os.makedirs(out, exist_ok=True)
    paths = {t: os.path.join(out, f"{args.trial}_{args.tag}_{slug(t)}.png") for t in args.topics}
    taken = [path for path in paths.values() if os.path.exists(path)]
    if taken:
        sys.exit(f"refusing to overwrite: {', '.join(taken)}")

    rclpy.init()
    node = rclpy.create_node("trial_photo")
    frames = {}
    subs = [node.create_subscription(Image, t, lambda m, t=t: frames.setdefault(t, m),
                                     qos_profile_sensor_data) for t in args.topics]
    deadline = time.monotonic() + args.timeout
    while len(frames) < len(args.topics) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    del subs
    node.destroy_node()
    rclpy.shutdown()

    for t, path in paths.items():
        if t in frames:
            cv2.imwrite(path, to_bgr(frames[t]))
            print(f"saved {path}")
        else:
            print(f"NO FRAME from {t} within {args.timeout:.0f} s", file=sys.stderr)
    sys.exit(0 if len(frames) == len(args.topics) else 1)


if __name__ == "__main__":
    main()
