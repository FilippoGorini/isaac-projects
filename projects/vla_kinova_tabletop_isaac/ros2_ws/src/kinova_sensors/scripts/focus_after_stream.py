#!/usr/bin/env python3
"""Set the fixed colour focus once the colour stream is up (kinova_vision may need several connection attempts, and
starting the stream could reset the sensor settings). Started by wrist_camera.launch.py.

    ros2 run kinova_sensors focus_after_stream.py <robot_ip> <focus value>
"""
import os
import subprocess
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

ip, value = sys.argv[1], sys.argv[2]
rclpy.init()
node = rclpy.create_node("focus_after_stream")
got = []
node.create_subscription(Image, "/camera/color/image_raw", lambda m: got.append(1), qos_profile_sensor_data)
t0 = time.time()
while not got and time.time() - t0 < 300:
    rclpy.spin_once(node, timeout_sec=0.5)
if not got:
    sys.exit("focus_after_stream: no colour image within 300 s, focus NOT set")
time.sleep(2.0)  # let the stream settle
r = subprocess.run([os.path.join(os.path.dirname(os.path.abspath(__file__)), "vision_focus"), ip, "manual", value])
sys.exit(r.returncode)
