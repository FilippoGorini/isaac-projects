#!/usr/bin/env python3
"""End-to-end test for the MoveIt Servo + pose_tracking pipeline.

Reads the current end-effector pose via TF at startup, then publishes a
continuous series of PoseStamped targets on /target_pose that oscillates along
a single base_link axis with a sinewave. EE orientation is held constant.

Default: ±0.20 m left/right (base_link y-axis), peak speed 0.15 m/s.

Usage (sim):
  ros2 run vla_kinova_tabletop servo_sine_test.py --ros-args -p use_sim_time:=true

Parameters (all optional):
  base_frame   : str   = 'base_link'
  ee_frame     : str   = 'end_effector_link'
  axis         : str   = 'y'    # one of 'x', 'y', 'z' (base_link axis to oscillate along)
  amplitude    : float = 0.20   # meters, peak displacement from anchor (±amplitude)
  speed        : float = 0.15   # m/s, peak linear speed (sets sinewave frequency)
  publish_rate : float = 30.0   # Hz
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


class ServoSineTest(Node):
    def __init__(self):
        super().__init__("servo_sine_test")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "end_effector_link")
        self.declare_parameter("axis", "y")
        self.declare_parameter("amplitude", 0.20)
        self.declare_parameter("speed", 0.15)
        self.declare_parameter("publish_rate", 30.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.axis = self.get_parameter("axis").value
        self.amplitude = self.get_parameter("amplitude").value
        self.peak_speed = self.get_parameter("speed").value
        self.rate_hz = self.get_parameter("publish_rate").value

        if self.axis not in AXIS_INDEX:
            self.get_logger().error(f"Unknown axis '{self.axis}', must be x/y/z.")
            raise SystemExit(2)

        # Peak speed of A*sin(omega*t) is A*omega, so omega = peak_speed / A.
        self.omega = self.peak_speed / self.amplitude
        self.period = 2.0 * math.pi / self.omega

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, "/target_pose", 10)

        self.anchor = None       # (x, y, z) at startup
        self.orientation = None
        self.start_time = None
        self.publish_timer = None

        self._init_timer = self.create_timer(0.1, self._try_init)
        self.get_logger().info(
            f"Servo sine test starting. Looking up '{self.base_frame}' -> '{self.ee_frame}'…"
        )

    def _try_init(self):
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, rclpy.time.Time())
        except Exception:
            return  # try again next tick — TF may not be ready yet

        self._init_timer.cancel()

        self.anchor = (t.transform.translation.x,
                       t.transform.translation.y,
                       t.transform.translation.z)
        self.orientation = t.transform.rotation
        self.start_time = self.get_clock().now()

        self.get_logger().info(
            f"Anchored EE: ({self.anchor[0]:.3f}, {self.anchor[1]:.3f}, {self.anchor[2]:.3f}). "
            f"Driving EE along {self.base_frame}.{self.axis} with amplitude={self.amplitude} m, "
            f"peak_speed={self.peak_speed} m/s, period={self.period:.2f} s."
        )
        self.publish_timer = self.create_timer(1.0 / self.rate_hz, self._publish)

    def _publish(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        offset = self.amplitude * math.sin(self.omega * elapsed)

        pos = list(self.anchor)
        pos[AXIS_INDEX[self.axis]] += offset

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation = self.orientation
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ServoSineTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
