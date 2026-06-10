#!/usr/bin/env python3
"""Stream a slow sine POSITION trajectory to the gripper.

Publishes a sine position on /gripper_position_controller/commands at 50 Hz
(a std_msgs/Float64MultiArray of length 1, position in RADIANS), so you can
watch how the real gripper tracks a streamed position reference.

This is the gripper analog of the VLA deployment path: arm on the
joint_trajectory_controller (LOW_LEVEL servoing) so the gripper position rides
the 1 kHz cyclic frame alongside the arm. The arm just holds still here.

Prereqs (real robot):
  1. ros2 launch vla_kinova_tabletop kinova_controllers.launch.py use_sim:=false
     (JTC active -> arm in LOW_LEVEL; gripper_position_controller loaded inactive)
  2. ros2 control switch_controllers --strict \
       --activate gripper_position_controller \
       --deactivate robotiq_gripper_controller
  3. ros2 run vla_kinova_tabletop gripper_sine_test.py

Parameters (all optional):
  center       : float = 0.40   # rad, midpoint of the sweep (0 open .. 0.81 closed)
  amplitude    : float = 0.30   # rad, peak deviation from center (stays within [0, 0.81])
  period       : float = 6.0    # s, one full open/close cycle
  publish_rate : float = 50.0   # Hz
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

GRIPPER_MAX_RAD = 0.81  # robotiq_2f_85 closed position


class GripperSineTest(Node):
    def __init__(self):
        super().__init__("gripper_sine_test")

        self.declare_parameter("center", 0.40)
        self.declare_parameter("amplitude", 0.30)
        self.declare_parameter("period", 6.0)
        self.declare_parameter("publish_rate", 50.0)

        self.center = float(self.get_parameter("center").value)
        self.amplitude = float(self.get_parameter("amplitude").value)
        self.period = float(self.get_parameter("period").value)
        self.rate_hz = float(self.get_parameter("publish_rate").value)

        # Keep the whole sweep inside the mechanical range [0, 0.81] rad.
        lo = self.center - self.amplitude
        hi = self.center + self.amplitude
        if lo < 0.0 or hi > GRIPPER_MAX_RAD:
            self.get_logger().error(
                f"Sweep [{lo:.3f}, {hi:.3f}] rad is outside [0, {GRIPPER_MAX_RAD}]. "
                "Lower center/amplitude.")
            raise SystemExit(2)

        self.omega = 2.0 * math.pi / self.period

        self.pub = self.create_publisher(
            Float64MultiArray, "/gripper_position_controller/commands", 10)

        self.start_time = self.get_clock().now()
        self.create_timer(1.0 / self.rate_hz, self._publish)

        self.get_logger().info(
            f"Gripper sine test @ {self.rate_hz:.0f} Hz: center={self.center:.3f} rad, "
            f"amplitude={self.amplitude:.3f} rad, period={self.period:.1f} s "
            f"(sweep [{lo:.3f}, {hi:.3f}] rad). Publishing /gripper_position_controller/commands.")

    def _publish(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        cmd = self.center + self.amplitude * math.sin(self.omega * elapsed)
        self.pub.publish(Float64MultiArray(data=[float(cmd)]))


def main():
    rclpy.init()
    node = GripperSineTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
