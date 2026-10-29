#!/usr/bin/env python3

"""
Kortex twist adapter: turns the teleop tracker's generic twist into what the Kinova firmware expects.

Subscribes to geometry_msgs/TwistStamped (the velocity of control_frame, expressed in the header's
frame, e.g. base_link) from vela_teleop's twist_pose_tracking_node, and publishes geometry_msgs/Twist
on the picknik_twist_controller command topic. kortex_driver interprets that twist in
CARTESIAN_REFERENCE_FRAME_TOOL (end_effector_link), so the twist is rotated into the tool frame.

When control_frame is offset from end_effector_link (fingertips_frame is a pure +z translation of
it), the firmware still actuates end_effector_link, so the linear velocity is corrected for the
lever arm: the angular velocity is unchanged but v_ee = v_control - w x t, where t is the control
frame origin expressed in EE axes (looked up once from TF).

Watchdog: the Kortex twist controller keeps executing the last twist until it gets a zero. The
tracker publishes every tick (zeros when idle), so if nothing arrives for input_timeout_s (tracker
killed / hung) this node commands zero twist.
"""


import signal
import numpy as np
import rclpy
import transforms3d.quaternions as t3dq
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, ConnectivityException


class KortexTwistAdapter(Node):
    def __init__(self):
        super().__init__("kortex_twist_adapter")

        # ee_frame is the firmware tool frame that kortex_driver actuates
        self.ee_frame = self._declare("ee_frame", "end_effector_link")
        # control_frame is the frame whose velocity the incoming twist describes (the tracker's PID frame)
        self.control_frame = self._declare("control_frame", "end_effector_link")
        twist_topic = self._declare("twist_topic", "/teleop_twist")
        kortex_twist_topic = self._declare("kortex_twist_topic", "/twist_controller/commands")
        self.input_timeout_s = self._declare("input_timeout_s", 0.1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.twist_sub = self.create_subscription(TwistStamped, twist_topic, self._twist_callback, 1)
        self.twist_pub = self.create_publisher(Twist, kortex_twist_topic, 1)

        self.control_offset = None
        self.last_input = None
        self.timed_out = False
        self.timer = self.create_timer(0.02, self._watchdog)

        self.get_logger().info(
            f"kortex_twist_adapter ready. Subscribing: {twist_topic} (TwistStamped) | "
            f"Publishing: {kortex_twist_topic} (Twist in '{self.ee_frame}') | "
            f"watchdog {self.input_timeout_s * 1e3:.0f} ms"
        )

    def _declare(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _control_offset(self):
        """Lever arm t = control_frame origin in ee_frame axes, or None if the
        control frame is the firmware tool frame (no correction needed). The
        transform is static, so it is looked up once and cached."""
        if self.control_frame == self.ee_frame:
            return None
        if self.control_offset is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.ee_frame, self.control_frame, Time())
            except (LookupException, ExtrapolationException, ConnectivityException) as e:
                self.get_logger().warn(
                    f"control-frame offset lookup failed ({self.ee_frame} -> {self.control_frame}): {e}; "
                    f"commanding twist without lever-arm correction",
                    throttle_duration_sec=5.0,
                )
                return None
            self.control_offset = np.array([
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
            ])
            self.get_logger().info(
                f"Control frame '{self.control_frame}' has lever arm "
                f"t={self.control_offset.tolist()} m in '{self.ee_frame}' axes."
            )
        return self.control_offset

    def _twist_callback(self, msg: TwistStamped):
        self.last_input = self.get_clock().now()
        if self.timed_out:
            self.get_logger().info("Twist input is back.")
            self.timed_out = False
        v = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        w = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
        if not v.any() and not w.any():
            self.twist_pub.publish(Twist())
            return

        # Current tool orientation in the twist's frame (latest available, not exact-time)
        try:
            tf_msg = self.tf_buffer.lookup_transform(msg.header.frame_id, self.ee_frame, Time())
        except (LookupException, ExtrapolationException, ConnectivityException) as e:
            self.get_logger().warn(f"TF lookup failed ({msg.header.frame_id} -> {self.ee_frame}): {e}",
                                   throttle_duration_sec=1.0)
            self.twist_pub.publish(Twist())
            return

        # Rotate the twist into the tool frame
        # R_base_to_tool = R_ee_from_base = (R_base_from_ee)^T
        # transforms3d uses scalar-first [w, x, y, z] order (ROS messages are [x, y, z, w])
        q_ee = [tf_msg.transform.rotation.w, tf_msg.transform.rotation.x,
                tf_msg.transform.rotation.y, tf_msg.transform.rotation.z]
        R_base_to_tool = t3dq.quat2mat(q_ee).T
        v_tool = R_base_to_tool @ v
        w_tool = R_base_to_tool @ w

        # The twist describes the control frame, but kortex_driver actuates the EE origin.
        # In EE axes the control origin sits at +t, so
        # v_ee = v_control + w x (p_ee - p_control) = v_tool - w_tool x t.
        offset = self._control_offset()
        if offset is not None:
            v_tool = v_tool - np.cross(w_tool, offset)

        out = Twist()
        out.linear.x, out.linear.y, out.linear.z = float(v_tool[0]), float(v_tool[1]), float(v_tool[2])
        out.angular.x, out.angular.y, out.angular.z = float(w_tool[0]), float(w_tool[1]), float(w_tool[2])
        self.twist_pub.publish(out)

    def _watchdog(self):
        if self.last_input is None:
            return
        age = (self.get_clock().now() - self.last_input).nanoseconds * 1e-9
        if age > self.input_timeout_s:
            if not self.timed_out:
                self.get_logger().warn(f"No twist input for {age * 1e3:.0f} ms, commanding zero twist.")
                self.timed_out = True
            self.twist_pub.publish(Twist())


def main():
    rclpy.init()
    node = KortexTwistAdapter()

    def _shutdown(signum=None, frame=None):
        try:
            node.twist_pub.publish(Twist())
        except Exception:
            pass
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    finally:
        try:
            node.twist_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
