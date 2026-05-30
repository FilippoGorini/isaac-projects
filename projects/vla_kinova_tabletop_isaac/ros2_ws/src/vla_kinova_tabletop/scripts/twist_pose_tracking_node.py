#!/usr/bin/env python3

"""
Twist-based pose tracking node for the real Kinova Gen3 6/7 DoF.
Subscribes to a target PoseStamped on /target_frame, looks up the current EE pose from TF, runs
a per-axis PID on the 6D error in base_link, saturates the twist, then rotates
it into the tool frame (end_effector_link) and publishes geometry_msgs/Twist
on the picknik_twist_controller command topic. kortex_driver interprets the
twist in CARTESIAN_REFERENCE_FRAME_TOOL, so the rotation into the EE frame is
mandatory
"""


import signal
import numpy as np
import rclpy
import transforms3d.quaternions as t3dq
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, ConnectivityException


# Dynamic parameters: read from the parameter store every tick so that you can use ros2 param set to update them while running the node
_DYNAMIC_PARAMS = {
    "target_pose_timeout_s": 0.5,
    "max_linear_velocity": 0.10,
    "max_angular_velocity": 0.40,
    "linear_windup_limit": 0.05,
    "angular_windup_limit": 0.05,
    "integrator_leak_rate": 5.0,
    # Low defaults, we pass the real ones trough the yaml config at launch time
    "x_proportional_gain": 1.0,
    "y_proportional_gain": 1.0,
    "z_proportional_gain": 1.0,
    "x_integral_gain": 0.2,
    "y_integral_gain": 0.2,
    "z_integral_gain": 0.2,
    "x_derivative_gain": 0.0,
    "y_derivative_gain": 0.0,
    "z_derivative_gain": 0.0,
    "angular_proportional_gain": 1.0,
    "angular_integral_gain": 0.2,
    "angular_derivative_gain": 0.0,
}


class TwistPoseTrackingNode(Node):
    def __init__(self):
        super().__init__("twist_pose_tracking_node")

        # Static parameters
        self.loop_rate_hz = self._declare("loop_rate_hz", 100.0)
        self.base_frame = self._declare("base_frame", "base_link")
        self.ee_frame = self._declare("ee_frame", "end_effector_link")
        target_topic = self._declare("target_topic", "/target_frame")
        twist_command_topic = self._declare("twist_command_topic", "/twist_controller/commands")

        # Dynamic parameters
        for name, default in _DYNAMIC_PARAMS.items():
            self.declare_parameter(name, default)

        # ROS interfaces
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_sub = self.create_subscription(
            PoseStamped, target_topic, self._target_callback, qos_profile_sensor_data
        )
        self.twist_pub = self.create_publisher(Twist, twist_command_topic, 10)

        # State
        self.latest_target = None
        self.latest_target_stamp = None
        self.integral_lin = np.zeros(3)
        self.integral_ang = np.zeros(3)
        self.prev_error_lin = None
        self.prev_error_ang = None
        self.prev_tick_time = None

        # Create timer to execute tick at needed loop rate in hz
        self.timer = self.create_timer(1.0 / self.loop_rate_hz, self._tick)

        self.get_logger().info(
            f"twist_pose_tracking_node ready @ {self.loop_rate_hz:.1f} Hz. "
            f"Subscribing: {target_topic} | Publishing: {twist_command_topic} "
        )

    def _declare(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _p(self, name):
        return self.get_parameter(name).value

    def _target_callback(self, msg: PoseStamped):
        # Our PID simple logic only handles targets expressed in base_frame so ...
        # ... we ignore incoming targets with different frame_id
        if msg.header.frame_id and msg.header.frame_id != self.base_frame:
            self.get_logger().warn(
                f"Ignoring target on frame '{msg.header.frame_id}' "
                f"(expected '{self.base_frame}').",
                throttle_duration_sec=5.0,
            )
            return
        self.latest_target = msg
        self.latest_target_stamp = self.get_clock().now()

    def _reset_integrators(self):
        self.integral_lin[:] = 0.0
        self.integral_ang[:] = 0.0
        self.prev_error_lin = None
        self.prev_error_ang = None
        self.prev_tick_time = None

    def _publish_zero_twist(self):
        self.twist_pub.publish(Twist())

    def _tick(self):
        now = self.get_clock().now()

        # hold and reset PID state if no target yet or target too old
        if self.latest_target is None or self.latest_target_stamp is None:
            self._publish_zero_twist()
            return
        age = (now - self.latest_target_stamp).nanoseconds * 1e-9
        if age > self._p("target_pose_timeout_s"):
            self._publish_zero_twist()
            self._reset_integrators()
            return

        # Current EE pose from TF (latest available, not exact-time)
        try:
            tf_msg = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, Time())
        except (LookupException, ExtrapolationException, ConnectivityException) as e:
            self.get_logger().warn(f"TF lookup failed ({self.base_frame} -> {self.ee_frame}): {e}",
                                   throttle_duration_sec=1.0)
            self._publish_zero_twist()
            return

        # transforms3d uses scalar-first [w, x, y, z] order (ROS messages are [x, y, z, w])
        q_current = [tf_msg.transform.rotation.w, tf_msg.transform.rotation.x,
                 tf_msg.transform.rotation.y, tf_msg.transform.rotation.z]
        q_target = [self.latest_target.pose.orientation.w, self.latest_target.pose.orientation.x,
                 self.latest_target.pose.orientation.y, self.latest_target.pose.orientation.z]
        pos_current = np.array([
            tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z,
        ])
        pos_target = np.array([
            self.latest_target.pose.position.x,
            self.latest_target.pose.position.y,
            self.latest_target.pose.position.z,
        ])

        # Pose error in base frame
        err_lin = pos_target - pos_current
        # Rotation error q_target * q_current^-1 expressed as axis*angle in base
        q_err = t3dq.qmult(q_target, t3dq.qinverse(q_current))
        axis, angle = t3dq.quat2axangle(q_err)
        err_ang = np.asarray(axis) * angle

        # Measured dt (falls back to nominal on the first tick after a reset)
        if self.prev_tick_time is None:
            dt = 1.0 / self.loop_rate_hz
        else:
            dt = max((now - self.prev_tick_time).nanoseconds * 1e-9, 1e-6)
        self.prev_tick_time = now

        # PID gains (live from parameter store).
        kp_lin = np.array([self._p("x_proportional_gain"), self._p("y_proportional_gain"), self._p("z_proportional_gain")])
        ki_lin = np.array([self._p("x_integral_gain"), self._p("y_integral_gain"), self._p("z_integral_gain")])
        kd_lin = np.array([self._p("x_derivative_gain"), self._p("y_derivative_gain"), self._p("z_derivative_gain")])
        kp_ang = self._p("angular_proportional_gain")
        ki_ang = self._p("angular_integral_gain")
        kd_ang = self._p("angular_derivative_gain")

        # We use a leaky integrator term so that for example once error is 0 the integrator term ...
        # ... slowly decays to 0. We do so that when the error is 0 we smoothly decrease the commanded twist
        windup_lin = self._p("linear_windup_limit")
        windup_ang = self._p("angular_windup_limit")
        leak_rate = self._p("integrator_leak_rate")
        decay = float(np.exp(-leak_rate * dt)) if leak_rate > 0.0 else 1.0
        self.integral_lin = np.clip(decay * self.integral_lin + err_lin * dt, -windup_lin, windup_lin)
        self.integral_ang = np.clip(decay * self.integral_ang + err_ang * dt, -windup_ang, windup_ang)

        # Derivative (zero on first tick after reset)
        d_lin = np.zeros(3) if self.prev_error_lin is None else (err_lin - self.prev_error_lin) / dt
        d_ang = np.zeros(3) if self.prev_error_ang is None else (err_ang - self.prev_error_ang) / dt
        self.prev_error_lin = err_lin
        self.prev_error_ang = err_ang

        # Compute linear and angular velocities in base frame
        v_base = kp_lin * err_lin + ki_lin * self.integral_lin + kd_lin * d_lin
        w_base = kp_ang * err_ang + ki_ang * self.integral_ang + kd_ang * d_ang

        # Saturate magnitudes in base frame
        v_base = _saturate(v_base, self._p("max_linear_velocity"))
        w_base = _saturate(w_base, self._p("max_angular_velocity"))

        # Rotate base_frame twist into end effector one
        # R_base_to_tool = R_ee_from_base = (R_base_from_ee)^T
        R_base_to_tool = t3dq.quat2mat(q_current).T
        v_tool = R_base_to_tool @ v_base
        w_tool = R_base_to_tool @ w_base

        # Prepare and publish message
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = float(v_tool[0]), float(v_tool[1]), float(v_tool[2])
        msg.angular.x, msg.angular.y, msg.angular.z = float(w_tool[0]), float(w_tool[1]), float(w_tool[2])
        self.twist_pub.publish(msg)


def _saturate(v, max_mag):
    mag = float(np.linalg.norm(v))
    if mag > max_mag and mag > 0.0:
        return v * (max_mag / mag)
    return v


def main():
    rclpy.init()
    node = TwistPoseTrackingNode()

    def _shutdown(signum=None, frame=None):
        try:
            node._publish_zero_twist()
        except Exception:
            pass
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    finally:
        try:
            node._publish_zero_twist()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
