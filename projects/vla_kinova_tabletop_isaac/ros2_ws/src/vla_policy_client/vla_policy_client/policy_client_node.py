#!/usr/bin/env python3
"""ROS 2 policy client for pi0 VLA — Kinova Gen3 6-DoF + Robotiq 2F-85."""

import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from openpi_client import WebsocketClientPolicy
    from openpi_client.image_tools import resize_with_pad
except ImportError as e:
    raise SystemExit(
        "openpi-client not installed — run: pip install openpi-client"
    ) from e

ARM_JOINTS = [
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6",
]
GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"
IMAGE_SIZE = 224  # pi0 canonical input resolution


def _ros_image_to_uint8_hwc(msg: Image) -> np.ndarray:
    """Convert a sensor_msgs/Image (rgb8 or bgr8) to uint8 HWC numpy array."""
    dtype = np.uint8
    data = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, -1)
    if msg.encoding == "bgr8":
        data = data[:, :, ::-1].copy()
    elif msg.encoding in ("rgb8", "8UC3"):
        data = data.copy()
    else:
        # Fallback: assume first 3 channels are RGB
        data = data[:, :, :3].copy()
    return data


class PolicyClientNode(Node):
    def __init__(self):
        super().__init__("policy_client")

        # --- parameters ---
        self.declare_parameter("policy_host", "localhost")
        self.declare_parameter("policy_port", 8000)
        self.declare_parameter("prompt", "pick up the object")
        self.declare_parameter("control_hz", 50.0)
        self.declare_parameter("chunk_size", 50)

        host = self.get_parameter("policy_host").get_parameter_value().string_value
        port = self.get_parameter("policy_port").get_parameter_value().integer_value
        self._prompt = self.get_parameter("prompt").get_parameter_value().string_value
        hz = self.get_parameter("control_hz").get_parameter_value().double_value
        self._chunk_size = self.get_parameter("chunk_size").get_parameter_value().integer_value

        self._dt = 1.0 / hz  # seconds per step (20 ms at 50 Hz)

        # --- WebSocket policy connection ---
        self.get_logger().info(f"Connecting to policy server ws://{host}:{port} ...")
        self._policy = WebsocketClientPolicy(host=host, port=port)
        self.get_logger().info("Connected.")

        # --- state (guarded by _lock) ---
        self._lock = threading.Lock()
        self._joint_positions: dict[str, float] = {}
        self._base_image: np.ndarray | None = None
        self._wrist_image: np.ndarray | None = None

        # --- publishers ---
        self._arm_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )
        self._gripper_client = ActionClient(
            self, GripperCommand, "/robotiq_gripper_controller/gripper_cmd"
        )

        # --- subscriptions ---
        self.create_subscription(JointState, "/joint_states", self._cb_joint_states, 10)
        self.create_subscription(
            Image, "/isaac_external_camera/color/image_raw", self._cb_base_image, 2
        )
        self.create_subscription(
            Image, "/isaac_wrist_camera/color/image_raw", self._cb_wrist_image, 2
        )

        # --- control loop thread ---
        self._running = True
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()
        self.get_logger().info("Policy client started.")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_joint_states(self, msg: JointState) -> None:
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = pos

    def _cb_base_image(self, msg: Image) -> None:
        arr = _ros_image_to_uint8_hwc(msg)
        with self._lock:
            self._base_image = resize_with_pad(arr, IMAGE_SIZE, IMAGE_SIZE)

    def _cb_wrist_image(self, msg: Image) -> None:
        arr = _ros_image_to_uint8_hwc(msg)
        with self._lock:
            self._wrist_image = resize_with_pad(arr, IMAGE_SIZE, IMAGE_SIZE)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _build_obs(self):
        """Return observation dict or None if state is not yet ready."""
        with self._lock:
            missing = [
                j for j in ARM_JOINTS + [GRIPPER_JOINT]
                if j not in self._joint_positions
            ]
            if missing or self._base_image is None or self._wrist_image is None:
                return None

            joints = np.array(
                [self._joint_positions[j] for j in ARM_JOINTS], dtype=np.float32
            )
            gripper = np.array(
                [self._joint_positions[GRIPPER_JOINT]], dtype=np.float32
            )
            obs = {
                "joints": joints,
                "gripper": gripper,
                "base_rgb": self._base_image.copy(),
                "wrist_rgb": self._wrist_image.copy(),
                "prompt": self._prompt,
            }
        return obs

    def _publish_arm_chunk(self, actions: np.ndarray) -> None:
        """Publish a 50-step JointTrajectory (absolute joint positions)."""
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        now = self.get_clock().now().to_msg()
        for i, action in enumerate(actions):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in action[:6]]
            elapsed_ns = int((i + 1) * self._dt * 1e9)
            pt.time_from_start = Duration(
                sec=elapsed_ns // 1_000_000_000,
                nanosec=elapsed_ns % 1_000_000_000,
            )
            msg.points.append(pt)
        self._arm_pub.publish(msg)

    def _send_gripper_goal(self, position: float) -> None:
        """Send a single GripperCommand goal (non-blocking)."""
        if not self._gripper_client.wait_for_server(timeout_sec=0.0):
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 50.0
        self._gripper_client.send_goal_async(goal)

    def _control_loop(self) -> None:
        while self._running and rclpy.ok():
            obs = self._build_obs()
            if obs is None:
                time.sleep(0.05)
                continue

            t0 = time.monotonic()
            try:
                result = self._policy.infer(obs)
            except Exception as exc:
                self.get_logger().error(f"Inference error: {exc}")
                time.sleep(0.1)
                continue
            t_infer = time.monotonic() - t0

            actions = np.asarray(result["actions"])  # (50, 7)
            if actions.ndim != 2 or actions.shape[1] < 7:
                self.get_logger().error(f"Unexpected action shape: {actions.shape}")
                continue

            self._publish_arm_chunk(actions)
            # Use the last gripper command in the chunk as target
            self._send_gripper_goal(actions[-1, 6])

            self.get_logger().debug(
                f"Chunk published — infer {t_infer*1e3:.1f} ms, "
                f"chunk_steps={len(actions)}"
            )

            # Sleep for one chunk duration minus inference time, so the next
            # inference starts roughly when this trajectory is finishing.
            sleep_time = self._chunk_size * self._dt - t_infer
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ------------------------------------------------------------------

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PolicyClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
