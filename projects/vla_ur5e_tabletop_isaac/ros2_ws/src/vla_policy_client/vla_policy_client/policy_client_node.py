#!/usr/bin/env python3
"""ROS 2 policy client for pi0 VLA — UR5e 6-DoF, direct Isaac Sim joint control.

Architecture:
  - Inference thread: runs flat-out, replaces the action buffer on every completion.
  - Execution timer (50 Hz): pops one action per tick and publishes it.
    Holds the last position if the buffer is momentarily empty.

This decouples inference latency from execution rate and eliminates the
gap between chunks that a sequential infer→execute loop would have.
"""

import collections
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

try:
    from openpi_client.websocket_client_policy import WebsocketClientPolicy
    from openpi_client.image_tools import resize_with_pad
except ImportError as e:
    raise SystemExit(
        f"Failed to import openpi_client ({e}). "
        "Run: pip install openpi-client typing-extensions"
    ) from e

ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
IMAGE_SIZE = 224


def _ros_image_to_uint8_hwc(msg: Image) -> np.ndarray:
    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding == "bgr8":
        return data[:, :, ::-1].copy()
    return data[:, :, :3].copy() if msg.encoding not in ("rgb8", "8UC3") else data.copy()


class PolicyClientNode(Node):
    def __init__(self):
        super().__init__("policy_client")

        # --- parameters ---
        self.declare_parameter("policy_host", "localhost")
        self.declare_parameter("policy_port", 8000)
        self.declare_parameter("prompt", "pick up the object")
        self.declare_parameter("control_hz", 50.0)

        host = self.get_parameter("policy_host").get_parameter_value().string_value
        port = self.get_parameter("policy_port").get_parameter_value().integer_value
        self._prompt = self.get_parameter("prompt").get_parameter_value().string_value
        hz = self.get_parameter("control_hz").get_parameter_value().double_value

        self._dt = 1.0 / hz

        # --- policy server connection ---
        self.get_logger().info(f"Connecting to policy server ws://{host}:{port} ...")
        self._policy = WebsocketClientPolicy(host=host, port=port)
        self.get_logger().info("Connected.")

        # --- observation state (guarded by _obs_lock) ---
        self._obs_lock = threading.Lock()
        self._joint_positions: dict[str, float] = {}
        self._base_image: np.ndarray | None = None

        # --- action buffer (guarded by _buf_lock) ---
        # Inference thread replaces the whole buffer; execution timer pops from it.
        self._buf_lock = threading.Lock()
        self._action_buffer: collections.deque = collections.deque()
        self._last_positions: list[float] | None = None

        # --- publisher ---
        self._arm_pub = self.create_publisher(JointState, "/isaac_joint_commands", 10)

        # --- subscriptions ---
        self.create_subscription(JointState, "/isaac_joint_states", self._cb_joint_states, 10)
        self.create_subscription(
            Image, "/isaac_external_camera/color/image_raw", self._cb_base_image, 2
        )

        # --- 50 Hz execution timer ---
        self.create_timer(self._dt, self._execution_step)

        # --- inference thread ---
        self._running = True
        self._infer_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self._infer_thread.start()

        self.get_logger().info("Policy client started.")

    # ------------------------------------------------------------------
    # Observation callbacks
    # ------------------------------------------------------------------

    def _cb_joint_states(self, msg: JointState) -> None:
        with self._obs_lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = pos

    def _cb_base_image(self, msg: Image) -> None:
        arr = _ros_image_to_uint8_hwc(msg)
        with self._obs_lock:
            self._base_image = resize_with_pad(arr, IMAGE_SIZE, IMAGE_SIZE)

    # ------------------------------------------------------------------
    # Execution timer (50 Hz) — runs in the ROS2 executor thread
    # ------------------------------------------------------------------

    def _execution_step(self) -> None:
        with self._buf_lock:
            if self._action_buffer:
                action = self._action_buffer.popleft()
                self._last_positions = [float(v) for v in action[:6]]

        if self._last_positions is None:
            return  # waiting for first inference

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM_JOINTS
        msg.position = self._last_positions
        self._arm_pub.publish(msg)

    # ------------------------------------------------------------------
    # Inference loop — runs in a background thread
    # ------------------------------------------------------------------

    def _build_obs(self) -> dict | None:
        with self._obs_lock:
            missing = [j for j in ARM_JOINTS if j not in self._joint_positions]
            if missing or self._base_image is None:
                return None
            joints = np.array(
                [self._joint_positions[j] for j in ARM_JOINTS], dtype=np.float32
            )
            obs = {
                "joints": joints,
                "gripper": np.zeros(1, dtype=np.float32),
                "base_rgb": self._base_image.copy(),
                "wrist_rgb": np.zeros((IMAGE_SIZE, IMAGE_SIZE, 3), dtype=np.uint8),
                "prompt": self._prompt,
            }
        return obs

    def _inference_loop(self) -> None:
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

            actions = np.asarray(result["actions"])
            if actions.ndim != 2 or actions.shape[1] < 7:
                self.get_logger().error(f"Unexpected action shape: {actions.shape}")
                continue

            with self._buf_lock:
                buf_remaining = len(self._action_buffer)
                self._action_buffer = collections.deque(actions)

            self.get_logger().info(
                f"Inference {t_infer * 1e3:.1f} ms | "
                f"replaced buffer ({buf_remaining} steps remaining)"
            )

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
