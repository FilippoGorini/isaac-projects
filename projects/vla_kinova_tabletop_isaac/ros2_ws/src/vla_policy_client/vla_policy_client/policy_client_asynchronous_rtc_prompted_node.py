#!/usr/bin/env python3
"""Voice-prompted asynchronous RTC VLA policy client.

Identical to policy_client_asynchronous_rtc (it subclasses it, so the whole grid/RTC
machinery is literally the same code); the only differences:

  * The prompt comes from `prompt_topic` (std_msgs/String, published by the speech_to_prompt
    node) instead of the static `prompt` parameter. A new message swaps the prompt used in
    all subsequent observations.
  * NO observations are sent until the first prompt arrives -- querying the model without
    the task conditioning it was trained with would make the arm do arbitrary things. The
    node connects to the server, then idles (with a throttled "waiting" log) until the
    first prompt; the grid anchors on the first real observation as usual.
  * The `prompt` parameter is kept as an OPTIONAL initial prompt: non-empty => start
    immediately with it (no waiting), and voice prompts replace it as they arrive. The
    voice config leaves it "" (wait for the first utterance).

Prompts are published already normalized (lowercase, no trailing punctuation) by the
speech_to_prompt node; the same normalization is applied here anyway so any other publisher
on the topic behaves identically.
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from vla_policy_client.policy_client_asynchronous_rtc_node import (
    PolicyClientAsynchronousRtcNode,
)
from vla_policy_client.speech_to_prompt_node import normalize_prompt


class PolicyClientAsynchronousRtcPromptedNode(PolicyClientAsynchronousRtcNode):
    def __init__(self):
        # Set BEFORE the base init: the base constructor starts the inference thread, which
        # may call _build_obs (overridden below) before this constructor finishes.
        self._prompt_ready = False
        self._prompt_topic = "/vla_prompt"

        super().__init__(node_name="policy_client_asynchronous_rtc_prompted")

        self.declare_parameter("prompt_topic", "/vla_prompt")
        self._prompt_topic = (
            self.get_parameter("prompt_topic").get_parameter_value().string_value)
        self.create_subscription(String, self._prompt_topic, self._cb_prompt, 10)

        if self._prompt.strip():
            self._prompt = normalize_prompt(self._prompt)
            self._prompt_ready = True
            self.get_logger().info(
                f"Initial prompt from config: \"{self._prompt}\" -- starting immediately; "
                f"voice prompts on '{self._prompt_topic}' will replace it.")
        else:
            self.get_logger().info(
                f"Holding observations until the first prompt arrives on "
                f"'{self._prompt_topic}' (start the speech_to_prompt node and speak).")

    def _cb_prompt(self, msg: String) -> None:
        prompt = normalize_prompt(msg.data)
        if not prompt:
            return
        # The topic is re-published at ~5 Hz; only react (log/swap) on an actual change.
        if prompt != self._prompt:
            self._prompt = prompt
            self.get_logger().info(f'New prompt: "{prompt}"')
            self._log_event({"type": "prompt", "t": self._now(), "prompt": prompt})
        self._prompt_ready = True

    def _build_obs(self) -> dict | None:
        # Gate: without the first prompt the model has no task conditioning, so send nothing
        # (the inference loop just logs the throttled "Waiting for observations" line).
        if not self._prompt_ready:
            with self._obs_lock:
                self._missing_status = f"waiting for first prompt on '{self._prompt_topic}'"
            return None
        return super()._build_obs()


def main(args=None):
    rclpy.init(args=args)
    node = PolicyClientAsynchronousRtcPromptedNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
