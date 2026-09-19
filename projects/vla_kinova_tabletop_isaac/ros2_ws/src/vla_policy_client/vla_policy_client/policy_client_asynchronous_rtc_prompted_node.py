#!/usr/bin/env python3
"""Voice-prompted asynchronous RTC VLA policy client.

Subclasses policy_client_asynchronous_rtc unchanged; the only difference is where the prompt
comes from: `prompt_topic` (std_msgs/String from the speech_to_prompt node) instead of the
static `prompt` parameter. It's a latched level signal -- a non-empty prompt is the active
task, "" means stop.

  * No task ("" prompt) => no inference: querying the model without task conditioning would
    move the arm arbitrarily, so _build_obs gates and the plan is dropped (the C++ streamer
    then holds the last knot -> arm holds pose).
  * A non-empty prompt (re)starts inference exactly like the first one -- dropping the plan
    makes first_chunk True again, so no stale RTC prefix/blend.
  * The `prompt` parameter is an OPTIONAL initial task: non-empty => start immediately; the
    voice config leaves it "" and waits for the first utterance.

Prompts are normalized here too (process_prompt) so any publisher on the topic behaves the same.
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String

from vla_policy_client.policy_client_asynchronous_rtc_node import (
    PolicyClientAsynchronousRtcNode,
)
from vla_policy_client.speech_to_prompt_node import process_prompt


class PolicyClientAsynchronousRtcPromptedNode(PolicyClientAsynchronousRtcNode):
    def __init__(self):
        # Set BEFORE the base init: the base constructor starts the inference thread, which may
        # call _build_obs (overridden below, reads _prompt_topic) before this constructor finishes
        self._prompt_topic = "/vla_prompt"

        super().__init__(node_name="policy_client_asynchronous_rtc_prompted")

        self.declare_parameter("prompt_topic", "/vla_prompt")
        self._prompt_topic = (
            self.get_parameter("prompt_topic").get_parameter_value().string_value)
        self._prompt = process_prompt(self._prompt)     # normalize the optional config prompt
        # Latched QoS to match the speec to test node. Last prompt is kept by DDS so this client gets the most recent prompt even if started after the voice prompting node
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, self._prompt_topic, self._cb_prompt, qos)

        if self._prompt:
            self.get_logger().info(
                f"Initial prompt from config: \"{self._prompt}\" -- starting immediately; "
                f"prompts on '{self._prompt_topic}' replace it (\"\" = stop).")
        else:
            self.get_logger().info(
                f"Idle until a prompt arrives on '{self._prompt_topic}' "
                f"(start the speech_to_prompt node and speak).")

    def _cb_prompt(self, msg: String) -> None:
        # The topic is latched so we only react on a change
        prompt = process_prompt(msg.data)
        if prompt == self._prompt:
            return
        self._prompt = prompt
        if prompt:
            self.get_logger().info(f'New prompt: "{prompt}"')
        else:
            # If stopping, drop the active plan -> C++ streamer holds the last knot -> arm goes idle
            # When resuming first_chunk goes True again: keep the whole chunk and no stale RTC prefix or blend
            with self._plan_lock:
                self._plan = None
            self.get_logger().info("Prompt cleared -> inference paused, arm holding pose.")
        self._log_event({"type": "prompt", "t": self._now(), "prompt": prompt})

    def _build_obs(self) -> dict | None:
        # Don't query the model and keep arm idle if there's no task on the prompt topic (with no language conditioning it would move arbitrarily)
        # As soon as there is a valid non empty prompt, build observation the usual way and query the model
        if not self._prompt:
            with self._obs_lock:
                self._missing_status = f"idle, waiting for a prompt on '{self._prompt_topic}'"
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
