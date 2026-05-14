"""
UR5e 6-DoF policy transforms for pi0_base.

Uses the native ur5e normalization stats that pi0_base was trained on.

Observation expected from the ROS 2 client:
  joints       float32 (6,)     arm joint positions in radians
  gripper      float32 (1,)     gripper position — send zeros if no gripper attached
  base_rgb     uint8   (H,W,3)  external/overhead camera
  wrist_rgb    uint8   (H,W,3)  wrist camera — send zeros if no wrist cam attached
  prompt       str              language instruction

Action returned to the ROS 2 client:
  actions      float32 (horizon, 7)  absolute positions: cols 0:6 = arm joints, col 6 = gripper
"""

import dataclasses

import einops
import numpy as np

from openpi import transforms
from openpi.models import model as _model


def _parse_image(image: np.ndarray) -> np.ndarray:
    image = np.asarray(image)
    if np.issubdtype(image.dtype, np.floating):
        image = (255 * image).astype(np.uint8)
    if image.ndim == 3 and image.shape[0] == 3:
        image = einops.rearrange(image, "c h w -> h w c")
    return image


@dataclasses.dataclass(frozen=True)
class UR5eInputs(transforms.DataTransformFn):
    model_type: _model.ModelType = _model.ModelType.PI0

    def __call__(self, data: dict) -> dict:
        state = np.concatenate([data["joints"], data["gripper"]])  # (7,)

        base_image = _parse_image(data["base_rgb"])
        wrist_image = _parse_image(data["wrist_rgb"])

        inputs = {
            "state": state,
            "image": {
                "base_0_rgb": base_image,
                "left_wrist_0_rgb": wrist_image,
                "right_wrist_0_rgb": np.zeros_like(base_image),
            },
            "image_mask": {
                "base_0_rgb": np.True_,
                # Mask out wrist cameras when not present; set True once a wrist cam is attached
                "left_wrist_0_rgb": np.False_,
                "right_wrist_0_rgb": np.False_,
            },
        }

        if "actions" in data:
            inputs["actions"] = data["actions"]
        if "prompt" in data:
            inputs["prompt"] = data["prompt"]

        return inputs


@dataclasses.dataclass(frozen=True)
class UR5eOutputs(transforms.DataTransformFn):
    def __call__(self, data: dict) -> dict:
        return {"actions": np.asarray(data["actions"][:, :7])}
