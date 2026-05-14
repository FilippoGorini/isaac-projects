#!/usr/bin/env python3
"""Serve pi0_base for UR5e 6-DoF."""
import dataclasses
import logging
import pathlib
import socket
import sys

import tyro

sys.path.insert(0, str(pathlib.Path(__file__).parent))

from openpi import transforms as _transforms
from openpi.models import pi0_config
from openpi.policies import policy_config as _policy_config
from openpi.serving import websocket_policy_server
from openpi.training.config import AssetsConfig, SimpleDataConfig, TrainConfig

from ur5e_policy import UR5eInputs, UR5eOutputs

PI0_BASE_CHECKPOINT = "gs://openpi-assets/checkpoints/pi0_base"

# Delta for arm joints 0-5, absolute for gripper (index 6)
_DELTA_MASK = _transforms.make_bool_mask(6, -1)


def _ur5e_data_transforms(model_config):
    return _transforms.Group(
        inputs=[UR5eInputs(model_type=model_config.model_type)],
        outputs=[UR5eOutputs()],
    ).push(
        inputs=[_transforms.DeltaActions(_DELTA_MASK)],
        outputs=[_transforms.AbsoluteActions(_DELTA_MASK)],
    )


def build_config() -> TrainConfig:
    return TrainConfig(
        name="pi0_ur5e",
        model=pi0_config.Pi0Config(),
        data=SimpleDataConfig(
            assets=AssetsConfig(
                assets_dir=f"{PI0_BASE_CHECKPOINT}/assets",
                asset_id="ur5e",
            ),
            data_transforms=_ur5e_data_transforms,
        ),
    )


@dataclasses.dataclass
class Args:
    port: int = 8000
    default_prompt: str | None = None


def main(args: Args) -> None:
    config = build_config()
    policy = _policy_config.create_trained_policy(
        config,
        PI0_BASE_CHECKPOINT,
        default_prompt=args.default_prompt,
    )

    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    logging.info("Creating server (host: %s, ip: %s)", hostname, local_ip)

    server = websocket_policy_server.WebsocketPolicyServer(
        policy=policy,
        host="0.0.0.0",
        port=args.port,
        metadata=policy.metadata or {},
    )
    server.serve_forever()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, force=True)
    main(tyro.cli(Args))
