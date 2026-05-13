#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OPENPI_DIR="$(cd "$SCRIPT_DIR/../../external/openpi" && pwd)"

cd "$OPENPI_DIR"
uv run scripts/serve_policy.py --env ALOHA_SIM

# This script serves the pi0 ALOHA policy from the openpi directory, and is meant to be run on the machine which performs VLA inference.

# The serve_policy.py script does the following:
# 1. Resolves the ALOHA_SIM environment to a hardcoded checkpoint: config name pi0_aloha_sim, weights at gs://openpi-assets/checkpoints/pi0_aloha_sim
# 2. Downloads the checkpoint into ~/.cache/openpi/ (~11 GB) if not already cached
# 3. Loads the pi0_aloha_sim training config from openpi's registry: this defines the model architecture, expected observation format, normalization stats, and action space
# 4. Loads the model weights into GPU memory (~6 GB, takes ~7 seconds)
# 5. Downloads the PaliGemma tokenizer (~4 MB): pi0 uses a vision-language backbone that needs it to tokenize the text prompt
# 6. Loads normalization statistics for the ALOHA sim dataset (mean/std for joint states and actions, used to normalize inputs before the model and denormalize outputs after)
# 7. Creates a WebsocketPolicyServer listening on 0.0.0.0:8000 and calls serve_forever(): from this point it waits for incoming connections, and for each one it receives observation dicts, runs inference, and sends action chunks back