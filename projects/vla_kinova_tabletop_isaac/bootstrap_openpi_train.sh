#!/bin/bash
set -e

# Server-side bootstrap script for the machine which trains the model.
# This is the same as bootstrap_openpi.sh but adds the system dependencies needed for training.
# To run inference on the lab's server use the bare bootstrap_openpi.sh instead of this.
# Not needed on machines that only run the ROS 2 client (use bootstrap_openpi_client.sh).

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OPENPI_DIR="$PROJECT_DIR/external/openpi"

FORK_URL="https://github.com/FilippoGorini/openpi.git"
FORK_BRANCH="kinova-gen3"
UPSTREAM_URL="https://github.com/Physical-Intelligence/openpi.git"

# Install uv if not already installed, this is needed for the openpi installation
if ! command -v uv &>/dev/null; then
    echo "==> Installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    source "$HOME/.local/bin/env"
fi

# Clone our fork at the kinova-gen3 branch if not present already
if [[ ! -d "$OPENPI_DIR/.git" ]]; then
    echo "==> Cloning openpi fork (branch: $FORK_BRANCH) into $OPENPI_DIR..."
    mkdir -p "$PROJECT_DIR/external"
    git clone --recurse-submodules --branch "$FORK_BRANCH" "$FORK_URL" "$OPENPI_DIR"
    # Keep upstream registered so we can pull new releases with:
    #   git checkout main && git pull upstream main && git push origin main
    #   git checkout kinova-gen3 && git rebase main
    git -C "$OPENPI_DIR" remote add upstream "$UPSTREAM_URL"
fi

echo "==> Running uv sync..."
cd "$OPENPI_DIR"
GIT_LFS_SKIP_SMUDGE=1 uv sync
GIT_LFS_SKIP_SMUDGE=1 uv pip install -e .

# Additional system packages needed for training:
# - ffmpeg: torchcodec needs it to decode dataset videos at training time
# - python3-pip: to install the huggingface_hub CLI + gsutil into system Python
echo "==> Installing system dependencies (ffmpeg, python3-pip)..."
sudo apt-get update -y
sudo apt-get install -y ffmpeg python3-pip

# gsutil: openpi pulls base checkpoints with `gsutil -m cp -r` from gs://openpi-assets.
# NOTE: the apt `gsutil` package is an unrelated grid tool that rejects `-m`: install
# Google's real gsutil from PyPI instead (public bucket, no credentials needed)
echo "==> Installing gsutil into system Python..."
python3 -m pip install -U gsutil

# Also install huggingface hub so that we can pull/push checkpoints
echo "==> Installing huggingface_hub CLI into system Python..."
python3 -m pip install "huggingface_hub[cli,hf_xet]"

echo ""
echo "==> openpi ready at $OPENPI_DIR"
