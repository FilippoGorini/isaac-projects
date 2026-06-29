#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
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

# System packages:
#   - python3-pip: to install openpi-client into system Python 
#   - ffmpeg: torchcodec (lerobot's video decoder) needs the FFmpeg libs to decode dataset videos at training time
#   - gsutil: faster checkpoint downloads from gs://openpi-assets (openpi falls back to gcsfs without it)
echo "==> Installing system dependencies (python3-pip, ffmpeg, gsutil)..."
sudo apt-get update -y
sudo apt-get install -y python3-pip ffmpeg gsutil

# Install the openpi WebSocket client into system Python so that the ROS 2
# policy_client node (which runs under /usr/bin/python3, not the uv venv) can
# import it.  typing-extensions is a required transitive dep that openpi-client
# omits from its metadata on Python <3.12.
echo "==> Installing openpi-client into system Python..."
python3 -m pip install openpi-client typing-extensions

echo ""
echo "==> openpi ready at $OPENPI_DIR"
