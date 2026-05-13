#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
OPENPI_DIR="$PROJECT_DIR/external/openpi"

# Install uv if not already installed, this is needed for the openpi installation
if ! command -v uv &>/dev/null; then
    echo "==> Installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    source "$HOME/.local/bin/env"
fi

# Clone openpi if not present already
if [[ ! -d "$OPENPI_DIR/.git" ]]; then
    echo "==> Cloning openpi into $OPENPI_DIR..."
    mkdir -p "$PROJECT_DIR/external"
    git clone --recurse-submodules https://github.com/Physical-Intelligence/openpi.git "$OPENPI_DIR"
fi

echo "==> Running uv sync..."
cd "$OPENPI_DIR"
GIT_LFS_SKIP_SMUDGE=1 uv sync
GIT_LFS_SKIP_SMUDGE=1 uv pip install -e .

# Install the openpi WebSocket client into system Python so that the ROS 2
# policy_client node (which runs under /usr/bin/python3, not the uv venv) can
# import it.  typing-extensions is a required transitive dep that openpi-client
# omits from its metadata on Python <3.12.
echo "==> Installing openpi-client into system Python..."
pip install openpi-client typing-extensions

echo ""
echo "==> openpi ready at $OPENPI_DIR"
