#!/bin/bash
set -e

# Shared-server variant of bootstrap_openpi_train.sh.
# Use this on a machine where you do NOT have root and must not touch the system Python
# (e.g. a lab/HPC server used by other people). Differences from bootstrap_openpi_train.sh:
#   - no `sudo apt-get` (assumes ffmpeg is already installed; only warns if it isn't)
#   - gsutil + huggingface_hub go INTO the openpi venv, not the system Python
# Everything else (uv, the openpi venv) is already per-user and isolated, so it's unchanged.

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OPENPI_DIR="$PROJECT_DIR/external/openpi"

FORK_URL="https://github.com/FilippoGorini/openpi.git"
FORK_BRANCH="train_rtc"
UPSTREAM_URL="https://github.com/Physical-Intelligence/openpi.git"

# Install uv if not already installed (user-level, into ~/.local — no root needed).
if ! command -v uv &>/dev/null; then
    echo "==> Installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    source "$HOME/.local/bin/env"
fi

# Clone our fork at the training branch if not present already.
if [[ ! -d "$OPENPI_DIR/.git" ]]; then
    echo "==> Cloning openpi fork (branch: $FORK_BRANCH) into $OPENPI_DIR..."
    mkdir -p "$PROJECT_DIR/external"
    git clone --recurse-submodules --branch "$FORK_BRANCH" "$FORK_URL" "$OPENPI_DIR"
    git -C "$OPENPI_DIR" remote add upstream "$UPSTREAM_URL"
fi

echo "==> Running uv sync..."
cd "$OPENPI_DIR"
GIT_LFS_SKIP_SMUDGE=1 uv sync
GIT_LFS_SKIP_SMUDGE=1 uv pip install -e .

# ffmpeg: torchcodec needs the system ffmpeg libraries to decode dataset videos at training time.
# We can't `sudo apt-get install` it on a shared box, so just check and warn.
if ! command -v ffmpeg &>/dev/null; then
    echo ""
    echo "!! WARNING: ffmpeg not found on PATH. torchcodec needs it to decode dataset videos, so"
    echo "!! training will likely fail. Ask the server admin to install ffmpeg (it's usually already"
    echo "!! present on ML servers). This script does NOT install system packages."
    echo ""
fi

# gsutil (for `gsutil -m cp` of gs://openpi-assets base checkpoints) and the huggingface_hub CLI
# (pull/push checkpoints) go INTO the openpi venv instead of the system Python, so no root and no
# system-wide changes. They land on PATH whenever you run via `uv run`.
# NOTE: these are installed AFTER `uv sync`; a later `uv sync` re-matches the lockfile and would drop
# them, so re-run this script (or add them to pyproject.toml) if you ever re-sync.
echo "==> Installing gsutil + huggingface_hub CLI into the openpi venv..."
uv pip install gsutil "huggingface_hub[cli,hf_xet]"

echo ""
echo "==> openpi ready at $OPENPI_DIR (shared-server setup, nothing installed system-wide)"
