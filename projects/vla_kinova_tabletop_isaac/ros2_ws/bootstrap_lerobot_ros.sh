#!/bin/bash
# Bootstrap for lerobot_ros. Run AFTER bootstrap_kinova_ws.sh.
#
# This script:
#   1. Installs uv and a Rust toolchain if they aren't already on the system.
#      Rust is needed because rust_py_timer (a lerobot_ros dependency) only
#      ships pre-built wheels for Python 3.12 on PyPI, so on Humble (Python
#      3.10) uv has to compile it from source
#   2. Clones FilippoGorini/lerobot_ros (our own fork of sacovo/lerobot_ros)
#      into ros2_ws/src/lerobot_ros on the `humble-patches` branch. The fork
#      carries the patches needed to make the package work with ROS Humble
#      (Python 3.10) and to fix some bugs
#   3. Creates a uv venv inside the cloned repo with --system-site-packages
#      and --python 3.10 so it inherits Humble's rclpy
#   4. Installs lerobot_ros and its Python deps into the venv with
#      `uv pip install .`. The lerobot version pin lives in the fork's
#      pyproject.toml so we don't need any `--no-sources` flags here
#   5. Builds lerobot_interfaces and lerobot_ros with colcon
#   6. Rewrites the entry-point shebangs to point at the venv's Python so
#      that `ros2 run lerobot_ros dataset_recorder` actually uses the venv

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEROBOT_ROS_DIR="$SCRIPT_DIR/src/lerobot_ros"
VENV_DIR="$LEROBOT_ROS_DIR/.venv"
LEROBOT_ROS_REPO="https://github.com/FilippoGorini/lerobot_ros.git"
LEROBOT_ROS_REF="humble-patches"  # branch on the fork

ROS_DISTRO="${ROS_DISTRO:-humble}"
if [[ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    echo "ERROR: /opt/ros/$ROS_DISTRO/setup.bash not found. Source your ROS env or set ROS_DISTRO."
    exit 1
fi

# Install uv if it's not already on the PATH
if ! command -v uv &>/dev/null; then
    echo "==> Installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    . "$HOME/.local/bin/env"
fi

# Install a Rust toolchain if cargo isn't already on the PATH
# We need it to build rust_py_timer from sdist on Python 3.10
if ! command -v cargo &>/dev/null; then
    echo "==> Installing Rust toolchain..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    . "$HOME/.cargo/env"
fi

# Clone the fork on the patches branch (skip if already cloned)
if [[ ! -d "$LEROBOT_ROS_DIR/.git" ]]; then
    echo "==> Cloning FilippoGorini/lerobot_ros@$LEROBOT_ROS_REF into $LEROBOT_ROS_DIR..."
    mkdir -p "$(dirname "$LEROBOT_ROS_DIR")"
    git clone -b "$LEROBOT_ROS_REF" "$LEROBOT_ROS_REPO" "$LEROBOT_ROS_DIR"
fi

# Create the uv venv with system-site-packages so it can see Humble's rclpy
if [[ ! -d "$VENV_DIR" ]]; then
    echo "==> Creating uv venv at $VENV_DIR..."
    cd "$LEROBOT_ROS_DIR"
    uv venv --system-site-packages --python 3.10
fi

source "$VENV_DIR/bin/activate"

# Install lerobot_ros into the venv
# We use `uv pip install` (not `uv sync`) on purpose: `uv sync` would notice
# that pyproject.toml has no `requires-python` and silently swap the venv to
# uv's own Python 3.12, which would lose access to rclpy (compiled for 3.10)
echo "==> Installing lerobot_ros + deps (could take some time)..."
cd "$LEROBOT_ROS_DIR"
uv pip install .

# Sanity check imports
python -c 'import rclpy, lerobot, rust_py_timer; print(f"lerobot {lerobot.__version__} OK")'

# `uv pip install .` leaves a copy of the package under $LEROBOT_ROS_DIR/build/,
# which colcon would later see as a second lerobot_ros and refuse to build so we delete these artifacts before building
echo "==> Removing pip build artifacts before colcon..."
rm -rf "$LEROBOT_ROS_DIR/build" "$LEROBOT_ROS_DIR/dist" "$LEROBOT_ROS_DIR"/*.egg-info

# Build the 2 lerobot ros packages
echo "==> colcon-building lerobot_* packages..."
source "/opt/ros/$ROS_DISTRO/setup.bash"
cd "$SCRIPT_DIR"
colcon build --symlink-install --packages-select lerobot_interfaces lerobot_ros

# Rewrite the entry-point shebangs to point at the venv's Python.
# ament_python's CMake locks colcon's Python interpreter to /usr/bin/python3,
# so setuptools writes shebangs like `#!/usr/bin/python3`. That interpreter
# doesn't see lerobot (it's installed in the venv only), so `ros2 run
# lerobot_ros dataset_recorder` would crash. We rewrite each shim's first line
# to the absolute path of the venv's python so activation isn't needed
echo "==> Repointing lerobot_ros entry-point shebangs at the venv python..."
VENV_PYTHON="$VENV_DIR/bin/python3"
for f in "$SCRIPT_DIR/install/lerobot_ros/lib/lerobot_ros"/*; do
    if [[ -f "$f" ]] && head -n1 "$f" | grep -qE '^#!.*python'; then
        sed -i "1s|^#!.*|#!${VENV_PYTHON}|" "$f"
    fi
done

echo ""
echo "==> lerobot_ros ready at $LEROBOT_ROS_DIR"
echo "    Re-run this bootstrap whenever you colcon-build lerobot_ros,"
echo "    otherwise the shebangs get reset to /usr/bin/python3 and the"
echo "    entry-point scripts will fail to import lerobot."
