#!/bin/bash
# Self-contained bootstrap for lerobot_ros. Run AFTER bootstrap_kinova_ws.sh.
#
#   1. Clones https://github.com/sacovo/lerobot_ros into ros2_ws/src/lerobot_ros
#   2. Installs uv + Rust toolchain if missing
#   3. Creates a uv venv at ros2_ws/src/lerobot_ros/.venv with --system-site-packages
#      so the venv inherits ROS Humble's rclpy (pinned to Python 3.10 for the same
#      reason — ROS Humble's binary bindings are built against 3.10)
#   4. uv pip-installs lerobot_ros (pulls in lerobot from git, rerun, rust_py_timer)
#   5. colcon-builds the three new packages (lerobot_interfaces, lerobot_ros, so101)
#      with the venv active so entry-point scripts shebang to the venv's Python.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEROBOT_ROS_DIR="$SCRIPT_DIR/src/lerobot_ros"
VENV_DIR="$LEROBOT_ROS_DIR/.venv"
LEROBOT_ROS_REPO="https://github.com/sacovo/lerobot_ros.git"
LEROBOT_ROS_REF="9e5c0bcdc4c016e49da158cc9a2b79eff0619a78"  # Last commit before recorder.py switched to lerobot.datasets.feature_utils (introduced in lerobot >=0.5, requires Python 3.12). Humble is 3.10, so we pin lerobot_ros to a state that imports from lerobot.datasets.utils (the 0.4.3 layout).

ROS_DISTRO="${ROS_DISTRO:-humble}"
if [[ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    echo "ERROR: /opt/ros/$ROS_DISTRO/setup.bash not found. Source your ROS env or set ROS_DISTRO."
    exit 1
fi

# uv
if ! command -v uv &>/dev/null; then
    echo "==> Installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    . "$HOME/.local/bin/env"
fi

# Clone lerobot_ros (pinned SHA for reproducibility; bump manually when you want to update)
if [[ ! -d "$LEROBOT_ROS_DIR/.git" ]]; then
    echo "==> Cloning lerobot_ros into $LEROBOT_ROS_DIR..."
    mkdir -p "$(dirname "$LEROBOT_ROS_DIR")"
    git clone "$LEROBOT_ROS_REPO" "$LEROBOT_ROS_DIR"
    git -C "$LEROBOT_ROS_DIR" checkout "$LEROBOT_ROS_REF"
fi

# Rust toolchain: rust_py_timer ships cp312-only wheels on PyPI. On Humble (Python
# 3.10) uv must compile the sdist, which needs cargo + rustc.
if ! command -v cargo &>/dev/null; then
    echo "==> Installing Rust toolchain..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    . "$HOME/.cargo/env"
fi

if [[ ! -d "$VENV_DIR" ]]; then
    echo "==> Creating uv venv at $VENV_DIR..."
    cd "$LEROBOT_ROS_DIR"
    uv venv --system-site-packages --python 3.10
fi

source "$VENV_DIR/bin/activate"

echo "==> Installing lerobot_ros + deps (lerobot pinned to 0.4.3; several GB)..."
# Two reasons we DON'T `uv sync` and we DON'T let `uv pip install` resolve lerobot
# fresh:
#   1. lerobot HEAD (0.5.2) bumped its Python floor to >=3.12 → breaks on Humble.
#   2. `uv sync` ignores the existing venv's Python and silently recreates it at
#      uv's managed CPython 3.12 (because pyproject.toml has no `requires-python`),
#      which then can't see ROS Humble's rclpy.
# So we use `uv pip install` (respects the existing 3.10 venv) + an explicit pin
# to the same lerobot SHA the upstream uv.lock targets (0.4.3, last 3.10-compatible).
cd "$LEROBOT_ROS_DIR"
LEROBOT_REF="603d44434f432c79765c6e18c290fccac1bd7b3e"
# --no-sources: ignore lerobot_ros's [tool.uv.sources] (which pins lerobot to git
# HEAD with no rev), so our explicit SHA below is the only source for lerobot.
uv pip install --no-sources . \
    "lerobot @ git+https://github.com/huggingface/lerobot.git@${LEROBOT_REF}"

python -c 'import rclpy, lerobot, rust_py_timer; print(f"lerobot {lerobot.__version__} OK")'

# pip's build leaves a copy of the source tree at $LEROBOT_ROS_DIR/build/lib/...
# which colcon then sees as a duplicate `lerobot_ros` package. Wipe it before
# colcon's package discovery runs.
echo "==> Removing pip build artifacts before colcon..."
rm -rf "$LEROBOT_ROS_DIR/build" "$LEROBOT_ROS_DIR/dist" "$LEROBOT_ROS_DIR"/*.egg-info

# Upstream lerobot_ros has `executable = /usr/bin/env python3` under [develop] in
# setup.cfg. setuptools 70+ rejects unknown options on the `develop` command and
# colcon errors out: "command 'develop' has no such option 'executable'". The
# line under [build_scripts] is valid and stays. We delete only the [develop] one.
LEROBOT_SETUP_CFG="$LEROBOT_ROS_DIR/src/lerobot_ros/setup.cfg"
if grep -qE '^\[develop\]' "$LEROBOT_SETUP_CFG"; then
    echo "==> Patching setup.cfg (drop spurious [develop] executable= line)..."
    sed -i '/^\[develop\]/,/^\[/{/^executable = /d}' "$LEROBOT_SETUP_CFG"
fi

# so101 is the SO-101 reference package shipped alongside lerobot_ros. We don't
# have that hardware and don't want it built — also its setup.cfg has the same
# spurious [develop] executable= line and would fail. Easiest: COLCON_IGNORE.
if [[ ! -f "$LEROBOT_ROS_DIR/src/so101/COLCON_IGNORE" ]]; then
    echo "==> Marking so101 as COLCON_IGNORE (we have no SO-101 hardware)..."
    touch "$LEROBOT_ROS_DIR/src/so101/COLCON_IGNORE"
fi

# Two-part patch so multiple TOML entries can subscribe to the same ROS topic
# (e.g. /joint_states recorded as both observation.state and action):
#   1. subscriber.py:156 passes the TOML section name (dict key) to
#      create_subscription instead of the per-topic configured ROS topic name.
#   2. BaseTopic only stores the *cleaned* form of topic_name (strips '/' and
#      replaces '/' with '.') — which is required as a dataset column prefix
#      but is invalid as a ROS topic name. So we add a `ros_topic_name`
#      attribute that preserves the raw form, and have subscriber.py use it.
LEROBOT_BASE_PY="$LEROBOT_ROS_DIR/src/lerobot_ros/lerobot_ros/convert/base.py"
LEROBOT_SUBSCRIBER_PY="$LEROBOT_ROS_DIR/src/lerobot_ros/lerobot_ros/subscriber.py"
if ! grep -q 'self\.ros_topic_name' "$LEROBOT_BASE_PY"; then
    echo "==> Patching convert/base.py (preserve raw ros_topic_name)..."
    sed -i 's|^        self\.topic_name = clean_topic_name(topic_name)$|        self.ros_topic_name = topic_name\n        self.topic_name = clean_topic_name(topic_name)|' "$LEROBOT_BASE_PY"
fi
if grep -qE '^                topic_name,$' "$LEROBOT_SUBSCRIBER_PY"; then
    echo "==> Patching subscriber.py (subscribe to topic.ros_topic_name)..."
    sed -i 's|^                topic_name,$|                topic.ros_topic_name,|' "$LEROBOT_SUBSCRIBER_PY"
fi

# subscriber.py:_convert_frame concatenates per-tag tensors (action /
# observation.state) in the order they appear in `frame.items()`. But the Rust
# FrameCollector returns its frame as a HashMap-backed dict whose iteration
# order is non-deterministic. With multiple tag="action" topics that get
# concatenated (e.g. arm joints + gripper command) the order flips between
# frames and the action column ends up scrambled. Iterate over self.topics
# (TOML insertion order) and look up tensors by name so concat is stable.
if ! grep -q 'Iterate over self\.topics (TOML insertion order)' "$LEROBOT_SUBSCRIBER_PY"; then
    echo "==> Patching subscriber.py (deterministic action concat order)..."
    python3 - "$LEROBOT_SUBSCRIBER_PY" <<'PYEOF'
import sys
path = sys.argv[1]
text = open(path).read()
old = '''        # average out the high frequency measurements
        for topic_name, tensors in frame.items():
            topic = self.topics[topic_name]
            key = key_for_topic(topic)'''
new = '''        # Iterate over self.topics (TOML insertion order) rather than `frame`
        # itself: the Rust FrameCollector returns its frame as a HashMap-backed
        # dict whose iteration order is non-deterministic. With multiple
        # tag="action" topics that get concatenated below, an undefined order
        # scrambles the action column on a per-frame basis. Looking up tensors
        # by topic_name keeps the concatenation order stable and matching the
        # column-name order produced by get_feature_description().
        for topic_name, topic in self.topics.items():
            tensors = frame.get(topic_name, [])
            key = key_for_topic(topic)'''
assert old in text, "_convert_frame loop snippet not found — upstream may have changed"
open(path, "w").write(text.replace(old, new))
PYEOF
fi

# Upstream recorder.py builds the LeRobotDataset without passing robot_type, so
# the value from the TOML never lands in info.json. Two small additions: unpack
# robot_type in Recorder.__init__ and pass it to LeRobotDataset.create().
LEROBOT_RECORDER_PY="$LEROBOT_ROS_DIR/src/lerobot_ros/lerobot_ros/recorder.py"
if ! grep -q 'self\.robot_type = config\.robot_type' "$LEROBOT_RECORDER_PY"; then
    echo "==> Patching recorder.py (thread robot_type into LeRobotDataset.create)..."
    sed -i 's|^        self\.tolerance_s = config\.tolerance_s$|        self.tolerance_s = config.tolerance_s\n        self.robot_type = config.robot_type|' "$LEROBOT_RECORDER_PY"
    sed -i 's|^                tolerance_s=self\.tolerance_s,$|                tolerance_s=self.tolerance_s,\n                robot_type=self.robot_type,|' "$LEROBOT_RECORDER_PY"
fi

# convert/sensor.py:JointStateTopic.to_tensor packs joint_state.position in the
# publisher's order, ignoring both the TOML's `joints` list and the message's
# own `name` field. That decouples dataset column labels from the values stored
# in them. Patch to_tensor to use joint_state.name to reindex into TOML order.
LEROBOT_SENSOR_PY="$LEROBOT_ROS_DIR/src/lerobot_ros/lerobot_ros/convert/sensor.py"
if ! grep -q 'name_to_idx = {name: i for i, name in enumerate(joint_state.name)}' "$LEROBOT_SENSOR_PY"; then
    echo "==> Patching convert/sensor.py (reindex JointState by name, not position)..."
    python3 - "$LEROBOT_SENSOR_PY" <<'PYEOF'
import sys
path = sys.argv[1]
text = open(path).read()
old = '''    def to_tensor(self, joint_state: sensor_msgs.JointState) -> torch.Tensor:
        """Convert a ROS JointState message to a PyTorch tensor."""
        values = []
        if self.has_position:
            values.append(np.array(joint_state.position, dtype=np.float32))
        if self.has_velocity:
            values.append(np.array(joint_state.velocity, dtype=np.float32))
        if self.has_effort:
            values.append(np.array(joint_state.effort, dtype=np.float32))
        return torch.tensor(
            np.stack(values, axis=0),
            dtype=torch.float32,
        ).flatten()'''
new = '''    def to_tensor(self, joint_state: sensor_msgs.JointState) -> torch.Tensor:
        """Convert a ROS JointState message to a PyTorch tensor.

        Reorders the message's values into the order declared by ``self.joints``
        in the TOML config — the message's joint order is publisher-defined and
        does not have to match the column order we want in the dataset.
        """
        name_to_idx = {name: i for i, name in enumerate(joint_state.name)}
        indices = [name_to_idx[j] for j in self.joints]
        values = []
        if self.has_position:
            position = np.array(joint_state.position, dtype=np.float32)
            values.append(position[indices])
        if self.has_velocity:
            velocity = np.array(joint_state.velocity, dtype=np.float32)
            values.append(velocity[indices])
        if self.has_effort:
            effort = np.array(joint_state.effort, dtype=np.float32)
            values.append(effort[indices])
        return torch.tensor(
            np.stack(values, axis=0),
            dtype=torch.float32,
        ).flatten()'''
assert old in text, "to_tensor snippet not found — upstream may have changed"
open(path, "w").write(text.replace(old, new))
PYEOF
fi

echo "==> colcon-building lerobot_* packages..."
source "/opt/ros/$ROS_DISTRO/setup.bash"
cd "$SCRIPT_DIR"
colcon build --symlink-install --packages-select lerobot_interfaces lerobot_ros

# ament_python locks colcon's Python interpreter to /usr/bin/python3 via CMake's
# find_package(PythonInterp), so setuptools generates entry-point shebangs as
# `#!/usr/bin/python3`. That interpreter can't import lerobot (venv-only), so
# `ros2 run lerobot_ros dataset_recorder` would crash on startup. Rewrite the
# shebangs to the venv's python — absolute path so activation isn't required.
echo "==> Repointing lerobot_ros entry-point shebangs at the venv python..."
VENV_PYTHON="$VENV_DIR/bin/python3"
for f in "$SCRIPT_DIR/install/lerobot_ros/lib/lerobot_ros"/*; do
    if [[ -f "$f" ]] && head -n1 "$f" | grep -qE '^#!.*python'; then
        sed -i "1s|^#!.*|#!${VENV_PYTHON}|" "$f"
    fi
done

echo ""
echo "==> lerobot_ros ready at $LEROBOT_ROS_DIR"
echo "    Per-terminal setup for nodes that import lerobot:"
echo "      source /opt/ros/$ROS_DISTRO/setup.bash"
echo "      source $VENV_DIR/bin/activate"
echo "      source $SCRIPT_DIR/install/setup.bash"
