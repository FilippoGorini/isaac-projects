#!/usr/bin/env bash
# Project environment setup script.
#
# Source this in every new terminal before working on the project:
#   source setup.bash
#
# It will:
#   1. Load project-specific variables from .env (if present)
#   2. Load the default ROS_DOMAIN_ID for this machine
#   3. Source the ROS 2 environment for the current Ubuntu version
#   4. Source the colcon workspace if it has been built
#
# Do not enable strict shell options here: this file is meant to be sourced,
# and changing set -e/-u/pipefail would leak into the user's interactive shell.

_project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ─── 1. Load project .env ─────────────────────────────────────────────────────
if [[ -f "${_project_dir}/.env" ]]; then
  set -a
  # shellcheck source=/dev/null
  source "${_project_dir}/.env"
  set +a
  echo "[setup] Loaded ${_project_dir}/.env"
else
  echo "[setup] No .env found — skipping project overrides."
fi

# ─── 2. Source ROS 2 ──────────────────────────────────────────────────────────
if [[ -z "${ROS_DOMAIN_ID:-}" && -r /etc/isaac-projects/ros.env ]]; then
  # shellcheck disable=SC1091
  source /etc/isaac-projects/ros.env
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

_ros_sourced=0
_expected_ros_distro=""
if [[ -r /etc/os-release ]]; then
  _os_id="$(. /etc/os-release && printf '%s' "${ID:-}")"
  _os_version="$(. /etc/os-release && printf '%s' "${VERSION_ID:-}")"
  if [[ "$_os_id" == "ubuntu" ]]; then
    case "$_os_version" in
      22.04) _expected_ros_distro="humble" ;;
      24.04) _expected_ros_distro="jazzy" ;;
    esac
  fi
fi

if [[ -n "$_expected_ros_distro" ]]; then
  _ros_setup="/opt/ros/${_expected_ros_distro}/setup.bash"
  if [[ -f "$_ros_setup" ]]; then
    # shellcheck source=/dev/null
    source "$_ros_setup"
    _ros_sourced=1
    echo "[setup] Sourced ROS 2: ${_ros_setup}"
  else
    echo "[setup] WARNING: Expected ROS 2 ${_expected_ros_distro} for this Ubuntu version, but ${_ros_setup} was not found."
  fi
else
  for _ros_setup in /opt/ros/humble/setup.bash /opt/ros/jazzy/setup.bash /opt/ros/iron/setup.bash /opt/ros/rolling/setup.bash; do
    if [[ -f "$_ros_setup" ]]; then
      # shellcheck source=/dev/null
      source "$_ros_setup"
      _ros_sourced=1
      echo "[setup] Sourced ROS 2: ${_ros_setup}"
      break
    fi
  done
fi

if [[ $_ros_sourced -eq 0 ]]; then
  echo "[setup] WARNING: ROS 2 not found. Run: ./isaac_vmctl.sh install ros2"
fi

# ─── 3. Source colcon workspace ───────────────────────────────────────────────
_ws_setup="${_project_dir}/ros2_ws/install/setup.bash"
if [[ -f "$_ws_setup" ]]; then
  # shellcheck source=/dev/null
  source "$_ws_setup"
  echo "[setup] Sourced workspace: ${_ws_setup}"
else
  echo "[setup] Workspace not built yet. Run: cd ros2_ws && colcon build"
fi

echo "[setup] Done. ROS_DISTRO=${ROS_DISTRO:-not set} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-not set}"

unset _project_dir _ros_sourced _ros_setup _expected_ros_distro _os_id _os_version _ws_setup
