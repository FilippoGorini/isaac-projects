#!/usr/bin/env bash
# zenoh/connect_zenoh_bridge.sh
#
# Local laptop Zenoh bridge client.
# Connects your local ROS 2 environment to the Zenoh router running on the
# assigned GPU server, making Isaac Sim's topics available locally.
#
# Usage:
#   ./zenoh/connect_zenoh_bridge.sh <GPU_HOST> [PORT] [options]
#
# Options:
#   --domain <ID>             Set ROS_DOMAIN_ID for this bridge process
#   --namespace <NS>          Namespace prefix for bridged topics
#   --mode <client|peer>      Zenoh mode (default: client)
#   --config <FILE>           Zenoh JSON5 config file (advanced filtering)
#   --multicast-scouting      Enable multicast scouting (off by default for WAN)
#   -h, --help                Show this help
#
# Examples:
#   ./zenoh/connect_zenoh_bridge.sh 203.0.113.45
#   ./zenoh/connect_zenoh_bridge.sh 203.0.113.45 7447 --domain 0
#   ./zenoh/connect_zenoh_bridge.sh 203.0.113.45 7447 --namespace /sim
#   ./zenoh/connect_zenoh_bridge.sh gpu.example.com 7447 --config zenoh/configs/example_filter.json5

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_BIN="${SCRIPT_DIR}/zenoh-bridge-ros2dds"
DEFAULT_PORT="7447"

GPU_HOST=""
PORT=""
MODE="client"
DOMAIN_ID=""
NAMESPACE=""
CONFIG_FILE=""
NO_MULTICAST=true

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

info()  { printf "${GREEN}[INFO]${NC}  %s\n" "$*"; }
warn()  { printf "${YELLOW}[WARN]${NC}  %s\n" "$*" >&2; }
error() { printf "${RED}[ERROR]${NC} %s\n" "$*" >&2; exit 1; }

usage() {
  cat <<EOF
Usage:
  $(basename "$0") <GPU_HOST> [PORT] [options]

Arguments:
  GPU_HOST    Public IP or hostname of the assigned GPU server
  PORT          Zenoh bridge port (default: ${DEFAULT_PORT})

Options:
  --domain <ID>             Set ROS_DOMAIN_ID for this bridge process
  --namespace <NS>          Namespace prefix for bridged topics
  --mode <client|peer>      Zenoh mode (default: client)
  --config <FILE>           Zenoh JSON5 config file
  --multicast-scouting      Enable multicast scouting (disabled by default)
  -h, --help                Show this help

Examples:
  $(basename "$0") 203.0.113.45
  $(basename "$0") 203.0.113.45 7447 --domain 0 --namespace /sim
  $(basename "$0") gpu.example.com 7447 --config zenoh/configs/example_filter.json5
EOF
}

is_valid_port() {
  [[ "$1" =~ ^[0-9]+$ ]] && [ "$1" -ge 1 ] && [ "$1" -le 65535 ]
}

is_valid_host() {
  [[ "$1" =~ ^[A-Za-z0-9._-]+$ ]]
}

source_ros_if_needed() {
  if [[ -n "${ROS_DISTRO:-}" ]]; then
    return 0
  fi

  local setup_file=""
  if [[ -n "${ROS_SETUP:-}" && -f "${ROS_SETUP}" ]]; then
    setup_file="${ROS_SETUP}"
  else
    for distro in jazzy humble iron rolling; do
      if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
        setup_file="/opt/ros/${distro}/setup.bash"
        break
      fi
    done
  fi

  [[ -n "$setup_file" ]] || error "ROS 2 not found in /opt/ros. Install ROS 2 or source it manually before running this script."
  warn "ROS 2 not sourced. Sourcing ${setup_file}..."
  # shellcheck source=/dev/null
  set +u
  source "${setup_file}"
  set -u
}

POSITIONAL=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)             usage; exit 0 ;;
    --domain)              [[ $# -ge 2 ]] || error "--domain requires a value"; DOMAIN_ID="$2"; shift 2 ;;
    --namespace)           [[ $# -ge 2 ]] || error "--namespace requires a value"; NAMESPACE="$2"; shift 2 ;;
    --mode)                [[ $# -ge 2 ]] || error "--mode requires a value"; MODE="$2"; shift 2 ;;
    --config)              [[ $# -ge 2 ]] || error "--config requires a value"; CONFIG_FILE="$2"; shift 2 ;;
    --multicast-scouting)  NO_MULTICAST=false; shift ;;
    -*) error "Unknown option: $1. Run with --help." ;;
    *)  POSITIONAL+=("$1"); shift ;;
  esac
done
set -- "${POSITIONAL[@]+"${POSITIONAL[@]}"}"

[[ $# -ge 1 ]] || { error "GPU_HOST is required."; }
GPU_HOST="$1"
PORT="${2:-$DEFAULT_PORT}"

is_valid_host "$GPU_HOST" || error "Invalid GPU_HOST: '${GPU_HOST}'"
is_valid_port "$PORT"       || error "Invalid PORT: '${PORT}'"

[[ "$MODE" == "client" || "$MODE" == "peer" || "$MODE" == "router" ]] \
  || error "Invalid mode '${MODE}'. Use client, peer, or router."

[[ -x "$BRIDGE_BIN" ]] || error "Bridge binary not found: ${BRIDGE_BIN}
Run: ./zenoh/setup.sh"

if [[ -n "$CONFIG_FILE" && ! -f "$CONFIG_FILE" ]]; then
  error "Config file not found: ${CONFIG_FILE}"
fi

source_ros_if_needed

if [[ -n "$DOMAIN_ID" ]]; then
  [[ "$DOMAIN_ID" =~ ^[0-9]+$ ]] || error "Invalid --domain value: '${DOMAIN_ID}'"
  export ROS_DOMAIN_ID="$DOMAIN_ID"
fi

echo ""
echo -e "${GREEN}=== Zenoh Bridge (client) ===${NC}"
echo ""
info "ROS distro  : ${ROS_DISTRO}"
info "GPU host    : ${GPU_HOST}:${PORT}"
info "Mode        : ${MODE}"
[[ -n "${ROS_DOMAIN_ID:-}" ]] && info "Domain ID   : ${ROS_DOMAIN_ID}"
[[ -n "$NAMESPACE" ]]         && info "Namespace   : ${NAMESPACE}"
[[ -n "$CONFIG_FILE" ]]       && info "Config      : ${CONFIG_FILE}"
echo ""

# Quick TCP reachability check before launching.
if command -v nc >/dev/null 2>&1; then
  if nc -z -w3 "$GPU_HOST" "$PORT" >/dev/null 2>&1; then
    info "Reachability check passed: ${GPU_HOST}:${PORT} is open."
  else
    warn "Reachability check failed: cannot reach ${GPU_HOST}:${PORT}."
    warn "Is start_zenoh_bridge.sh running? Is TCP ${PORT} open or mapped externally?"
  fi
  echo ""
fi

warn "Connecting... Press Ctrl+C to disconnect."
echo ""

CMD=("$BRIDGE_BIN" "-e" "tcp/${GPU_HOST}:${PORT}")
[[ "$NO_MULTICAST" == true ]] && CMD+=("--no-multicast-scouting")
[[ -n "$NAMESPACE" ]]         && CMD+=("-n" "$NAMESPACE")
[[ -n "$CONFIG_FILE" ]]       && CMD+=("-c" "$CONFIG_FILE")
CMD+=("$MODE")

exec "${CMD[@]}"
