#!/usr/bin/env bash
# zenoh/start_zenoh_bridge.sh
#
# Server-side Zenoh bridge launcher.
# Run this on the assigned GPU server to expose Isaac Sim's ROS 2 topics over TCP
# so that local laptops can subscribe to them via connect_zenoh_bridge.sh.
#
# Usage:
#   ./zenoh/start_zenoh_bridge.sh [PORT] [options]
#
# Options:
#   --domain <ID>          ROS_DOMAIN_ID to use (overrides current env)
#   --namespace <NS>       Namespace prefix applied to all bridged topics
#   --config <FILE>        Path to a Zenoh JSON5 config file (advanced)
#   -h, --help             Show this help
#
# Examples:
#   ./zenoh/start_zenoh_bridge.sh
#   ./zenoh/start_zenoh_bridge.sh 7447 --domain <id>
#   ./zenoh/start_zenoh_bridge.sh 7447 --config zenoh/configs/example_filter.json5

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_BIN="${SCRIPT_DIR}/zenoh-bridge-ros2dds"
DEFAULT_PORT="7447"

PORT=""
DOMAIN_ID=""
NAMESPACE=""
CONFIG_FILE=""

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
  $(basename "$0") [PORT] [options]

Arguments:
  PORT          TCP port to listen on (default: ${DEFAULT_PORT})

Options:
  --domain <ID>       Set ROS_DOMAIN_ID for the bridge process
  --namespace <NS>    Apply a namespace prefix to all bridged topics
  --config <FILE>     Zenoh JSON5 config file (topic filtering, routing, etc.)
  -h, --help          Show this help

Examples:
  $(basename "$0")
  $(basename "$0") 7447 --domain <id>
  $(basename "$0") 7447 --namespace /sim --config zenoh/configs/example_filter.json5
EOF
}

is_valid_port() {
  [[ "$1" =~ ^[0-9]+$ ]] && [ "$1" -ge 1 ] && [ "$1" -le 65535 ]
}

validate_domain_id() {
  [[ "$1" =~ ^[0-9]+$ ]] || error "Invalid ROS_DOMAIN_ID value: '$1'"
  local domain_id=$((10#$1))
  (( domain_id <= 232 )) || error "ROS_DOMAIN_ID must be between 0 and 232. Found: $1"
  printf '%s' "$domain_id"
}

source_domain_if_needed() {
  if [[ -z "${ROS_DOMAIN_ID:-}" && -r /etc/isaac-projects/ros.env ]]; then
    # shellcheck disable=SC1091
    set +u
    source /etc/isaac-projects/ros.env
    set -u
  fi

  if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
    ROS_DOMAIN_ID=$(validate_domain_id "$ROS_DOMAIN_ID")
    export ROS_DOMAIN_ID
  fi
}

source_ros_if_needed() {
  if [[ -n "${ROS_DISTRO:-}" ]]; then
    return 0
  fi

  local setup_file="" expected_distro="" os_pretty=""
  if [[ -n "${ROS_SETUP:-}" && -f "${ROS_SETUP}" ]]; then
    setup_file="${ROS_SETUP}"
  else
    if [[ -r /etc/os-release ]]; then
      local ID="" VERSION_ID="" PRETTY_NAME=""
      # shellcheck disable=SC1091
      . /etc/os-release
      os_pretty="${PRETTY_NAME:-Ubuntu ${VERSION_ID:-unknown}}"
      if [[ "${ID:-}" == "ubuntu" ]]; then
        case "${VERSION_ID:-}" in
          22.04) expected_distro="humble" ;;
          24.04) expected_distro="jazzy" ;;
        esac
      fi
    fi

    if [[ -n "$expected_distro" ]]; then
      setup_file="/opt/ros/${expected_distro}/setup.bash"
      [[ -f "$setup_file" ]] || error "Expected ROS 2 ${expected_distro} for ${os_pretty}, but ${setup_file} was not found. Run: ./isaac_vmctl.sh install ros2"
    else
      for distro in humble jazzy iron rolling; do
        if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
          setup_file="/opt/ros/${distro}/setup.bash"
          break
        fi
      done
    fi
  fi

  [[ -n "$setup_file" ]] || error "ROS 2 not found. Source /opt/ros/<distro>/setup.bash or run: ./isaac_vmctl.sh install ros2"
  warn "ROS 2 not sourced. Sourcing ${setup_file}..."
  # shellcheck source=/dev/null
  set +u
  source "${setup_file}"
  set -u
}

POSITIONAL=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)       usage; exit 0 ;;
    --domain)        [[ $# -ge 2 ]] || error "--domain requires a value"; DOMAIN_ID="$2"; shift 2 ;;
    --namespace)     [[ $# -ge 2 ]] || error "--namespace requires a value"; NAMESPACE="$2"; shift 2 ;;
    --config)        [[ $# -ge 2 ]] || error "--config requires a value"; CONFIG_FILE="$2"; shift 2 ;;
    -*) error "Unknown option: $1. Run with --help." ;;
    *)  POSITIONAL+=("$1"); shift ;;
  esac
done
set -- "${POSITIONAL[@]+"${POSITIONAL[@]}"}"

PORT="${1:-$DEFAULT_PORT}"

is_valid_port "$PORT" || error "Invalid port: '${PORT}'"

[[ -x "$BRIDGE_BIN" ]] || error "Bridge binary not found: ${BRIDGE_BIN}
Run: ./zenoh/setup.sh"

if [[ -n "$CONFIG_FILE" && ! -f "$CONFIG_FILE" ]]; then
  error "Config file not found: ${CONFIG_FILE}"
fi

source_domain_if_needed
source_ros_if_needed

if [[ -n "$DOMAIN_ID" ]]; then
  ROS_DOMAIN_ID=$(validate_domain_id "$DOMAIN_ID")
  export ROS_DOMAIN_ID
fi

echo ""
echo -e "${GREEN}=== Zenoh Bridge (router) ===${NC}"
echo ""
info "ROS distro : ${ROS_DISTRO}"
info "Listen     : tcp/0.0.0.0:${PORT}"
info "Mode       : router"
[[ -n "${ROS_DOMAIN_ID:-}" ]] && info "Domain ID  : ${ROS_DOMAIN_ID}"
[[ -n "$NAMESPACE" ]]         && info "Namespace  : ${NAMESPACE}"
[[ -n "$CONFIG_FILE" ]]       && info "Config     : ${CONFIG_FILE}"
echo ""
warn "Local laptops connect to: tcp/<GPU_PUBLIC_IP>:${PORT}"
warn "Ensure TCP ${PORT} is open inbound, or mapped externally by Vast.ai."
warn "Press Ctrl+C to stop the bridge."
echo ""

CMD=("$BRIDGE_BIN" "-l" "tcp/0.0.0.0:${PORT}" "--no-multicast-scouting")
[[ -n "$NAMESPACE" ]]   && CMD+=("-n" "$NAMESPACE")
[[ -n "$CONFIG_FILE" ]] && CMD+=("-c" "$CONFIG_FILE")
CMD+=("router")

exec "${CMD[@]}"
