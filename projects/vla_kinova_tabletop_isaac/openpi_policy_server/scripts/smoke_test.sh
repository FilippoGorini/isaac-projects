#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OPENPI_DIR="$(cd "$SCRIPT_DIR/../../external/openpi" && pwd)"

HOST="${1:-localhost}"
PORT="${2:-8000}"

cd "$OPENPI_DIR"
uv run examples/simple_client/main.py --env ALOHA_SIM --host "$HOST" --port "$PORT"
