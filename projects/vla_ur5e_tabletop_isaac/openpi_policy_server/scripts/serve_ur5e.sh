#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OPENPI_DIR="$(cd "$SCRIPT_DIR/../../external/openpi" && pwd)"
SERVE_SCRIPT="$(cd "$SCRIPT_DIR/../src" && pwd)/serve_ur5e.py"

cd "$OPENPI_DIR"
uv run python "$SERVE_SCRIPT" "$@"
