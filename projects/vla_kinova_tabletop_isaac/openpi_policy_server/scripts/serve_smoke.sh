#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OPENPI_DIR="$(cd "$SCRIPT_DIR/../../external/openpi" && pwd)"

cd "$OPENPI_DIR"
uv run scripts/serve_policy.py --env ALOHA_SIM
