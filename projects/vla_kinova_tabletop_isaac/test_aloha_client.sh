#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OPENPI_DIR="$(cd "$SCRIPT_DIR/external/openpi" && pwd)"

HOST="${1:-localhost}"
PORT="${2:-8000}"

cd "$OPENPI_DIR"
uv run examples/simple_client/main.py --env ALOHA_SIM --host "$HOST" --port "$PORT"

# This script runs a simple client example provided by openpi, which sends 20 observations to the server and collects the returned action chunks, while monitoring latency
# Meant to be run on the machine collecting images from the robot and controlling it.

# The simple client script does the following:
# 1. Creates a WebsocketClientPolicy and connects to the server at localhost:8000
# 2. Sends 2 warmup observations (not timed): these trigger JAX JIT compilation on the first run, which is why the first run times out and you might have to execute the script again. On subsequent runs these just verify the connection is alive
# 3. Each observation is a randomly generated ALOHA-shaped dict:
#   - state: 14 joint positions (np.ones((14,))) — ALOHA has 2 arms × 7 joints
#   - images: 4 cameras (cam_high, cam_low, cam_left_wrist, cam_right_wrist), each random uint8 pixels at 224×224
#   - prompt: "do something"
# 4. Runs 20 timed inferences in a loop — each one sends the observation dict over WebSocket, waits for the response, and records how long it took
# 5. The server responds to each observation with an action chunk: a (50, 14) array: 50 future timesteps × 14 joint positions (one for each ALOHA joint). The client receives this but discards it since this is just a timing test
# 6. Prints the timing table with mean, std, and percentiles
