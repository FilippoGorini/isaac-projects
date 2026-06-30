#!/bin/bash
set -e

# Bootstrap for the openpi client machine, which only talks to the remote server serving the policy over websockets, 
# so we don't need anything from the training stack: no repo, no uv venv, no jax and no checkpoint download.
# Therefore we only need to install the openpi-client package

# openpi-client must be importable by the python that runs the ros2 side client node, therefore we install it on system python, no venv

# NOTE: openpi-client requires numpy<2.0 which conflicts with pyzed package for the zed camera. For now we're fine as we abandoned the zed
# path, if we later want to use it again we can instead install the client into a venv and point the node to it instead of system python

echo "==> Installing system dependencies (python3-pip)..."
sudo apt-get update -y
sudo apt-get install -y python3-pip

echo "==> Installing openpi-client into system Python..."
# typing-extensions is a transitive dep required by openpi-client which was omitted in its metadata on python<3.12
python3 -m pip install openpi-client typing-extensions

echo ""
echo "==> openpi-client ready (system Python)"
