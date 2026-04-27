#!/bin/bash

# This bash script pulls the ros2_kortex package and all of its dependencies into the workspace and then builds it
# Run once for server

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Imports the ros2_kortex package
echo ""
echo "==> Importing top-level deps from deps.repos..."
vcs import src --skip-existing --input deps.repos
echo ""

# Imports the ros2_kortex package dependencies
echo ""
echo "==> Importing ros2_kortex transitive deps..."
vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.$ROS_DISTRO.repos
vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.$ROS_DISTRO.repos
echo ""


echo ""
echo "==> Installing system dependencies via rosdep..."
# Explicitly install topic-based-ros2-control plugin for isaac, as it is not handled by rosdep (not present in the manifests of the kortex packages)
sudo apt-get update && sudo apt-get install -y ros-$ROS_DISTRO-topic-based-ros2-control
# Imports system dependencies
rosdep install --from-paths src --ignore-src -r -y
echo ""

# Build workspace
echo ""
echo "==> Building workspace..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3
echo ""

echo ""
echo "==> Imported ros2_kortex packages, installed their dependencies and built the workspace, source with:"
echo "    source ~/isaac-projects/projects/vla_kinova_tabletop_isaac/setup.bash"

