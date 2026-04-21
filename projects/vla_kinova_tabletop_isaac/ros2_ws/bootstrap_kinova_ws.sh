#!/bin/bash

# This bash script pulls the ros2_kortex package and all of its dependencies into the workspace and then builds it
# Run once for server

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Imports the ros2_kortex package
echo "==> Importing top-level deps from deps.repos..."
vcs import src --skip-existing --input deps.repos

# Imports the ros2_kortex package dependencies
echo "==> Importing ros2_kortex transitive deps..."
vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.$ROS_DISTRO.repos
vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.$ROS_DISTRO.repos

# Imports system dependencies
echo "==> Installing system dependencies via rosdep..."
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
echo "==> Building workspace..."
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3

echo "==> Done: imported ros2_kortex packages and built the workspace, source with:"
echo "    source ~/isaac-projects/projects/vla_kinova_tabletop_isaac/setup.bash"
