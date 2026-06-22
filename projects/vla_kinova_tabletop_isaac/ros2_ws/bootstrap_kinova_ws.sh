#!/bin/bash

# This bash script pulls the ros2_kortex packages (ros2_kortex_vision as well) and all of its dependencies into the workspace and then builds it
# Run once per machine

set -e

export DEBIAN_FRONTEND=noninteractive
export NEEDRESTART_MODE=a

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Imports the ros2_kortex package
echo ""
echo "==> Importing top-level deps from kinova_deps.repos..."
vcs import src --skip-existing --input kinova_deps.repos
echo ""

# Imports the ros2_kortex package dependencies
echo ""
echo "==> Importing ros2_kortex transitive deps..."
vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.$ROS_DISTRO.repos
vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.$ROS_DISTRO.repos
echo ""


echo ""
echo "==> Installing apt system packages (control + cameras)..."
sudo apt-get update
# Explicitly install topic-based-ros2-control plugin for isaac, as it is not handled by rosdep (not present in the manifests of the kortex packages)
sudo apt-get install -y ros-$ROS_DISTRO-topic-based-ros2-control

# apt install the realsense ros2 libraries for RealSense D435
sudo apt-get install -y "ros-$ROS_DISTRO-librealsense2*"
sudo apt-get install -y "ros-$ROS_DISTRO-realsense2-*"

# Explicitly install the GStreamer runtime for the kinova vision package to work (not declared in the package.xml)
sudo apt-get install -y \
  gstreamer1.0-tools gstreamer1.0-libav \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev
echo ""

echo ""
echo "==> Installing remaining system dependencies via rosdep..."
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

