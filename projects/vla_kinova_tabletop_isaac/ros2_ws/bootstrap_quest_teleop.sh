#!/bin/bash

# This bash scripts imports and builds the necessary repositories to get the Quest 3 teleoperation stack to work with the kinova.
# Run after the bootstrap_kinova_ws.sh script, only if you actually need to teleoperate the robot, you can skip this otherwise.

set -e

export DEBIAN_FRONTEND=noninteractive
export NEEDRESTART_MODE=a

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

ROS_DISTRO="${ROS_DISTRO:-humble}"

echo ""
echo "==> Importing teleop repos (ROS-TCP-Endpoint, Quest2ROS2)..."
vcs import src --skip-existing --input teleop_deps.repos
echo ""

# Generate the quest2ros message package. Skip if it already exists so re-runs are safe (ros2 pkg create would otherwise error on the existing directory).
echo ""
if [ ! -d src/quest2ros ]; then
  echo "==> Generating quest2ros message package..."
  ( cd src && ros2 pkg create --build-type ament_cmake quest2ros )
  cp -r src/Quest2ROS2/Files_for_msg_pkg/* src/quest2ros/
else
  echo "==> src/quest2ros already exists, skipping generation."
fi
echo ""

echo ""
echo "==> Installing teleop system dependencies..."
sudo apt-get update && sudo apt-get install -y ros-$ROS_DISTRO-tf-transformations
rosdep install --from-paths src --ignore-src -r -y
echo ""

echo ""
echo "==> Building workspace..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3
echo ""

echo ""
echo "==> Teleop stack ready. Source with:"
echo "    source ~/isaac-projects/projects/vla_kinova_tabletop_isaac/setup.bash"
