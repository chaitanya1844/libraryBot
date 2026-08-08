#!/bin/bash
# Builds libraryBot from a clean Ubuntu 22.04 checkout: system deps, the
# FAST-LIO2 overlay workspace (not on apt, so built from source into
# ~/fastlio_ws), and this repo's ros2_ws. Safe to re-run -- every step
# skips work that's already done.
#
# Usage: ./build.sh [--skip-apt]
#   --skip-apt   skip `apt install`/rosdep (use when deps are already installed
#                or you don't have sudo)

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FASTLIO_WS="$HOME/fastlio_ws"
SKIP_APT=0
[[ "${1:-}" == "--skip-apt" ]] && SKIP_APT=1

echo "==> libraryBot build starting (repo: $REPO_DIR)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: ROS 2 Humble not found at /opt/ros/humble. Install ros-humble-desktop first:" >&2
  echo "  https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html" >&2
  exit 1
fi
source /opt/ros/humble/setup.bash

if [[ "$SKIP_APT" -eq 0 ]]; then
  echo "==> Installing system packages"
  sudo apt-get update
  sudo apt-get install -y \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-simple-commander \
    ros-humble-robot-localization ros-humble-realsense2-description \
    ros-humble-pointcloud-to-laserscan ros-humble-mecanum-drive-controller \
    python3-pyqt5 python3-opencv ros-humble-cv-bridge python3-yaml python3-pip \
    python3-rosdep qtbase5-dev git cmake build-essential

  sudo rosdep init 2>/dev/null || true
  rosdep update

  echo "==> Installing Python deps (open3d, tf_transformations, ros2_numpy)"
  pip3 install --user open3d tf_transformations ros2_numpy
fi

echo "==> Building FAST-LIO2 overlay workspace ($FASTLIO_WS)"
if [[ ! -d "$FASTLIO_WS/src/FAST_LIO" ]]; then
  mkdir -p "$FASTLIO_WS/src" && cd "$FASTLIO_WS/src"
  [[ -d Livox-SDK2 ]] || git clone https://github.com/Livox-SDK/Livox-SDK2.git
  [[ -d livox_ros_driver2 ]] || git clone https://github.com/Livox-SDK/livox_ros_driver2.git
  [[ -d FAST_LIO ]] || git clone --recursive --branch ROS2 https://github.com/hku-mars/FAST_LIO.git

  LIVOX_INSTALL_PREFIX="$FASTLIO_WS/livox_sdk2_install"
  mkdir -p Livox-SDK2/build && cd Livox-SDK2/build
  cmake -DCMAKE_INSTALL_PREFIX="$LIVOX_INSTALL_PREFIX" ..
  make -j"$(nproc)" install
  cd "$FASTLIO_WS/src"

  cp livox_ros_driver2/package_ROS2.xml livox_ros_driver2/package.xml
  cp -r livox_ros_driver2/launch_ROS2/. livox_ros_driver2/launch/
  # CMakeLists.txt hardcodes /usr/local for the SDK; point it at our prefix instead.
  sed -i "s#/usr/local#${LIVOX_INSTALL_PREFIX}#g" livox_ros_driver2/CMakeLists.txt
else
  echo "    already cloned, skipping"
fi

cd "$FASTLIO_WS"
if [[ ! -d install/fast_lio ]]; then
  colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
else
  echo "    already built, skipping (delete $FASTLIO_WS/install to force a rebuild)"
fi
source "$FASTLIO_WS/install/setup.bash"

echo "==> Building ros2_ws"
cd "$REPO_DIR/ros2_ws"
if [[ "$SKIP_APT" -eq 0 ]]; then
  rosdep install --from-paths src --ignore-src -r -y --skip-keys "fast_lio livox_ros_driver2"
fi
colcon build --symlink-install
source install/setup.bash

echo "==> Build complete."
echo ""
echo "Every new terminal, before running anything:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/fastlio_ws/install/setup.bash"
echo "  source $REPO_DIR/ros2_ws/install/setup.bash"
echo ""
echo "Then: ros2 launch libbot_bringup simulation.launch.py headless:=false rviz:=true"
