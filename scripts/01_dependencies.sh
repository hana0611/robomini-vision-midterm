#!/usr/bin/env bash
set -eo pipefail
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 24.04 ]]; then
  echo 'STOP: Ubuntu 24.04 required.'; exit 2
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'STOP: install ROS2 Jazzy using guide section 3B first.'; exit 3
fi
if [[ -n "${CONDA_PREFIX:-}" || -n "${VIRTUAL_ENV:-}" ]]; then
  echo 'STOP: deactivate Conda / venv first and open a clean system terminal.'; exit 4
fi
sudo apt-get update
sudo apt-get install -y git cmake build-essential curl gnupg lsb-release \
  python3-colcon-common-extensions python3-rosdep python3-opencv python3-numpy \
  ros-jazzy-cv-bridge ros-jazzy-ros-gz ros-jazzy-rqt-image-view \
  ros-jazzy-rosbag2-storage-mcap v4l-utils
# Upstream CMake calls find_package(gz-sim8), so supply the native dev libs.
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
printf 'deb [arch=%s signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main\n' \
  "$(dpkg --print-architecture)" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic libgz-sim8-dev libgz-transport13-dev \
  libgz-msgs10-dev libgz-math7-dev libgz-common5-dev
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -c 'import cv2, numpy, rclpy; from cv_bridge import CvBridge; print("Python imports OK", cv2.__version__)'
gz sim --versions
echo 'Dependencies ready. Next: bash scripts/02_build.sh'
