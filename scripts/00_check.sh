#!/usr/bin/env bash
# Read-only check; run in Ubuntu, not Windows PowerShell.
set -o pipefail
printf '\n=== Ubuntu version ===\n'
cat /etc/os-release
printf '\n=== Installed ROS directories ===\n'
ls /opt/ros 2>/dev/null || true
printf '\n=== Current shell ROS ===\n'
printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-not sourced}"
printf '\n=== Python ===\n'
command -v python3
/usr/bin/python3 --version
printf '\n=== Graphics ===\n'
lspci | grep -Ei 'vga|3d|display' || true
nvidia-smi 2>/dev/null || true
printf '\n=== Gazebo ===\n'
gz sim --versions 2>/dev/null || true
printf '\n=== Disk / RAM ===\n'
df -h "$HOME"
free -h
printf '\n=== Persistent ROS source lines ===\n'
grep -nE '/opt/ros|setup.bash|conda' "$HOME/.bashrc" 2>/dev/null || true
printf '\n=== Assessment compatibility ===\n'
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 24.04 ]]; then
  echo 'STOP: this guide targets Ubuntu 24.04 + ROS2 Jazzy. Do not mix releases.'
  exit 2
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'Ubuntu OK; Jazzy setup missing. Follow guide section 3B.'
  exit 3
fi
source /opt/ros/jazzy/setup.bash
printf 'Ubuntu 24.04 + Jazzy: OK\n'
ros2 doctor --report 2>/dev/null | head -n 60 || true
