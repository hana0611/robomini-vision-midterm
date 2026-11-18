#!/usr/bin/env bash
set -eo pipefail
PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_ROOT="$HOME/robomini_sim"
if [[ -e "$SIM_ROOT" ]]; then
  echo "$SIM_ROOT already exists. Keeping all existing files."
else
  # Copy the exact inspected upstream snapshot; internet is not needed here.
  cp -a "$PACKAGE_ROOT/upstream/robomini_sim" "$SIM_ROOT"
  echo "Copied pinned upstream snapshot to $SIM_ROOT"
fi
if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "$SIM_ROOT/worlds/robomini.sdf" ]]; then
  echo 'STOP: missing Jazzy or incomplete robomini_sim directory.'; exit 2
fi
source /opt/ros/jazzy/setup.bash
export ROBOMINI_SIM_ROOT="$SIM_ROOT"
# Do not use all CPU cores on a small-RAM machine.
cmake -S "$SIM_ROOT/gz_plugins" -B "$SIM_ROOT/gz_plugins/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$SIM_ROOT/gz_plugins/build" -j2
cmake --install "$SIM_ROOT/gz_plugins/build"
cd "$SIM_ROOT/ros2_ws"
colcon build --symlink-install --executor sequential
source install/setup.bash
ros2 pkg prefix robomini_sim
ros2 interface show robomini_interfaces/msg/VisionToGimbal
echo 'Build done. Leave this terminal open, then follow guide section 5.'
