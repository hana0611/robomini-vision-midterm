# Source this file in EVERY new terminal. Do not run it with bash.
# Example: source ~/robomini_A/scripts/env.sh
if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "$HOME/robomini_sim/ros2_ws/install/setup.bash" ]]; then
  echo 'Missing setup. Finish dependency installation and build first.'
  return 2
fi
source /opt/ros/jazzy/setup.bash
source "$HOME/robomini_sim/ros2_ws/install/setup.bash"
export ROBOMINI_SIM_ROOT="$HOME/robomini_sim"
export ROS_DOMAIN_ID=27
export GZ_PARTITION=robomini_A
export PYTHONUNBUFFERED=1
cd "$HOME/robomini_A"
