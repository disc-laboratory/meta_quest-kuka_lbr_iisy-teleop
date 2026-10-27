#!/usr/bin/env bash
# configure | activate | deactivate | cleanup | state  for /robot_manager.
# Activating on HARDWARE releases brakes and starts external control: typed confirmation required.
set -eo pipefail
cmd=${1:?usage: robot_lifecycle.sh configure|activate|deactivate|cleanup|state}
[[ -n ${ROS_DOMAIN_ID:-} ]] || echo "WARNING: ROS_DOMAIN_ID is not set (default 0). Hardware sessions use: export ROS_DOMAIN_ID=78"
timeout 5 ros2 lifecycle get /robot_manager >/dev/null 2>&1 || { echo "robot_manager not visible on ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}: is robot.launch.py running in this domain?"; exit 1; }
case $cmd in
  state) exec ros2 lifecycle get /robot_manager ;;
  activate)
    mode=$(ros2 param get /robot_manager robot_model >/dev/null 2>&1; timeout 5 ros2 topic echo --once /robot_description std_msgs/msg/String 2>/dev/null | grep -o 'KukaMockHardwareInterface' || true)
    if [[ -z $mode ]]; then
      echo "HARDWARE activation: brakes release, robot follows external commands."
      echo "Operator at pendant with enabling switch, E-stop in reach, workspace clear."
      read -r -p "Type ACTIVATE to continue: " ok; [[ $ok == ACTIVATE ]] || { echo aborted; exit 1; }
    fi ;;
esac
ros2 lifecycle set /robot_manager "$cmd"
ros2 lifecycle get /robot_manager
