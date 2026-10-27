#!/usr/bin/env bash
# Real Quest -> bridge -> Servo -> KUKA MOCK hardware, with RViz. No real robot connection.
set -o pipefail
OUT=${1:-reports/quest/quest_mock_$(date +%Y%m%d_%H%M%S)}
command -v ros2 >/dev/null || { echo "Source ROS 2 and the workspace first (see README)"; exit 1; }
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_MOCK:-77}   # never share a graph with a real driver
mkdir -p "$OUT"; trap 'trap - EXIT TERM; kill -TERM 0' EXIT TERM
ros2 launch quest_kuka_teleop robot.launch.py mode:=mock rt_prio:=0 lock_memory:=false >"$OUT/robot.log" 2>&1 &
timeout 90 bash -c "until [ \$(ros2 control list_controllers 2>/dev/null | wc -l) -ge 6 ]; do sleep 1; done"; sleep 1
ros2 lifecycle set /robot_manager configure && sleep 2 && ros2 lifecycle set /robot_manager activate || { echo "activation failed"; exit 1; }
sleep 2
timeout 30 ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.0, -1.3, 1.3, 0.0, 1.35, 0.0], time_from_start: {sec: 6}}]}}" | tail -1
ros2 launch quest_kuka_teleop teleop.launch.py endpoint:=true gripper:=true gripper_backend:=mock >"$OUT/teleop.log" 2>&1 &
QT_QPA_PLATFORM=xcb rviz2 -d "$(ros2 pkg prefix --share quest_kuka_teleop)/config/view.rviz" >"$OUT/rviz.log" 2>&1 &
echo "READY: $OUT"
wait
