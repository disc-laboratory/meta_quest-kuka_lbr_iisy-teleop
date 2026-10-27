#!/usr/bin/env bash
# Run a bridge/servo PROFILE on the KUKA MOCK (domain 77) with synthetic Quest input and
# report achieved TCP speed and continuity. Usage: profile_mock_test.sh <profile_dir> [radius_m]
set -o pipefail
PROF=$(realpath "$1"); RADIUS=${2:-0.3}
OUT=$PROF/mock_test_$(date +%H%M%S)
command -v ros2 >/dev/null || { echo "Source ROS 2 and the workspace first (see README)"; exit 1; }
export ROS_DOMAIN_ID=77
mkdir -p "$OUT"; trap 'trap - EXIT TERM; kill -TERM 0' EXIT TERM
ros2 launch quest_kuka_teleop robot.launch.py mode:=mock rt_prio:=0 lock_memory:=false >"$OUT/robot.log" 2>&1 &
timeout 90 bash -c "until [ \$(ros2 control list_controllers 2>/dev/null | wc -l) -ge 6 ]; do sleep 1; done"; sleep 1
ros2 lifecycle set /robot_manager configure && sleep 2 && ros2 lifecycle set /robot_manager activate || { echo "activation failed"; exit 1; }
timeout 30 ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.2238, -1.5723, 1.6803, -0.2975, 1.1132, 0.4401], time_from_start: {sec: 6}}]}}" | tail -1
ros2 launch quest_kuka_teleop teleop.launch.py endpoint:=false gripper:=false \
  bridge_config:=$PROF/bridge.yaml servo_config:=$PROF/servo.yaml workspace_collision_model:=true \
  tool_envelope_radius:=0.09 tool_envelope_length:=0.166 table_top_z:=-0.05 >"$OUT/teleop.log" 2>&1 &
sleep 6
ros2 run quest_kuka_teleop fake_quest --ros-args -p radius:=$RADIUS -p period:=${PERIOD:-8.0} >"$OUT/fake.log" 2>&1 &
sleep 2
ros2 bag record -s mcap -o "$OUT/bag" --topics /tf /joint_states /teleop/teleop_state /servo_node/status \
  /servo_node/pose_target_cmds /teleop/hand_intent >"$OUT/bag.log" 2>&1 & B=$!
sleep 30; kill -TERM $B; wait $B
python3 "$(dirname "$0")/analyze_motion.py" "$OUT/bag" | tee "$OUT/motion.txt"
grep -oE 'state: [a-z_:0-9]+' "$OUT/teleop.log" | sort | uniq -c | tee -a "$OUT/motion.txt"
echo "artifacts: $OUT"
