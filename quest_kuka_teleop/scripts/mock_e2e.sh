#!/usr/bin/env bash
# Full chain on the KUKA MOCK hardware interface: driver(mock) -> activate -> Servo + bridge
# + fake Quest -> 40 s recorded episode -> audit. No connection to the real controller.
set -o pipefail
OUT=${1:-reports/quest/mock_e2e_$(date +%Y%m%d_%H%M%S)}
command -v ros2 >/dev/null || { echo "Source ROS 2 and the workspace first (see README)"; exit 1; }
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}   # isolate from any live robot graph
mkdir -p "$OUT"
ros2 launch quest_kuka_teleop robot.launch.py mode:=mock rt_prio:=0 lock_memory:=false >"$OUT/robot.log" 2>&1 & R=$!
trap 'trap - EXIT TERM; kill -TERM 0' EXIT TERM
sleep 8
ros2 lifecycle set /robot_manager configure && sleep 2 && ros2 lifecycle set /robot_manager activate || { echo activation failed; tail -30 "$OUT/robot.log"; exit 1; }
sleep 2; ros2 control list_controllers | tee "$OUT/controllers.txt"
# mock starts at all-zero joints (singular, arm straight up): go to a ready pose first
timeout 30 ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.0, -1.3, 1.3, 0.0, 1.35, 0.0], time_from_start: {sec: 6}}]}}" | tail -2
ros2 launch quest_kuka_teleop teleop.launch.py fake_quest:=true endpoint:=false gripper:=true gripper_backend:=mock >"$OUT/teleop.log" 2>&1 & T=$!
sleep 6
ros2 bag record -s mcap -o "$OUT/episode_0000/bag" --topics /joint_states /tf /right_controller_odom \
  /right_grip_button_state /teleop/hand_intent /servo_node/pose_target_cmds /servo_node/status \
  /joint_trajectory_controller/joint_trajectory /joint_trajectory_controller/controller_state \
  /teleop/teleop_state /teleop/gripper_command /teleop/gripper_state >"$OUT/bag.log" 2>&1 & B=$!
sleep 40; kill -TERM $B; wait $B
grep -E 'state:|POSE mode' "$OUT/teleop.log" | sort | uniq -c | sort -rn | head -20
ros2 run quest_kuka_teleop audit_episode "$OUT/episode_0000"
python3 - "$OUT" <<'PY'
import sys, json, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1]+'/episode_0000/bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
q = []
while r.has_next():
    t, d, _ = r.read_next()
    if t == '/joint_states': q.append(deserialize_message(d, JointState).position)
q = np.array(q); span = (q.max(0) - q.min(0)).tolist() if len(q) else []
print('joint position span (rad):', np.round(span, 4)); json.dump(dict(joint_span_rad=span), open(sys.argv[1]+'/joint_span.json', 'w'))
PY
echo "artifacts: $OUT"
