# Recording demonstrations

Run in a separate terminal while teleop is running (after the per-terminal setup).

## Episode recorder

Fill `quest_kuka_teleop/config/session.yaml` first (operator, task, camera topics).

```bash
SHARE=$(ros2 pkg prefix --share quest_kuka_teleop)
ros2 run quest_kuka_teleop episode_recorder --mode hardware \
  --root data/physical/pilot_001 \
  --session $SHARE/config/session.yaml \
  --tcp $SHARE/config/tcp.yaml
```

| Key (+ Enter) | Action |
| --- | --- |
| Enter | Start a new episode |
| `s` / `f` / `d` | Stop and label success / failure / discard |
| `q` | Quit (when idle) |

Each episode is saved as `episode_NNNN/` with `bag/` (MCAP) and `meta.json`.

Check timing of an episode:

```bash
ros2 run quest_kuka_teleop audit_episode data/physical/pilot_001/episode_0000
```

## Quick bag (motion only)

```bash
ros2 bag record -s mcap -o "data/physical/demo_$(date +%Y%m%d_%H%M%S)" \
  --topics /joint_states /tf /tf_static \
  /right_controller_odom /right_grip_button_state \
  /teleop/hand_intent /servo_node/pose_target_cmds \
  /joint_trajectory_controller/joint_trajectory \
  /joint_trajectory_controller/controller_state \
  /servo_node/status /teleop/teleop_state \
  /teleop/gripper_command /teleop/gripper_state
```

Ctrl+C saves the bag. No camera images are recorded unless you add their topics.

## Diagnostics (`tools/`)

All are read-only: they publish nothing.

| Command | Output |
| --- | --- |
| `python3 tools/monitor.py --seconds 60 --out reports/input.jsonl` | Quest input log and per-topic rates |
| `python3 tools/flow_audit.py --seconds 30 --out reports/flow.jsonl` | Input → target → Servo → joint timing, gaps over 150 ms |
| `python3 tools/preflight.py --out reports/preflight.json` | Installed ROS packages, nodes, topics, services |
