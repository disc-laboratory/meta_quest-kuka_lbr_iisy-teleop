"""Read-only timing/flow audit. Run in the same ROS domain as teleoperation.

Writes raw input, target, Servo status, feedback and bridge state to JSONL.
Does not publish commands or change parameters.
"""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import time


def finite_values(value):
    if isinstance(value, dict):
        return {k: finite_values(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_values(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def summarize(rows):
    stamps = defaultdict(list)
    states, servo = Counter(), Counter()
    for r in rows:
        stamps[r['topic']].append(r['receive_monotonic'])
        if r['topic'] == '/teleop/teleop_state':
            states[json.loads(r['message']['data'])['reason']] += 1
        elif r['topic'] == '/servo_node/status':
            servo[str(r['message']['code'])] += 1
    topics = {}
    for topic, ts in stamps.items():
        gaps = sorted(b-a for a, b in zip(ts, ts[1:]))
        topics[topic] = dict(messages=len(ts),
            active_hz=(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 and ts[-1]>ts[0] else None,
            max_gap_s=max(gaps, default=0), gaps_over_150ms=sum(g>.15 for g in gaps))
    return dict(topics=topics, bridge_state_samples=dict(states), servo_status_samples=dict(servo))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seconds', type=float, default=30)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    if not 0 < args.seconds <= 120:
        ap.error('seconds must be in (0, 120]')
    summary_path = args.out.with_suffix('.summary.json')
    if args.out.exists() or summary_path.exists():
        ap.error('choose a fresh output filename')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import ServoStatus
    from trajectory_msgs.msg import JointTrajectory
    rclpy.init()
    node = rclpy.create_node('flow_audit')
    rows, subs = [], []
    args.out.parent.mkdir(parents=True, exist_ok=True)
    try:
        with args.out.open('x') as stream:
            def save(msg, topic):
                row = dict(topic=topic, receive_monotonic=time.monotonic(),
                           message=message_to_ordereddict(msg))
                # Joint velocity may be NaN when unavailable: retain it as null.
                encoded = json.dumps(finite_values(row), allow_nan=False)
                stream.write(encoded+'\n')
                rows.append(json.loads(encoded))
            for topic, cls in [('/right_controller_odom', Odometry),
                    ('/right_grip_button_state', Bool), ('/joint_states', JointState),
                    ('/teleop/hand_intent', PoseStamped), ('/servo_node/pose_target_cmds', PoseStamped),
                    ('/joint_trajectory_controller/joint_trajectory', JointTrajectory),
                    ('/teleop/teleop_state', String), ('/servo_node/status', ServoStatus)]:
                subs.append(node.create_subscription(cls, topic,
                    lambda m, t=topic: save(m, t), qos_profile_sensor_data))
            print('Read-only audit started. Use only the already commissioned motion settings.', flush=True)
            start = time.monotonic()
            while time.monotonic()-start < args.seconds:
                rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    result = summarize(rows)
    summary_path.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return 0 if rows else 1


if __name__ == '__main__':
    raise SystemExit(main())
