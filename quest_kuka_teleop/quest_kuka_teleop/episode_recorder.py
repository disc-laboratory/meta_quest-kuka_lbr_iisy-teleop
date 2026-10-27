"""Episode-segmented raw recording with rosbag2/MCAP plus meta.json.

Terminal-driven (an assistant at the PC): Enter = start; while recording:
  s = success, f = failure, d = discard (kept, labelled invalid), Ctrl+C = abort.
(Headset buttons are not used: the PickNik app reserves Left A for its IP keyboard.)
Raw streams only: no resampling, no overlays. Bags keep receive timestamps,
messages keep their source header stamps. Timing is audited separately.
"""
import argparse
import datetime
import json
import os
import select
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml

TOPICS = [
    '/joint_states', '/tf', '/tf_static', '/robot_description',
    '/right_controller_odom', '/left_controller_odom',
    '/right_grip_button_state', '/right_trigger_button_state',
    '/left_grip_button_state', '/left_trigger_button_state',
    '/left_a_button_state', '/left_b_button_state',
    '/teleop/hand_intent', '/servo_node/pose_target_cmds',          # Cartesian intent / bounded target
    '/joint_trajectory_controller/joint_trajectory',             # issued joint command after IK/limits
    '/joint_trajectory_controller/controller_state',             # reference vs feedback
    '/servo_node/status', '/teleop/teleop_state', '/teleop/gripper_command',
    '/teleop/gripper_state',                                       # only if a real feedback source exists
]
REGEX = r'/(event_broadcaster|control_mode_handler|robot_manager)/.*'


def versions():
    pk = ['ros-jazzy-kuka-iiqka-eac-driver', 'ros-jazzy-moveit-servo', 'ros-jazzy-kuka-lbr-iisy-support']
    out = subprocess.run(['dpkg-query', '-W', '-f=${Package} ${Version}\n', *pk], capture_output=True, text=True)
    return dict(l.split(' ', 1) for l in out.stdout.splitlines() if ' ' in l)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--root', type=Path, required=True, help='dataset root, e.g. data/physical/pilot_001')
    ap.add_argument('--session', type=Path, required=True, help='session.yaml')
    ap.add_argument('--tcp', type=Path, required=True, help='tcp.yaml')
    ap.add_argument('--mode', choices=['mock', 'hardware'], required=True)
    args = ap.parse_args()
    session = yaml.safe_load(args.session.read_text())
    tcp = yaml.safe_load(args.tcp.read_text())
    topics = TOPICS + list(session.get('camera_topics') or [])
    args.root.mkdir(parents=True, exist_ok=True)
    print(f'Dataset root {args.root}. Enter = start episode, q+Enter = quit.')
    while True:
        if input().strip() == 'q':
            return 0
        n = len([d for d in args.root.iterdir() if d.name.startswith('episode_')])
        ep = args.root / f'episode_{n:04d}'
        ep.mkdir()
        start = dict(wall=datetime.datetime.now().astimezone().isoformat(), ros_ns=time.time_ns())
        proc = subprocess.Popen(['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(ep / 'bag'),
                                 '--include-unpublished-topics', '--topics', *topics, '-e', REGEX],
                                stdout=open(ep / 'record.log', 'w'), stderr=subprocess.STDOUT)
        print(f'RECORDING {ep.name}: s=success f=failure d=discard (+Enter)')
        label = None
        try:
            while label is None:
                if proc.poll() is not None:
                    label = 'recorder_died'
                    break
                r, _, _ = select.select([sys.stdin], [], [], .5)
                if r:
                    k = sys.stdin.readline().strip()
                    label = {'s': 'success', 'f': 'failure', 'd': 'discard'}.get(k)
        except KeyboardInterrupt:
            label = 'aborted'
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=20)
        note = input('Note (optional): ')
        meta = dict(episode=ep.name, mode=args.mode, start=start,
                    end=dict(wall=datetime.datetime.now().astimezone().isoformat(), ros_ns=time.time_ns()),
                    outcome=label, valid_for_training=label == 'success', operator_note=note,
                    session=session, tcp=tcp, topics_requested=topics, versions=versions(),
                    gripper_feedback='absent' if '/teleop/gripper_state' not in topics else 'see bag',
                    schema='physical_raw_v0')
        (ep / 'meta.json').write_text(json.dumps(meta, indent=2))
        print(f'saved {ep} outcome={label}. Run audit_episode on it. Enter = next.')


if __name__ == '__main__':
    raise SystemExit(main())
