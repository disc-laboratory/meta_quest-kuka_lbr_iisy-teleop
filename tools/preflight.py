"""Read-only ROS inventory. Never starts controllers or publishes commands."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess


def query(command):
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=15)
        return dict(command=command, returncode=result.returncode,
                    stdout=result.stdout, stderr=result.stderr)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return dict(command=command, error=str(exc))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        parser.error('Output exists; choose a new filename')
    report = dict(ros_distro=os.getenv('ROS_DISTRO'),
                  ros_domain_id=os.getenv('ROS_DOMAIN_ID', '0'),
                  installed_distros=[p.name for p in Path('/opt/ros').glob('*')],
                  robot_control_started=False, queries=[])
    if shutil.which('ros2'):
        for command in (['ros2', 'pkg', 'list'], ['ros2', 'node', 'list'],
                        ['ros2', 'topic', 'list', '-t'], ['ros2', 'service', 'list', '-t']):
            report['queries'].append(query(command))
    else:
        report['error'] = 'ros2 not on PATH. Source the real robot ROS workspace first.'
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open('x') as stream:
        json.dump(report, stream, indent=2)
    print(f'Read-only inventory: {args.out.resolve()}')
    print('Also supply pendant software/version, installed external-control option, '
          'and physical gripper model/interface. This inventory cannot discover pendant licenses.')


if __name__ == '__main__':
    main()
