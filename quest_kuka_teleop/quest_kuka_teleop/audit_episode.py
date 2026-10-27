"""Timing audit of one recorded episode bag: rates, gaps, header-vs-receive age."""
import argparse
import json
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('episode', type=Path)
    args = ap.parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.episode / 'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    recv, lag = {}, {}
    while reader.has_next():
        topic, data, t = reader.read_next()
        recv.setdefault(topic, []).append(t)
        if topic in ('/joint_states', '/right_controller_odom', '/servo_node/pose_target_cmds',
                     '/joint_trajectory_controller/joint_trajectory') or 'image' in topic:
            m = deserialize_message(data, get_message(types[topic]))
            st = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            if st:
                lag.setdefault(topic, []).append((t - st) * 1e-6)
    rep = {}
    for k, ts in sorted(recv.items()):
        ts = np.array(ts)
        d = np.diff(ts) * 1e-6 if len(ts) > 1 else np.array([np.nan])
        r = dict(type=types[k], count=len(ts), duration_s=float((ts[-1] - ts[0]) * 1e-9),
                 mean_hz=float(len(ts) / max((ts[-1] - ts[0]) * 1e-9, 1e-9)),
                 dt_ms_p50=float(np.nanpercentile(d, 50)), dt_ms_p99=float(np.nanpercentile(d, 99)),
                 dt_ms_max=float(np.nanmax(d)))
        if k in lag:
            a = np.array(lag[k])
            r.update(receive_minus_header_ms_p50=float(np.percentile(a, 50)),
                     receive_minus_header_ms_p99=float(np.percentile(a, 99)))
        rep[k] = r
    out = args.episode / 'audit.json'
    out.write_text(json.dumps(rep, indent=2))
    for k, r in rep.items():
        print(f"{k:55s} n={r['count']:6d} {r['mean_hz']:7.1f}Hz p99dt={r['dt_ms_p99']:7.1f}ms max={r['dt_ms_max']:7.1f}ms"
              + (f" lag50={r['receive_minus_header_ms_p50']:.1f}ms" if 'receive_minus_header_ms_p50' in r else ''))
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
