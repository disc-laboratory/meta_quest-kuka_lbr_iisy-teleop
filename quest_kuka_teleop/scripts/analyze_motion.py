"""TCP speed / tracking / continuity from a bag with /tf, /teleop/teleop_state, /teleop/hand_intent."""
import json
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
# TCP from joint_states via FK is overkill here: rebuild link_6 chain from /tf is heavy, so use
# the commanded-vs-intent streams plus /tf of tool0 if a direct base_link->tool0 is absent.
fk_t, fk_p, it_t, it_p, cm_t, cm_p, eng = [], [], [], [], [], [], []
chain = {}
while r.has_next():
    topic, data, t = r.read_next()
    if topic == '/tf':
        for tr in deserialize_message(data, TFMessage).transforms:
            chain[tr.child_frame_id] = (tr.header.frame_id, tr.transform)
        if 'link_6' in chain:
            # compose base_link -> link_6 from the latest transforms
            from scipy.spatial.transform import Rotation as R
            T = np.eye(4); f = 'link_6'; ok = True
            while f != 'base_link':
                if f not in chain:
                    ok = False; break
                parent, x = chain[f]
                M = np.eye(4); M[:3, :3] = R.from_quat([x.rotation.x, x.rotation.y, x.rotation.z, x.rotation.w]).as_matrix()
                M[:3, 3] = [x.translation.x, x.translation.y, x.translation.z]
                T = M @ T; f = parent
            if ok:
                fk_t.append(t * 1e-9); fk_p.append(T[:3, 3].copy())
    elif topic in ('/teleop/hand_intent', '/servo_node/pose_target_cmds'):
        m = deserialize_message(data, PoseStamped).pose.position
        (it_t, it_p) if topic == '/teleop/hand_intent' else (cm_t, cm_p)
        ((it_t if topic == '/teleop/hand_intent' else cm_t)).append(t * 1e-9)
        ((it_p if topic == '/teleop/hand_intent' else cm_p)).append([m.x, m.y, m.z])
    elif topic == '/teleop/teleop_state':
        eng.append((t * 1e-9, json.loads(deserialize_message(data, String).data)['engaged']))

fk_t, fk_p = np.array(fk_t), np.array(fk_p)
# resample link_6 path at 50 Hz to get a clean speed estimate
g = np.arange(fk_t[0], fk_t[-1], .02)
P = np.stack([np.interp(g, fk_t, fk_p[:, i]) for i in range(3)], 1)
v = np.linalg.norm(np.diff(P, axis=0), axis=1) / .02
e = np.array(eng)
engaged_frac = e[:, 1].mean() if len(e) else 0
toggles = int(np.abs(np.diff(e[:, 1])).sum()) if len(e) > 1 else 0
print(f'link_6 speed m/s: p50 {np.percentile(v, 50):.3f}  p90 {np.percentile(v, 90):.3f}  max {v.max():.3f}')
print(f'path length {np.sum(v) * .02:.3f} m over {g[-1] - g[0]:.1f} s; engaged {engaged_frac * 100:.0f} % of ticks, engage/disengage transitions {toggles}')
if it_p and cm_p:
    it_t, it_p, cm_t, cm_p = map(np.array, (it_t, it_p, cm_t, cm_p))
    # tracking error of the robot vs hand intent while engaged (link_6, so includes fixed TCP offset bias)
    ip = np.stack([np.interp(fk_t, it_t, it_p[:, i]) for i in range(3)], 1)
    m = (fk_t > it_t[0]) & (fk_t < it_t[-1])
    print(f'intent path length {np.sum(np.linalg.norm(np.diff(it_p, axis=0), axis=1)):.3f} m')
