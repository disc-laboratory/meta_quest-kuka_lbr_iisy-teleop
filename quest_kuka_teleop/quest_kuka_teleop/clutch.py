"""Pure relative-pose mapping, independent of ROS and robot drivers.

Input positions are metres; quaternions are xyzw. base_from_quest is a proper
rotation mapping ALREADY right-handed Quest coordinates to robot base axes.
Not a collision checker, robot controller, or certified safety mechanism.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def pose(position, quaternion):
    p, q = np.asarray(position, float), np.asarray(quaternion, float)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(np.r_[p, q]).all():
        raise ValueError('Expected finite position[3] and quaternion_xyzw[4]')
    if abs(np.linalg.norm(q) - 1.) > .01:
        raise ValueError('Quaternion must be normalized')
    return p.copy(), Rotation.from_quat(q)


class Clutch:
    def __init__(self, base_from_quest, scale=.3, max_age=.15,
                 max_translation_jump=.15, max_rotation_jump=.8, translation_only=False):
        m = np.asarray(base_from_quest, float)
        if m.shape != (3, 3) or not np.isfinite(m).all() or not np.allclose(m.T @ m, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(m), 1.):
            raise ValueError('base_from_quest must be a proper rotation')
        for x in (scale, max_age, max_translation_jump, max_rotation_jump):
            if not np.isfinite(x) or x <= 0:
                raise ValueError('Limits must be positive and finite')
        self.mapping = Rotation.from_matrix(m)
        self.translation_only = translation_only
        self.scale, self.max_age = scale, max_age
        self.max_translation_jump, self.max_rotation_jump = max_translation_jump, max_rotation_jump
        self.anchor = None
        self.previous = None
        # A release must be observed before startup or re-arming after a fault.
        self.armed = False

    def reset(self):
        self.anchor = self.previous = None
        self.armed = False

    def update(self, hand_position, hand_quaternion, tcp_position, tcp_quaternion,
               *, held, tracking_valid, age):
        """Return requested (position, xyzw), or None to request no motion.

        age must include pose AND button freshness. The caller must additionally
        bound transport delay, verify robot feedback freshness, and implement
        controller-specific stopping; returning None does not stop hardware.
        """
        if not tracking_valid or not np.isfinite(age) or age < 0 or age > self.max_age:
            self.reset()
            return None
        if not held:
            self.anchor = self.previous = None
            self.armed = True
            return None
        if not self.armed:
            return None
        try:
            hp, hr = pose(hand_position, hand_quaternion)
            tp, tr = pose(tcp_position, tcp_quaternion)
        except ValueError:
            self.reset()
            raise
        if self.anchor is None:
            self.anchor = (hp, hr, tp, tr)
        if self.previous is not None:
            pp, pr = self.previous
            if np.linalg.norm(hp - pp) > self.max_translation_jump or (hr * pr.inv()).magnitude() > self.max_rotation_jump:
                self.reset()
                return None
        self.previous = hp, hr
        h0, r0, t0, rt0 = self.anchor
        target_p = t0 + self.scale * self.mapping.apply(hp - h0)
        target_r = rt0 if self.translation_only else self.mapping * (hr * r0.inv()) * self.mapping.inv() * rt0
        return target_p, target_r.as_quat()
