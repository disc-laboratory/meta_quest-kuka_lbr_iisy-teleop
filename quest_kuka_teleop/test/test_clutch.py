import unittest
import numpy as np
from scipy.spatial.transform import Rotation
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from quest_kuka_teleop.clutch import Clutch  # noqa: E402


class Tests(unittest.TestCase):
    def setUp(self):
        self.c = Clutch(np.eye(3))
        self.q = [0, 0, 0, 1]

    def call(self, p=(0, 0, 0), held=True, age=0, q=None):
        return self.c.update(p, self.q if q is None else q, [1, 2, 3], self.q,
                             held=held, tracking_valid=True, age=age)

    def test_start_requires_release_and_press(self):
        self.assertIsNone(self.call())
        self.call(held=False)
        np.testing.assert_allclose(self.call()[0], [1, 2, 3])

    def test_scaled_motion_and_reclutch(self):
        self.call(held=False); self.call()
        np.testing.assert_allclose(self.call([.1, 0, 0])[0], [1.03, 2, 3])
        self.call(held=False)
        np.testing.assert_allclose(self.call([2, 0, 0])[0], [1, 2, 3])

    def test_stale_input_requires_new_release(self):
        self.call(held=False); self.call()
        self.assertIsNone(self.call(age=.2))
        self.assertIsNone(self.call())
        self.call(held=False)
        self.assertIsNotNone(self.call())

    def test_tracking_jump_latches(self):
        self.call(held=False); self.call()
        self.assertIsNone(self.call([1, 0, 0]))
        self.assertIsNone(self.call())

    def test_orientation_and_frame_mapping(self):
        self.c = Clutch(Rotation.from_euler('z', 90, degrees=True).as_matrix())
        self.call(held=False); self.call()
        p, q = self.call([.1, 0, 0], q=Rotation.from_euler('x', .2).as_quat())
        np.testing.assert_allclose(p, [1, 2.03, 3])
        np.testing.assert_allclose(Rotation.from_quat(q).as_rotvec(), [0, .2, 0], atol=1e-10)

    def test_invalid_inputs(self):
        with self.assertRaises(ValueError):
            Clutch(np.diag([-1, 1, 1]))
        self.call(held=False)
        with self.assertRaises(ValueError):
            self.call(q=[0, 0, 0, 0])
        self.assertIsNone(self.call())

    def test_translation_only_holds_anchor_orientation_and_reanchors(self):
        c = Clutch(np.eye(3), translation_only=True)
        initial = Rotation.from_euler('xyz', [.3, -.4, .5]).as_quat()
        moved = Rotation.from_euler('xyz', [.4, -.2, .6]).as_quat()
        def update(hand, tcp, held=True, age=0):
            return c.update([.1, 0, 0], hand, [1, 2, 3], tcp,
                            held=held, tracking_valid=True, age=age)
        update(self.q, initial, held=False)
        update(self.q, initial)
        _, q = update(moved, moved)
        np.testing.assert_allclose(q, initial, atol=1e-10)
        p, q = c.update([.2, 0, 0], moved, [1, 2, 3], moved,
                        held=True, tracking_valid=True, age=0)
        np.testing.assert_allclose(p, [1.03, 2, 3])
        np.testing.assert_allclose(q, initial, atol=1e-10)
        self.assertIsNone(update(moved, moved, age=.2))
        self.assertIsNone(update(moved, moved))
        update(moved, moved, held=False)
        np.testing.assert_allclose(update(moved, moved)[1], moved, atol=1e-10)


if __name__ == '__main__':
    unittest.main()
