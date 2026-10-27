"""Protocol and pulse lifecycle tests; never opens a physical serial device."""
import importlib.util
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[1] / 'quest_kuka_teleop/relay_pulse.py'
spec = importlib.util.spec_from_file_location('relay_pulse', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SerialFake:
    def __init__(self):
        self.frames, self.closed, self.fail = [], False, False

    def write(self, data):
        if self.fail:
            raise OSError('USB disconnected')
        self.frames.append(data)
        return len(data)

    def close(self):
        self.closed = True


class RelayTests(unittest.TestCase):
    def setUp(self):
        self.ser, self.now = SerialFake(), 0.
        self.io = module.PulseRelay(self.ser, clock=lambda: self.now)

    def test_startup_only_turns_outputs_off(self):
        self.assertEqual(self.ser.frames, [bytes.fromhex('a00200a2'), bytes.fromhex('a00100a1')])
        self.assertIsNone(self.io.target)

    def test_close_pulse_is_half_second_and_nonblocking(self):
        self.io.command(True)
        self.assertEqual(self.ser.frames[-1], bytes.fromhex('a00101a2'))
        self.now = .499
        self.io.step()
        self.assertTrue(self.io.out['close'])
        self.now = .5
        self.io.step()
        self.assertEqual(self.io.out, dict(open=False, close=False))
        self.assertEqual(self.io.result, 'pulse_complete')

    def test_open_and_opposite_command_break_before_make(self):
        self.io.command(True)
        self.io.command(False)
        self.assertEqual(self.ser.frames[-3:], [bytes.fromhex(x) for x in
                         ['a00200a2', 'a00100a1', 'a00201a3']])
        self.assertEqual(self.io.out, dict(open=True, close=False))

    def test_repeated_command_does_not_extend_or_repeat(self):
        self.io.command(True)
        n = len(self.ser.frames)
        self.now = .3
        self.io.command(True)
        self.assertEqual(len(self.ser.frames), n)
        self.assertEqual(self.io.deadline, .5)
        self.now = .6
        self.io.step()
        n = len(self.ser.frames)
        self.io.command(True)
        self.assertEqual(len(self.ser.frames), n)

    def test_shutdown_terminates_pulse_and_closes_port(self):
        self.io.command(False)
        self.io.close()
        self.assertTrue(self.ser.closed)
        self.assertTrue(all(data[2] == 0 for data in self.ser.frames[-2:]))

    def test_fault_latches_and_never_replays_on_recovery(self):
        self.io.command(True)
        self.ser.fail = True
        self.now = 1.
        with self.assertRaises(OSError):
            self.io.step()
        self.assertTrue(self.io.faulted)
        self.assertEqual(self.io.result, 'error')
        self.ser.fail = False
        n = len(self.ser.frames)
        self.io.command(False)
        self.assertEqual(len(self.ser.frames), n)

    def test_invalid_parameters(self):
        for kwargs in [dict(duration=0), dict(duration=float('nan')), dict(duration=3),
                       dict(open_channel=1, close_channel=1), dict(open_channel=4)]:
            with self.assertRaises(ValueError):
                module.PulseRelay(self.ser, **kwargs)


if __name__ == '__main__':
    unittest.main()
