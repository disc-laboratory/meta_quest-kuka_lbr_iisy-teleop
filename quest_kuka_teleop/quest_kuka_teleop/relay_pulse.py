"""Nonblocking CH340 relay pulses using the protocol in kuka_ws/relay.py."""
import math
import time


def frame(channel, on):
    state = int(bool(on))
    return bytes([0xA0, channel, state, (0xA0 + channel + state) & 0xFF])


class PulseRelay:
    def __init__(self, serial_port, open_channel=2, close_channel=1,
                 duration=.5, clock=time.monotonic):
        if (open_channel not in (1, 2) or close_channel not in (1, 2)
                or open_channel == close_channel):
            raise ValueError('Open/close must use distinct channels 1 and 2')
        if not math.isfinite(duration) or not 0 < duration <= 2:
            raise ValueError('Pulse duration must be > 0 and <= 2 seconds')
        self.ser = serial_port
        self.open_channel, self.close_channel = open_channel, close_channel
        self.duration, self.clock = duration, clock
        self.deadline, self.target = None, None
        self.out = dict(open=False, close=False)
        self.result = 'idle'
        self.faulted = False
        self.all_off()

    def write(self, channel, on):
        data = frame(channel, on)
        if self.ser.write(data) != len(data):
            raise OSError('Incomplete relay serial write')

    def all_off(self):
        # Try both even if one fails. No automatic retry of an ON command.
        error = None
        for channel in (self.open_channel, self.close_channel):
            try:
                self.write(channel, False)
            except Exception as exc:
                error = exc
        self.out = dict(open=False, close=False)
        self.deadline = None
        if error:
            raise error

    def fail(self):
        self.faulted, self.result = True, 'error'
        try:
            self.all_off()
        except Exception:
            pass  # Output state is unknown when serial communication fails.

    def command(self, closed):
        if self.faulted:
            return
        if self.target is not None and self.target == bool(closed):
            return  # Repeated Bool samples never extend/retrigger a pulse.
        try:
            self.all_off()  # Break before make, including an interrupted pulse.
            self.write(self.close_channel if closed else self.open_channel, True)
            self.out = dict(open=not bool(closed), close=bool(closed))
            self.target = bool(closed)
            self.deadline = self.clock() + self.duration
            self.result = 'pulsing'
        except Exception:
            self.fail()
            raise

    def step(self):
        if self.deadline is not None and self.clock() >= self.deadline:
            try:
                self.all_off()
                self.result = 'pulse_complete'
            except Exception:
                self.fail()
                raise

    def close(self):
        try:
            self.all_off()
        finally:
            self.ser.close()
