"""Schunk Co-act gripper over plain 24 V digital I/O, driven from the PC.

Mirrors the iiQKA "SCHUNK Gripper Signals" semantics found on the pendant:
  outputs Open, Close; inputs Sensor A, Sensor B.
  outer grip: GRIPPED = (A False, B True), RELEASED = (A True, B False), else UNKNOWN.
Command: /teleop/gripper_command (Bool, True = close). One output is held until the
target state is sensed or action_timeout expires, then both are cleared. Open and
Close are never TRUE together.
State: /teleop/gripper_state (String JSON) with measured sensors, outputs, state, command.

backend:=mock simulates the gripper (for mock_e2e). Real hardware backends are added
per I/O module once one is chosen; they must replace (never parallel) the IFB-STD lines.
"""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String


class MockBackend:
    def __init__(self, travel_s=.6):
        self.travel_s, self.out = travel_s, dict(open=False, close=False)
        self.pos, self.t = 0., time.monotonic()  # 0 = released, 1 = gripped

    def write(self, open_, close):
        assert not (open_ and close)
        self.out = dict(open=open_, close=close)

    def read(self):
        now = time.monotonic()
        step = (now - self.t) / self.travel_s
        self.t = now
        if self.out['close']:
            self.pos = min(1., self.pos + step)
        elif self.out['open']:
            self.pos = max(0., self.pos - step)
        return dict(a=self.pos <= 0., b=self.pos >= 1.)


BACKENDS = {'mock': MockBackend}


def state_of(a, b):
    return 'GRIPPED' if (not a and b) else 'RELEASED' if (a and not b) else 'UNKNOWN'


class GripperIO(Node):
    def __init__(self):
        super().__init__('gripper_io')
        name = self.declare_parameter('backend', 'mock').value
        self.timeout = self.declare_parameter('action_timeout', 3.).value
        if name not in BACKENDS:
            raise RuntimeError(f'unknown backend {name}; available {list(BACKENDS)}')
        self.io = BACKENDS[name]()
        self.io.write(False, False)
        self.target, self.since, self.result = None, None, 'idle'
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/teleop/gripper_command', self.on_cmd, latched)
        self.pub = self.create_publisher(String, '/teleop/gripper_state', 10)
        self.create_timer(.01, self.step)
        self.get_logger().info(f'gripper backend: {name}')

    def on_cmd(self, msg):
        target = 'GRIPPED' if msg.data else 'RELEASED'
        if target == self.target:
            return
        self.target, self.since, self.result = target, time.monotonic(), 'moving'
        self.io.write(False, False)  # break-before-make
        self.io.write(target == 'RELEASED', target == 'GRIPPED')

    def step(self):
        s = self.io.read()
        state = state_of(s['a'], s['b'])
        if self.result == 'moving':
            if state == self.target:
                self.result = 'reached'
                self.io.write(False, False)
            elif time.monotonic() - self.since > self.timeout:
                self.result = 'timeout'  # e.g. grasped object stops fingers before sensor
                self.io.write(False, False)
                self.get_logger().warn(f'gripper did not reach {self.target}; sensors {s}')
        self.pub.publish(String(data=json.dumps(dict(
            state=state, sensor_a=s['a'], sensor_b=s['b'], outputs=self.io.out,
            target=self.target, result=self.result))))

    def destroy_node(self):
        self.io.write(False, False)
        super().destroy_node()


def main():
    rclpy.init()
    n = GripperIO()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
