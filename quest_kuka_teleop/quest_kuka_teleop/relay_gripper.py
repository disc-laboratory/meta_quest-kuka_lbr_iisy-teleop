"""Quest gripper Bool intent -> 0.5 s USB relay pulse. No finger feedback."""
import json
import time

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String

from .relay_pulse import PulseRelay


class RelayGripper(Node):
    def __init__(self):
        super().__init__('gripper_relay')
        p = lambda name, value: self.declare_parameter(name, value).value
        port = p('relay_port', '/dev/ttyUSB0')
        open_ch, close_ch = p('relay_open_channel', 2), p('relay_close_channel', 1)
        duration = p('relay_pulse_duration', .5)
        # Validate before opening a physical device.
        if open_ch not in (1, 2) or close_ch not in (1, 2) or open_ch == close_ch:
            raise ValueError('Use distinct relay channels 1 and 2')
        if not 0 < duration <= 2:
            raise ValueError('Pulse duration must be > 0 and <= 2 seconds')
        ser = serial.Serial(port, 9600, timeout=.1, write_timeout=.1, exclusive=True)
        try:
            time.sleep(.1)  # Same device-open settling delay as relay.py; startup only.
            self.io = PulseRelay(ser, open_ch, close_ch, duration)
        except Exception:
            ser.close()
            raise
        # VOLATILE deliberately ignores commands retained from before this node started.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Bool, '/teleop/gripper_command', self.on_command, qos)
        self.pub = self.create_publisher(String, '/teleop/gripper_state', 10)
        self.create_timer(.01, self.step)
        self.get_logger().info(
            f'Relay ready on {port}: CH{open_ch}=open, CH{close_ch}=close, '
            f'{duration}s pulses. Waiting for a NEW trigger command; no startup pulse.')

    def on_command(self, msg):
        try:
            self.io.command(bool(msg.data))
        except Exception as exc:
            self.get_logger().error(f'Relay fault; restart node after fixing serial connection: {exc}')

    def step(self):
        try:
            self.io.step()
        except Exception as exc:
            self.get_logger().error(f'Relay pulse termination failed: {exc}')
        self.pub.publish(String(data=json.dumps(dict(
            backend='relay', state='UNKNOWN', feedback_available=False,
            sensor_a=None, sensor_b=None,
            outputs=None if self.io.faulted else self.io.out,
            target=None if self.io.target is None else ('GRIPPED' if self.io.target else 'RELEASED'),
            result=self.io.result))))

    def destroy_node(self):
        try:
            self.io.close()
        except Exception as exc:
            self.get_logger().error(f'Could not confirm relays OFF during shutdown: {exc}')
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = RelayGripper()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
