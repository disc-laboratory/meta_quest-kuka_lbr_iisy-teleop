"""Synthetic PickNik-format Quest input for mock commissioning (never for hardware).

Timeline: 1 s released (arms clutch), then grip held while the hand traces a
small circle, release, then a frozen-pose segment to exercise tracking-loss.
"""
import math
import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool


class FakeQuest(Node):
    def __init__(self):
        super().__init__('fake_quest')
        self.radius = self.declare_parameter('radius', .1).value
        self.odom = self.create_publisher(Odometry, '/right_controller_odom', qos_profile_sensor_data)
        self.grip = self.create_publisher(Bool, '/right_grip_button_state', qos_profile_sensor_data)
        self.trig = self.create_publisher(Bool, '/right_trigger_button_state', qos_profile_sensor_data)
        self.t0 = self.get_clock().now()
        self.period = self.declare_parameter('period', 8.).value
        self.create_timer(1 / 60., self.step)

    def step(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        phase = t % 14.
        held = 1. < phase < 9. or 10. < phase < 13.
        frozen = 10. < phase < 13.
        s = min(phase, 10.) if frozen else phase
        a = 2 * math.pi * max(0., s - 1.) / self.period
        m = Odometry()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), 'quest'
        m.child_frame_id = 'right_controller_odom'
        m.pose.pose.position.x = .4
        m.pose.pose.position.y = self.radius * math.sin(a)
        m.pose.pose.position.z = 1. + self.radius * (1 - math.cos(a))
        if not frozen:  # real tracking jitters; bit-identical poses mean lost tracking
            m.pose.pose.position.x += random.gauss(0., 2e-4)
        m.pose.pose.orientation.w = 1.
        self.odom.publish(m)
        self.grip.publish(Bool(data=held))
        self.trig.publish(Bool(data=4. < phase < 4.2))


def main():
    rclpy.init()
    n = FakeQuest()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
