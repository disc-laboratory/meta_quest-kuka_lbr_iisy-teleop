"""Quest controller pose -> clutch -> bounded PoseStamped for MoveIt Servo.

Motion only while the clutch (grip) is held, input is fresh, tracking looks
live, joint feedback is fresh and Servo is not halted. When any gate fails the
node STOPS PUBLISHING; Servo then halts after incoming_command_timeout. That
is a software stop only: the pendant enabling switch / E-stop remain the
safety layer. Not a certified safety function.
"""
import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
import tf2_ros

from .clutch import Clutch

HALT_CODES = {ServoStatus.HALT_FOR_SINGULARITY, ServoStatus.HALT_FOR_COLLISION, ServoStatus.INVALID}


class Bridge(Node):
    def __init__(self):
        super().__init__('quest_bridge')
        p = lambda n, v: self.declare_parameter(n, v).value
        self.frame, self.tcp_frame = p('planning_frame', 'base_link'), p('tcp_frame', 'gripper_tcp')
        hand = p('hand', 'right')
        m = np.array(p('base_from_quest', [1., 0., 0., 0., 1., 0., 0., 0., 1.]), float).reshape(3, 3)
        self.max_input_age, self.max_js_age = p('max_input_age', .15), p('max_joint_state_age', .1)
        # Wi-Fi delivers in bursts: a gap up to max_input_hold pauses motion but keeps the
        # clutch anchor; longer gaps require a fresh grip release.
        self.max_input_hold = p('max_input_hold', .5)
        self.stale_n = p('stale_pose_count', 15)
        # max_lead*: safety tether between command and MEASURED tcp (runaway guard, not speed).
        # max_target_*speed: rate limit on the published target itself (sets teleop speed).
        self.max_lead, self.max_lead_angle = p('max_lead', .03), p('max_lead_angle', .25)
        self.max_speed = p('max_target_speed', .05)
        self.max_ang_speed = p('max_target_angular_speed', .3)
        self.cmd = None  # last published (pos, Rotation); None while disengaged
        self.ws_min = np.array(p('workspace_min', [.25, -.35, .1]), float)
        self.ws_max = np.array(p('workspace_max', [.65, .35, .6]), float)
        self.demo_bounds = p('demo_bounds', False)
        self.enforce_workspace = p('enforce_workspace', False) or self.demo_bounds
        self.require_servo_status = p('require_servo_status', False) or self.demo_bounds
        if self.enforce_workspace and (not np.isfinite(np.r_[self.ws_min, self.ws_max]).all()
                                       or np.any(self.ws_min >= self.ws_max)):
            raise ValueError('Invalid workspace bounds')
        self.demo_rotation = Rotation.from_euler('xyz', p('demo_rpy', [0., 0., 0.]))
        self.demo_angle = p('demo_angle', .1745329252)
        if self.demo_bounds and (not np.isfinite(np.r_[self.ws_min, self.ws_max, self.demo_angle]).all()
                                 or np.any(self.ws_min >= self.ws_max) or not 0 < self.demo_angle <= .2):
            raise ValueError('Invalid demo bounds')
        self.clutch = Clutch(m, scale=p('scale', .3), max_age=self.max_input_hold,
                             max_translation_jump=p('max_translation_jump', .15),
                             max_rotation_jump=p('max_rotation_jump', .8),
                             translation_only=p('translation_only', False))
        self.pose = self.pose_t = self.grip = self.grip_t = None
        self.trigger = None  # Require a received trigger release before accepting a press.
        self.same_count, self.last_xyzq = 0, None
        self.js_t = None
        self.servo_code, self.servo_msg = None, ''
        self.servo_t = None
        self.gripper_closed = False
        self.engaged, self.reason = False, 'startup'
        self.servo_ready = False

        self.create_subscription(Odometry, f'/{hand}_controller_odom', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(Bool, f'/{hand}_grip_button_state', self.on_grip, qos_profile_sensor_data)
        self.create_subscription(Bool, f'/{hand}_trigger_button_state', self.on_trigger, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_js, qos_profile_sensor_data)
        self.create_subscription(ServoStatus, '/servo_node/status', self.on_status, 10)
        self.cmd_pub = self.create_publisher(PoseStamped, '/servo_node/pose_target_cmds', 10)
        self.intent_pub = self.create_publisher(PoseStamped, '/teleop/hand_intent', 10)
        self.state_pub = self.create_publisher(String, '/teleop/teleop_state', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grip_pub = self.create_publisher(Bool, '/teleop/gripper_command', latched)
        # Publish only deliberate trigger edges: never open a real gripper on startup.
        self.tf = tf2_ros.Buffer()
        # TF on its own thread: high-rate /tf must not delay pose/joint_state callbacks.
        self.tf_listener = tf2_ros.TransformListener(self.tf, self, spin_thread=True)
        self.dt = 1. / p('rate_hz', 100.)
        self.switch = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        self.create_timer(self.dt, self.step)
        self.create_timer(1., self.ensure_pose_mode)

    def now(self):
        return time.monotonic()

    def ensure_pose_mode(self):
        if self.servo_ready or not self.switch.service_is_ready():
            return
        fut = self.switch.call_async(ServoCommandType.Request(command_type=ServoCommandType.Request.POSE))
        fut.add_done_callback(self._switched)

    def _switched(self, fut):
        ok = fut.result() is not None and fut.result().success
        self.servo_ready = bool(ok)
        self.get_logger().info(f'Servo POSE mode: {ok}')

    def on_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        xyzq = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        # Quest/Unity keeps re-sending the last transform when tracking is lost.
        self.same_count = self.same_count + 1 if xyzq == self.last_xyzq else 0
        self.last_xyzq, self.pose, self.pose_t = xyzq, xyzq, self.now()

    def on_grip(self, msg):
        self.grip, self.grip_t = bool(msg.data), self.now()

    def on_trigger(self, msg):
        pressed = bool(msg.data)
        if pressed and self.trigger is False:  # released -> pressed toggles intent
            self.gripper_closed = not self.gripper_closed
            self.grip_pub.publish(Bool(data=self.gripper_closed))
        self.trigger = pressed

    def on_js(self, _):
        self.js_t = self.now()

    def on_status(self, msg):
        self.servo_code, self.servo_msg = msg.code, msg.message
        self.servo_t = self.now()

    def tcp(self):
        t = self.tf.lookup_transform(self.frame, self.tcp_frame, rclpy.time.Time())
        tr, r = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), np.array([r.x, r.y, r.z, r.w])

    def gate(self, now):
        if not self.servo_ready:
            return 'servo_not_in_pose_mode'
        if self.pose_t is None or self.grip_t is None:
            return 'no_quest_input'
        if self.js_t is None or now - self.js_t > self.max_js_age:
            return 'joint_states_stale'
        if self.require_servo_status:
            if self.servo_t is None or now - self.servo_t > .2:
                return 'servo_status_stale'
            if self.servo_code in HALT_CODES:
                return f'servo_halt:{self.servo_code}'
        if self.servo_code in HALT_CODES:
            if self.grip is False:  # operator released: acknowledge; clutch still needs a fresh grip
                self.servo_code = None
                return 'servo_halt_acknowledged'
            return f'servo_halt:{self.servo_code}'
        return None

    def step(self):
        now = self.now()
        reason = self.gate(now)
        target = None
        if reason is None:
            try:
                tp, tq = self.tcp()
                if self.enforce_workspace and (np.any(tp < self.ws_min) or np.any(tp > self.ws_max)):
                    reason = 'outside_workspace'
                    self.clutch.reset()
                elif self.demo_bounds and (Rotation.from_quat(tq) * self.demo_rotation.inv()).magnitude() > self.demo_angle + .02:
                    reason = 'outside_demo_orientation'
                    self.clutch.reset()
            except tf2_ros.TransformException:
                reason = 'tcp_tf_unavailable'
        if reason is None:
            age = now - min(self.pose_t, self.grip_t)
            tracking = self.same_count < self.stale_n
            if self.max_input_age < age <= self.max_input_hold and tracking:
                reason = 'input_gap_pause'  # publish nothing: Servo halts, anchor kept
            target = None if reason else self.clutch.update(self.pose[:3], self.pose[3:], tp, tq,
                                        held=self.grip, tracking_valid=tracking, age=age)
            if target is None and not reason:
                reason = ('input_stale' if age > self.max_input_age else
                          'tracking_lost' if not tracking else
                          'clutch_released' if not self.grip else 'awaiting_release_or_jump')
        else:
            self.clutch.reset()  # any hard fault requires a fresh grip release
        self.engaged = target is not None
        if not self.engaged:
            self.cmd = None
        if self.engaged:
            pos, quat = self.bound(np.asarray(target[0]), np.asarray(target[1]), tp, tq)
            self.intent_pub.publish(self.msg(*target))
            self.cmd_pub.publish(self.msg(pos, quat))
            reason = 'engaged'
        if reason != self.reason:
            self.get_logger().info(f'state: {reason}')
        self.reason = reason
        self.state_pub.publish(String(data=json.dumps(dict(
            engaged=self.engaged, reason=reason, gripper_closed=self.gripper_closed,
            servo_code=self.servo_code, tracking_same_count=self.same_count,
            input_age=None if self.pose_t is None else now - self.pose_t,
            joint_state_age=None if self.js_t is None else now - self.js_t))))

    def bound(self, pos, quat, tp, tq):
        pos = np.clip(pos, self.ws_min, self.ws_max)
        # Rate-limit the target stream: smooth under Wi-Fi bursts, independent of feedback lag.
        if self.cmd is None:
            self.cmd = (tp.copy(), Rotation.from_quat(tq))
        cp, cr = self.cmd
        step = pos - cp
        n = np.linalg.norm(step)
        lim = self.max_speed * self.dt
        pos = cp + (step * (lim / n) if n > lim else step)
        rot = (Rotation.from_quat(quat) * cr.inv()).as_rotvec()
        a = np.linalg.norm(rot)
        alim = self.max_ang_speed * self.dt
        quat = (Rotation.from_rotvec(rot * (alim / a) if a > alim else rot) * cr).as_quat()
        d = pos - tp
        n = np.linalg.norm(d)
        if n > self.max_lead:
            pos = tp + d * (self.max_lead / n)
        rt, rc = Rotation.from_quat(tq), Rotation.from_quat(quat)
        if self.demo_bounds:
            offset = (rc * self.demo_rotation.inv()).as_rotvec()
            angle = np.linalg.norm(offset)
            if angle > self.demo_angle:
                rc = Rotation.from_rotvec(offset * self.demo_angle / angle) * self.demo_rotation
        delta = (rc * rt.inv()).as_rotvec()
        a = np.linalg.norm(delta)
        if a > self.max_lead_angle:
            rc = Rotation.from_rotvec(delta * (self.max_lead_angle / a)) * rt
        self.cmd = (pos, rc)
        return pos, rc.as_quat()

    def msg(self, pos, quat):
        m = PoseStamped()
        m.header.frame_id, m.header.stamp = self.frame, self.get_clock().now().to_msg()
        (m.pose.position.x, m.pose.position.y, m.pose.position.z) = map(float, pos)
        (m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w) = map(float, quat)
        return m


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
