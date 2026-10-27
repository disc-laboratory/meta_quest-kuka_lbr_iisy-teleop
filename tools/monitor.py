"""Record PickNik Quest ROS input to JSONL. No robot command publishers.

This is an input commissioning log, NOT an imitation-learning dataset.
"""
import argparse
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=60.)
    args = parser.parse_args()
    if not 0 < args.seconds <= 3600:
        parser.error('--seconds must be in (0, 3600]')
    if args.out.exists():
        parser.error('Output exists; choose a new filename')
    # Run with the system ROS Python, not a virtualenv.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool

    args.out.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('quest_input_monitor')
    start = time.monotonic()
    counts = {}
    subscriptions = []
    try:
        with args.out.open('x', buffering=1) as stream:
            def save(topic, value):
                counts[topic] = counts.get(topic, 0) + 1
                stream.write(json.dumps(dict(topic=topic, receive_monotonic=time.monotonic(),
                                            receive_ros_ns=node.get_clock().now().nanoseconds,
                                            value=value), allow_nan=False) + '\n')

            def pose_callback(msg, topic):
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                save(topic, dict(frame_id=msg.header.frame_id,
                                 source_stamp_ns=msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                                 position=[p.x, p.y, p.z], quaternion_xyzw=[q.x, q.y, q.z, q.w]))

            for hand in ('right', 'left'):
                topic = f'/{hand}_controller_odom'
                subscriptions.append(node.create_subscription(
                    Odometry, topic, lambda m, t=topic: pose_callback(m, t), qos_profile_sensor_data))
                for button in ('grip', 'trigger', 'a', 'b'):
                    topic = f'/{hand}_{button}_button_state'
                    subscriptions.append(node.create_subscription(
                        Bool, topic, lambda m, t=topic: save(t, bool(m.data)), qos_profile_sensor_data))
            print('Listening only. Move controllers and press/release grip and trigger.')
            print('No robot output is published. Ctrl+C finishes the input log.')
            while rclpy.ok() and time.monotonic() - start < args.seconds:
                rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.monotonic() - start
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(dict(elapsed_s=elapsed, messages=counts,
                          average_receive_hz={k: v / elapsed for k, v in counts.items()}), indent=2))
    if not counts:
        print('No input received. Check Quest app, ROS TCP endpoint, ROS domain and network.')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
