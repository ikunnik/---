"""Read-only ROS image inventory; never constructs a robot or a controller."""
import argparse
import json
import math
import time


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=5.)
    options = parser.parse_args(args)
    if not math.isfinite(options.seconds) or not 0 < options.seconds <= 60:
        parser.error('--seconds must be in (0, 60]')
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Image, CompressedImage
    context = Context()
    rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('rover_camera_inventory', context=context, use_global_arguments=False)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    types = {'sensor_msgs/msg/Image': Image, 'sensor_msgs/msg/CompressedImage': CompressedImage}
    report, subscriptions = {}, {}
    start = time.monotonic()

    def receive(topic, msg):
        entry = report[topic]
        entry['frames'] += 1
        ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if ns <= entry.get('last_stamp_ns', 0):
            entry['non_increasing_or_zero_stamps'] += 1
        entry['last_stamp_ns'] = ns
        entry['frame_id'] = msg.header.frame_id
        entry['last_payload_bytes'] = len(msg.data)
        entry['source_age_s'] = time.time() - ns / 1e9 if ns > 0 else None
        if isinstance(msg, Image):
            entry.update(encoding=msg.encoding, width=msg.width, height=msg.height,
                         step=msg.step, bigendian=msg.is_bigendian)
        else:
            entry['format'] = msg.format

    try:
        while time.monotonic() - start < options.seconds:
            for topic, names in node.get_topic_names_and_types():
                if topic in subscriptions:
                    continue
                supported = [name for name in names if name in types]
                if len(supported) != 1:
                    continue
                report[topic] = {'type': supported[0], 'frames': 0,
                                 'non_increasing_or_zero_stamps': 0,
                                 'observed_since_s': time.monotonic() - start}
                subscriptions[topic] = node.create_subscription(
                    types[supported[0]], topic, lambda msg, t=topic: receive(t, msg),
                    qos_profile_sensor_data)
            executor.spin_once(timeout_sec=.05)
        for topic, entry in report.items():
            duration = time.monotonic() - start - entry['observed_since_s']
            entry['received_hz'] = entry['frames'] / max(duration, .001)
            entry['publishers'] = [
                {'node': info.node_namespace.rstrip('/') + '/' + info.node_name,
                 'reliability': info.qos_profile.reliability.name,
                 'durability': info.qos_profile.durability.name,
                 'history': info.qos_profile.history.name, 'depth': info.qos_profile.depth}
                for info in node.get_publishers_info_by_topic(topic)]
        print(json.dumps({'duration_s': time.monotonic() - start, 'image_topics': report},
                         ensure_ascii=False, indent=2))
    finally:
        executor.shutdown(timeout_sec=1.)
        node.destroy_node()
        if context.ok():
            context.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
