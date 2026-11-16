#!/usr/bin/env python3
"""ROS 2 只读体检：确认速度话题有订阅者、里程计/雷达有数据。

用法（roverx1-dev 容器）：
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python3 tools/ros2_check.py --seconds 5

不会发布非零速度，不会让机器狗移动。
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ros2_robot_link import _ensure_ros2_dds  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="ROS 2 只读链路体检")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--vel-topic", default="/planner/vel_cmd")
    parser.add_argument("--odom-topic", default="/foot_odom/data")
    parser.add_argument("--scan-topic", default="/front_scan")
    args = parser.parse_args()

    _ensure_ros2_dds()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = Node("rover_x1_ros2_check")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    pub = node.create_publisher(Twist, args.vel_topic, qos)
    sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    seen = {"odom": 0, "scan": 0}

    def on_odom(_msg):
        seen["odom"] += 1

    def on_scan(_msg):
        seen["scan"] += 1

    node.create_subscription(Odometry, args.odom_topic, on_odom, sensor_qos)
    node.create_subscription(LaserScan, args.scan_topic, on_scan, sensor_qos)

    deadline = time.monotonic() + max(1.0, args.seconds)
    subscribers = 0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        subscribers = node.count_subscribers(args.vel_topic)

    print(f"速度话题 {args.vel_topic}: 订阅者 {subscribers}")
    print(f"里程计 {args.odom_topic}: {seen['odom']} 帧")
    print(f"雷达   {args.scan_topic}: {seen['scan']} 帧")
    pub.publish(Twist())  # 只发一帧零速，确认发布链路
    node.destroy_node()
    rclpy.shutdown()

    ok = subscribers > 0 and seen["odom"] > 0
    print("结论：", "正常" if ok else "异常（检查 DDS 网卡/ROS_DOMAIN_ID）")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
