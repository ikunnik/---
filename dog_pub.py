import time 
import rclpy
import signal
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy


class Pubdog(Node):
    def __init__(self, name):
        super().__init__(name)
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        self.scale = 0.0

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(Twist, "/planner/vel_cmd", qos)

        self.timer = self.create_timer(0.5, self.timer_callback)

    def stop(self):
        self.timer.cancel()
        self.get_logger().info('发布停止指令: x=0, z=0')
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        for _ in range(3):
            msg = Twist()
            msg.linear.x = float(self.vx * self.scale)
            msg.linear.y = float(self.vy * self.scale)
            msg.angular.z = float(self.vyaw)
            self.pub.publish(msg)
            self.get_logger().info('x: "%s" y: "%s" z: "%s"' % (msg.linear.x, msg.linear.y, msg.angular.z))
            time.sleep(0.5)   # 给 DDS 一点时间真正发出去

    def timer_callback(self):
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        msg = Twist()
        msg.linear.x = float(self.vx * self.scale)
        msg.linear.y = float(self.vy * self.scale)
        msg.angular.z = float(self.vyaw)
        self.pub.publish(msg)
        self.get_logger().info('x: "%s" y: "%s" z: "%s"' % (msg.linear.x, msg.linear.y, msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    
    node = Pubdog("topic_dog_pub")
    signal.signal(signal.SIGINT,  lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        rclpy.shutdown()

    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()