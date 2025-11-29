#!/usr/bin/env python3
# scripts/pub_motion_continuous.py
import time
import rclpy
from rclpy.node import Node
from weedbot_msgs.msg import MotionCmd

class PubNode(Node):
    def __init__(self):
        super().__init__('pub_motion_continuous')
        self.pub = self.create_publisher(MotionCmd, '/motion_cmd', 10)
        self.get_logger().info('Publisher ready. Publishing at 1 Hz...')
        self.timer_period = 1.0

    def run(self):
        r = self.create_rate = self.create_timer  # just to avoid linter noise
        try:
            while rclpy.ok():
                msg = MotionCmd()
                msg.linear_x = 0.2
                msg.angular_z = 0.0
                self.pub.publish(msg)
                self.get_logger().info(f'publishing: linear_x={msg.linear_x} angular_z={msg.angular_z}')
                rclpy.spin_once(self, timeout_sec=self.timer_period)
        except KeyboardInterrupt:
            pass

def main():
    rclpy.init()
    node = PubNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
