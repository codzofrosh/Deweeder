#!/usr/bin/env python3
"""
motion_bridge.py
Convert weedbot_msgs/MotionCmd -> geometry_msgs/Twist for the simulator.

Usage:
  python3 scripts/motion_bridge.py
Or run via ros2 run after installing entry point.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
try:
    from weedbot_msgs.msg import MotionCmd
except Exception:
    # fallback import error will be visible when running; keeps the file importable
    MotionCmd = None

class MotionBridge(Node):
    def __init__(self):
        super().__init__('motion_bridge')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        if MotionCmd is None:
            self.get_logger().error('Cannot import weedbot_msgs.msg.MotionCmd - ensure workspace is sourced and package built')
        else:
            self.sub = self.create_subscription(MotionCmd, '/motion_cmd', self.cb_motion, 10)
        self.get_logger().info('motion_bridge started (MotionCmd -> /cmd_vel)')

    def cb_motion(self, msg):
        t = Twist()
        # translate linear_x -> linear.x, angular_z -> angular.z
        try:
            t.linear.x = float(msg.linear_x)
            t.linear.y = 0.0
            t.linear.z = 0.0
            t.angular.x = 0.0
            t.angular.y = 0.0
            t.angular.z = float(msg.angular_z)
        except Exception as e:
            self.get_logger().warn(f'bad MotionCmd payload: {e}')
        self.pub.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = MotionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
