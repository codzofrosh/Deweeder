#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from weedbot_msgs.msg import MotionCmd, ToolCmd, SafetyCmd

SAFETY_NORMAL = 0
SAFETY_SLOW = 1
SAFETY_STOP = 2
SAFETY_EMERGENCY = 3

class HalNode(Node):
    def __init__(self):
        super().__init__('hal_node')
        self.safety_state = SAFETY_NORMAL
        self.latest_motion = MotionCmd()
        self.latest_tool = ToolCmd()

        self.create_subscription(MotionCmd, '/motion_cmd', self.motion_cb, 10)
        self.create_subscription(ToolCmd, '/tool_cmd', self.tool_cb, 10)
        self.create_subscription(SafetyCmd, '/safety_cmd', self.safety_cb, 10)

        # publish a simple diagnostics topic if you want later; for now log.
        self.get_logger().info("HAL node started (simulated)")

    def safety_cb(self, msg: SafetyCmd):
        prev = self.safety_state
        self.safety_state = int(msg.state)
        if prev != self.safety_state:
            self.get_logger().info(f"HAL observed safety change: {prev} -> {self.safety_state}")

    def motion_cb(self, msg: MotionCmd):
        # apply last-resort safety check
        m = MotionCmd()
        m.linear_x = msg.linear_x
        m.angular_z = msg.angular_z
        if self.safety_state >= SAFETY_STOP:
            # enforce zero
            m.linear_x = 0.0
            m.angular_z = 0.0
        # simulate PWM or motor setpoint scaling
        pwm_left = max(min(m.linear_x, 1.0), -1.0)
        pwm_right = pwm_left  # simple differential omitted
        self.get_logger().info(f"HAL apply motion -> pwm L={pwm_left:.3f} R={pwm_right:.3f}")

    def tool_cb(self, msg: ToolCmd):
        t_on = bool(msg.front_trimmer_on)
        if self.safety_state >= SAFETY_STOP:
            t_on = False
        mode = int(msg.belly_trimmer_mode)
        self.get_logger().info(f"HAL apply tool -> front_trimmer_on={t_on} mode={mode}")

def main(args=None):
    rclpy.init(args=args)
    node = HalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
