#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from weedbot_msgs.msg import MotionCmd, ToolCmd, SafetyCmd

# Safety states (match safety_node)
SAFETY_NORMAL = 0
SAFETY_SLOW = 1
SAFETY_STOP = 2
SAFETY_EMERGENCY = 3

class BehaviorNode(Node):
    def __init__(self):
        super().__init__('behavior_node')
        self.declare_parameter('speed_scale', 0.2)  # default mapping scale
        self.declare_parameter('tool_enable', True)

        self.safety_state = SAFETY_NORMAL

        # subscribers
        self.create_subscription(Twist, '/robot_state', self.robot_state_cb, 10)
        self.create_subscription(SafetyCmd, '/safety_cmd', self.safety_cb, 10)

        # publishers
        self.motion_pub = self.create_publisher(MotionCmd, '/motion_cmd', 10)
        self.tool_pub = self.create_publisher(ToolCmd, '/tool_cmd', 10)

        self.get_logger().info('BehaviorNode started')

    def safety_cb(self, msg: SafetyCmd):
        prev = self.safety_state
        self.safety_state = int(msg.state)
        if prev != self.safety_state:
            self.get_logger().info(f"Safety changed: {prev} -> {self.safety_state}")

    def robot_state_cb(self, state: Twist):
        """
        Convert robot_state (Twist) to MotionCmd and ToolCmd,
        but honor safety: if safety indicates STOP/EMERGENCY publish zeros and disable tools.
        """
        # prepare commands
        scale = float(self.get_parameter('speed_scale').value)
        motion = MotionCmd()
        tool = ToolCmd()

        # nominal mapping
        motion.linear_x = float(state.linear.x) * scale
        motion.angular_z = float(state.angular.z) * scale

        # nominal tool logic (simple): if forward motion requested, enable front trimmer
        tool.front_trimmer_on = bool(state.linear.x > 0.1) and self.get_parameter('tool_enable').value
        tool.belly_trimmer_mode = 1 if tool.front_trimmer_on else 0
        tool.agitator_pulse = False

        # Safety override (highest priority)
        if self.safety_state >= SAFETY_STOP:
            # zero motion, disable tools
            if motion.linear_x != 0.0 or motion.angular_z != 0.0 or tool.front_trimmer_on:
                self.get_logger().warn(f"Safety override active (state={self.safety_state}) — zeroing commands")
            motion.linear_x = 0.0
            motion.angular_z = 0.0
            tool.front_trimmer_on = False
            tool.belly_trimmer_mode = 0
            tool.agitator_pulse = False

        # If safety is SLOW, scale further (optional)
        if self.safety_state == SAFETY_SLOW:
            motion.linear_x *= 0.5
            motion.angular_z *= 0.5

        # publish
        self.motion_pub.publish(motion)
        self.tool_pub.publish(tool)

        self.get_logger().debug(f"Published MotionCmd linear_x={motion.linear_x:.3f} angular_z={motion.angular_z:.3f} Tool front={tool.front_trimmer_on}")

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
