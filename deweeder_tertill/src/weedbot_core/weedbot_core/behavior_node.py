import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from weedbot_msgs.msg import MotionCmd, ToolCmd

class BehaviorNode(Node):
    def __init__(self):
        super().__init__('behavior_node')

        self.sub = self.create_subscription(
            Twist,
            '/robot_state',
            self.state_callback,
            10
        )

        self.motion_pub = self.create_publisher(MotionCmd, '/motion_cmd', 10)
        self.tool_pub = self.create_publisher(ToolCmd, '/tool_cmd', 10)

    def state_callback(self, state):
        motion = MotionCmd()
        tool = ToolCmd()

        front_contact = state.linear.x > 0.5
        belly_contact = state.linear.y > 0.5
        side_contact = state.angular.z > 0.5

        if side_contact:
            motion.linear_x = 0.0
            motion.angular_z = 0.6

        elif front_contact:
            tool.front_trimmer_on = True
            motion.linear_x = 0.05

        elif belly_contact:
            tool.belly_trimmer_mode = 2
            motion.linear_x = 0.03

        else:
            motion.linear_x = 0.08
            tool.belly_trimmer_mode = 1

        self.motion_pub.publish(motion)
        self.tool_pub.publish(tool)

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
