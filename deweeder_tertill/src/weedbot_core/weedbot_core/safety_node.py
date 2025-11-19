import rclpy
from rclpy.node import Node
from weedbot_msgs.msg import SafetyCmd
from geometry_msgs.msg import Twist

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        self.sub = self.create_subscription(
            Twist,
            '/robot_state',
            self.state_callback,
            10
        )

        self.pub = self.create_publisher(SafetyCmd, '/safety_cmd', 10)

    def state_callback(self, state):
        cmd = SafetyCmd()

        belly_contact = state.linear.y > 0.5
        if belly_contact:
            cmd.state = 1

        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
