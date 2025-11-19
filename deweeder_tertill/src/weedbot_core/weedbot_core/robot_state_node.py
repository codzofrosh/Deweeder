import rclpy
def to_float(x):
    try:
        return float(x)
    except Exception:
        return 0.0

def to_int(x):
    try:
        return int(x)
    except Exception:
        return 0

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from weedbot_msgs.msg import MicroSensorPacket

class RobotStateNode(Node):
    def __init__(self):
        super().__init__('robot_state_node')

        self.sub = self.create_subscription(
            MicroSensorPacket,
            '/micro/sensor_packet',
            self.sensor_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)

        self.state_pub = self.create_publisher(
            Twist,
            '/robot_state',
            10
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def sensor_callback(self, msg):
        imu = Imu()
        imu.linear_acceleration.x = to_float(msg.imu_linear[0])
        imu.linear_acceleration.y = to_float(msg.imu_linear[1])
        imu.linear_acceleration.z = to_float(msg.imu_linear[2])

        imu.angular_velocity.x = to_float(msg.imu_angular[0])
        imu.angular_velocity.y = to_float(msg.imu_angular[1])
        imu.angular_velocity.z = to_float(msg.imu_angular[2])
        self.imu_pub.publish(imu)

        odom = Odometry()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        self.odom_pub.publish(odom)

        state = Twist()
        state.linear.x = to_float(msg.cap_front)
        state.linear.y = to_float(msg.belly_contact)
        state.angular.z = to_float(msg.cap_left or msg.cap_right)
        self.state_pub.publish(state)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
