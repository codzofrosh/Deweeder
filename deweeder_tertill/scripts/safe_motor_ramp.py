#!/usr/bin/env python3
import time
import rclpy
from geometry_msgs.msg import Twist
from weedbot_msgs.msg import SafetyCmd

def main():
    rclpy.init()
    node = rclpy.create_node('safe_ramp')
    pub = node.create_publisher(Twist, '/robot_state', 10)
    safety_sub = node.create_subscription(SafetyCmd, '/safety_cmd', lambda m: print("safety", m.state), 10)
    try:
        for s in [0.1*i for i in range(0,6)] + [0.5,0.4,0.2,0.0]:
            t = Twist(); t.linear.x = s
            print("publishing", s)
            pub.publish(t)
            time.sleep(0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
