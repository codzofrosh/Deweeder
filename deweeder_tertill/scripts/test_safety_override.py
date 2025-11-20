#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from weedbot_msgs.msg import MotionCmd, SafetyCmd

TIMEOUT = 5.0

class Tester(Node):
    def __init__(self):
        super().__init__('test_safety_override')
        self.motion_received = None
        self.safety_received = None

        self.create_subscription(MotionCmd, '/motion_cmd', self.motion_cb, 10)
        self.create_subscription(SafetyCmd, '/safety_cmd', self.safety_cb, 10)

        self.pub_state = self.create_publisher(Twist, '/robot_state', 10)
        self.pub_safety = self.create_publisher(SafetyCmd, '/safety_cmd', 10)

        self.get_logger().info("Tester ready")

    def motion_cb(self, msg):
        self.motion_received = msg

    def safety_cb(self, msg):
        self.safety_received = msg

    def publish_state(self, x):
        t = Twist()
        t.linear.x = float(x)
        self.pub_state.publish(t)

    def publish_safety(self, state):
        s = SafetyCmd()
        s.state = int(state)
        # publish a couple times to improve delivery reliability
        for _ in range(3):
            self.pub_safety.publish(s)
            time.sleep(0.02)

    def wait_for_safety(self, expected_state, timeout=TIMEOUT):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.safety_received is not None and int(self.safety_received.state) == expected_state:
                return True
        return False

def main():
    rclpy.init()
    node = Tester()

    # Give system time to start and nodes to connect
    time.sleep(0.5)

    # 1) baseline: publish normal robot_state -> expect nonzero motion_cmd
    node.motion_received = None
    node.safety_received = None
    node.publish_safety(0)  # NORMAL
    # Wait for NORMAL to be seen (best-effort)
    node.wait_for_safety(0, timeout=1.0)
    time.sleep(0.05)
    node.publish_state(0.8)
    t0 = time.time()
    while node.motion_received is None and (time.time() - t0) < TIMEOUT:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.motion_received is None:
        print("FAIL: no motion_cmd received in baseline test")
    else:
        print("baseline motion_cmd:", node.motion_received.linear_x)

    # 2) now publish safety STOP and then robot_state -> expect zeroed motion
    node.motion_received = None
    node.safety_received = None
    node.publish_safety(2)  # STOP
    # wait until STOP is observed by the system
    ok = node.wait_for_safety(2, timeout=2.0)
    if not ok:
        print("WARN: STOP message not observed within timeout; test may be flaky")
    # Small pause to allow subscribers to process (should be unnecessary now)
    time.sleep(0.05)
    node.publish_state(0.8)
    t1 = time.time()
    while node.motion_received is None and (time.time() - t1) < TIMEOUT:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.motion_received is None:
        print("FAIL: no motion_cmd received in safety test")
    else:
        print("safety test motion_cmd:", node.motion_received.linear_x)
        if abs(node.motion_received.linear_x) < 1e-6:
            print("PASS: safety override zeroed motion_cmd")
        else:
            print("FAIL: safety override did NOT zero motion_cmd")

    # cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
