#!/usr/bin/env python3
"""ROS adapter for the fail-closed mock HAL; physical backends remain inhibited."""
import math
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from weedbot_msgs.msg import MotionCmd, ToolCmd, SafetyCmd
from weedbot_core.drivers.mock_driver import MockDriver
from weedbot_core.hal.safety_gate import SafetyGate


class HalHW(Node):
    def __init__(self):
        super().__init__('hal_hw_gpio')
        self.declare_parameter('hw_mode', False)
        self.declare_parameter('hw_backend', 'mock')
        self.declare_parameter('heartbeat_timeout_ms', 500.0)
        mode = self.get_parameter('hw_mode').value
        backend = self.get_parameter('hw_backend').value
        if str(mode).lower() not in ('false', '0') or backend != 'mock':
            raise RuntimeError('Physical HAL inhibited: tool protocol and firmware watchdog unverified')
        self.driver = MockDriver()
        if not self.driver.init():
            raise RuntimeError('Mock driver initialization failed')
        self.gate = SafetyGate(self.driver, timeout=float(
            self.get_parameter('heartbeat_timeout_ms').value) / 1000.0)
        self.hal_ready_pub = self.create_publisher(Bool, '/hal/ready', 1)
        self.diag_pub = self.create_publisher(String, '/hal/diag', 1)
        self.create_subscription(MotionCmd, '/motion_cmd', self.cb_motion, 1)
        self.create_subscription(ToolCmd, '/tool_cmd', self.cb_tool, 1)
        self.create_subscription(SafetyCmd, '/safety_cmd', self.cb_safety, 10)
        self.create_subscription(Float32, '/heartbeat', self.cb_heartbeat, 1)
        self.create_subscription(Bool, '/emergency_stop', self.cb_estop, 10)
        self.create_subscription(Odometry, '/odom', self.cb_feedback, 1)
        self.create_service(Trigger, '/hal/arm', self.arm)
        self.create_service(Trigger, '/hal/disarm', self.disarm)
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.02, self.watchdog, clock=self._steady_clock)
        self.create_timer(0.2, self._publish_ready, clock=self._steady_clock)

    def cb_motion(self, msg):
        self.gate.command_motion(float(msg.linear_x), float(msg.angular_z))

    def cb_tool(self, msg):
        self.gate.command_tool(msg.front_trimmer_on, int(msg.belly_trimmer_mode), msg.agitator_pulse)

    def cb_safety(self, msg):
        self.gate.update_safety(int(msg.state))

    def cb_heartbeat(self, msg):
        if math.isfinite(msg.data):
            self.gate.heartbeat()
        else:
            self.gate.trip('invalid heartbeat')

    def cb_estop(self, msg):
        self.gate.emergency_stop(msg.data)

    def cb_feedback(self, msg):
        self.gate.feedback(float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z))

    def arm(self, request, response):
        response.success, response.message = self.gate.arm()
        return response

    def disarm(self, request, response):
        self.gate.trip('operator disarm')
        response.success = not self.gate.stop_errors
        response.message = self.gate.reason + '; '.join(self.gate.stop_errors)
        return response

    def watchdog(self):
        self.gate.tick()

    def _publish_ready(self):
        self.gate.tick()
        self.hal_ready_pub.publish(Bool(data=self.gate.armed))
        self.diag_pub.publish(String(data=f'armed={self.gate.armed} reason={self.gate.reason} stop_errors={self.gate.stop_errors}'))

    def destroy_node(self):
        try:
            self.gate.shutdown()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HalHW()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
