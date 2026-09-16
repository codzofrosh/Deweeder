#!/usr/bin/env python3
"""Fail-closed sensor monitoring with a steady-clock watchdog and reset service.

Twist mapping: linear.y=belly, angular.z=side, angular.y=rear. Front contact
remains a behavior input. IMU acceleration must include gravity with +Z upright.
"""
import math

import rclpy
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from weedbot_msgs.msg import SafetyCmd

from weedbot_core.hal.safety_monitor import DEFAULT_LIMITS, SafetyMonitor


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        limits = {}
        for name, default in DEFAULT_LIMITS.items():
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
            limits[name] = self.get_parameter(name).value
        self._monitor = SafetyMonitor(**limits)
        self._last_report = None
        self._pub = self.create_publisher(SafetyCmd, '/safety_cmd', 10)
        self.create_subscription(Twist, '/robot_state', self._state_cb, 10)
        self.create_subscription(Imu, '/imu', self._imu_cb, qos_profile_sensor_data)
        self._reset_service = self.create_service(Trigger, '/safety/reset', self._reset_cb)
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(0.05, self._publish, clock=self._steady_clock)
        self._publish()

    def _state_cb(self, state):
        signals = (state.linear.y, state.angular.z, state.angular.y)
        valid = all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in signals)
        self._monitor.update_contacts(belly=state.linear.y > 0.5,
                                      side=state.angular.z > 0.5,
                                      rear=state.angular.y > 0.5, valid=valid)
        self._publish()

    def _imu_cb(self, msg):
        # A negative covariance[0] marks an unavailable measurement. Orientation
        # is not needed because the monitor estimates static tilt from gravity.
        valid = (math.isfinite(msg.linear_acceleration_covariance[0]) and
                 math.isfinite(msg.angular_velocity_covariance[0]) and
                 msg.linear_acceleration_covariance[0] >= 0 and
                 msg.angular_velocity_covariance[0] >= 0)
        acceleration, rate = msg.linear_acceleration, msg.angular_velocity
        self._monitor.update_imu(acceleration.x, acceleration.y, acceleration.z,
                                 rate.x, rate.y, rate.z, valid=valid)
        self._publish()

    def _reset_cb(self, request, response):
        response.success, response.message = self._monitor.reset()
        self._publish()
        return response

    def _publish(self):
        status = self._monitor.evaluate()
        cmd = SafetyCmd()
        cmd.state = status.state
        self._pub.publish(cmd)
        report = (status.state, status.reason)
        if report != self._last_report:
            self.get_logger().info(f'Safety state={status.state}: {status.reason}')
            self._last_report = report


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
