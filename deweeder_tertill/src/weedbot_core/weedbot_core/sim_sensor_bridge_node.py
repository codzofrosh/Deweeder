#!/usr/bin/env python3
"""Fresh raw Gazebo inputs -> sensor packets, with no reconstruction feedback.

Required sources: /sim/raw/odom, /sim/raw/imu, /gazebo/front_contact and
/gazebo/belly_contact. Missing/stale sources suppress packets; downstream safety
then expires. Side/rear contacts and motor currents remain unmodeled, not verified
healthy hardware signals. tick_time is the source-odom interval between emitted
snapshots; first/recovery snapshots use zero to invalidate velocity.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from weedbot_msgs.msg import MicroSensorPacket
from weedbot_core.sim_sensor_cache import SimSensorCache

try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:
    Contacts = None


class SimSensorBridgeNode(Node):
    def __init__(self):
        super().__init__('sim_sensor_bridge')
        self.declare_parameter('source_timeout', 0.5, ParameterDescriptor(read_only=True))
        self._cache = SimSensorCache(source_timeout=self.get_parameter('source_timeout').value)
        self._pub = self.create_publisher(MicroSensorPacket, '/micro/sensor_packet', 10)
        self.create_subscription(Odometry, '/sim/raw/odom', self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, '/sim/raw/imu', self._imu_cb, qos_profile_sensor_data)
        if Contacts is not None:
            self.create_subscription(Contacts, '/gazebo/front_contact', self._front_cb,
                                     qos_profile_sensor_data)
            self.create_subscription(Contacts, '/gazebo/belly_contact', self._belly_cb,
                                     qos_profile_sensor_data)
        else:
            self.get_logger().error('Contact message support unavailable; sensor packets inhibited')
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(0.02, self._publish, clock=self._steady_clock)

    def _front_cb(self, msg):
        self._cache.update_contact('front_contact', bool(msg.contacts))

    def _belly_cb(self, msg):
        self._cache.update_contact('belly_contact', bool(msg.contacts))

    def _odom_cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._cache.update_odom(stamp, msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _imu_cb(self, msg):
        valid = (math.isfinite(msg.linear_acceleration_covariance[0]) and
                 math.isfinite(msg.angular_velocity_covariance[0]) and
                 msg.linear_acceleration_covariance[0] >= 0 and
                 msg.angular_velocity_covariance[0] >= 0)
        a, w = msg.linear_acceleration, msg.angular_velocity
        self._cache.update_imu([a.x, a.y, a.z], [w.x, w.y, w.z], valid=valid)

    def _publish(self):
        values = self._cache.packet()
        if values is None:
            return
        packet = MicroSensorPacket()
        packet.wheel_encoder = values['wheel_encoder']
        packet.imu_linear = [float(v) for v in values['imu_linear']]
        packet.imu_angular = [float(v) for v in values['imu_angular']]
        packet.cap_front = values['cap_front']
        packet.belly_contact = values['belly_contact']
        packet.cap_left = packet.cap_right = packet.cap_rear = False
        packet.motor_currents = [0.0] * 6
        packet.tick_time = float(values['tick_time'])
        self._pub.publish(packet)


def main(args=None):
    rclpy.init(args=args)
    node = SimSensorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
