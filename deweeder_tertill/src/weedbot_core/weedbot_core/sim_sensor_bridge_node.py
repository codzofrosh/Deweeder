#!/usr/bin/env python3
"""
sim_sensor_bridge_node.py
─────────────────────────
Simulation-only node.  Bridges Gazebo contact sensor output and odometry
into the MicroSensorPacket that the rest of the weedbot_core pipeline
expects.  This keeps robot_state_node, behavior_node, and safety_node
completely unchanged between simulation and hardware.

Topic flow
──────────
  /gazebo/front_contact  (ros_gz_interfaces/Contacts)  → cap_front
  /gazebo/belly_contact  (ros_gz_interfaces/Contacts)  → belly_contact
  /odom                  (nav_msgs/Odometry)            → wheel_encoder (derived)
  /imu                   (sensor_msgs/Imu)              → imu_linear, imu_angular

  → /micro/sensor_packet (weedbot_msgs/MicroSensorPacket)   @ 50 Hz
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from weedbot_msgs.msg import MicroSensorPacket

try:
    from ros_gz_interfaces.msg import Contacts
    _HAS_CONTACTS = True
except ImportError:
    _HAS_CONTACTS = False

# Physical constants matching weedbot.urdf
_WHEEL_RADIUS = 0.035        # m
_WHEEL_SEP = 0.14            # m  (left-right center distance)
_TICKS_PER_REV = 2048        # encoder resolution
_TICKS_PER_METER = _TICKS_PER_REV / (2.0 * math.pi * _WHEEL_RADIUS)


class SimSensorBridgeNode(Node):
    def __init__(self):
        super().__init__('sim_sensor_bridge')

        # ── internal state ──────────────────────────────────────────────
        self._cap_front = False
        self._belly_contact = False
        self._imu_linear = [0.0, 0.0, 9.81]   # z = 1g at rest
        self._imu_angular = [0.0, 0.0, 0.0]

        # Accumulated encoder counts (float, cast to int when publishing)
        self._enc = [0.0, 0.0, 0.0, 0.0]      # FL, FR, RL, RR

        self._last_stamp = None   # used to compute dt between odom callbacks

        # ── subscribers ─────────────────────────────────────────────────
        if _HAS_CONTACTS:
            self.create_subscription(
                Contacts, '/gazebo/front_contact', self._front_cb, 10)
            self.create_subscription(
                Contacts, '/gazebo/belly_contact', self._belly_cb, 10)
        else:
            self.get_logger().warn(
                'ros_gz_interfaces not found — contact topics disabled. '
                'Install ros-jazzy-ros-gz-interfaces and rebuild.')

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Imu, '/imu', self._imu_cb, 10)

        # ── publisher ───────────────────────────────────────────────────
        self._pub = self.create_publisher(
            MicroSensorPacket, '/micro/sensor_packet', 10)

        # Publish at 50 Hz (matches sensor update_rate in URDF)
        self.create_timer(0.02, self._publish)

        self.get_logger().info('SimSensorBridgeNode started')

    # ────────────────────────────────────────────────────────────────────
    # Contact callbacks
    # ────────────────────────────────────────────────────────────────────

    def _front_cb(self, msg):
        # Any non-empty contacts list means something is touching the bumper
        self._cap_front = len(msg.contacts) > 0

    def _belly_cb(self, msg):
        self._belly_contact = len(msg.contacts) > 0

    # ────────────────────────────────────────────────────────────────────
    # Odometry → wheel encoder simulation
    # ────────────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        now_sec = (msg.header.stamp.sec
                   + msg.header.stamp.nanosec * 1e-9)

        if self._last_stamp is None:
            self._last_stamp = now_sec
            return

        dt = now_sec - self._last_stamp
        self._last_stamp = now_sec

        if dt <= 0.0 or dt > 1.0:
            # Skip abnormal time steps
            return

        # Chassis velocities from the diff-drive plugin
        v = msg.twist.twist.linear.x    # forward velocity (m/s)
        w = msg.twist.twist.angular.z   # yaw rate (rad/s)

        # Per-side velocities
        v_left = v - w * _WHEEL_SEP / 2.0
        v_right = v + w * _WHEEL_SEP / 2.0

        # Integrate to encoder tick delta
        d_left = v_left * dt * _TICKS_PER_METER
        d_right = v_right * dt * _TICKS_PER_METER

        # Four wheels: FL=0, FR=1, RL=2, RR=3
        self._enc[0] += d_left    # FL
        self._enc[1] += d_right   # FR
        self._enc[2] += d_left    # RL
        self._enc[3] += d_right   # RR

    # ────────────────────────────────────────────────────────────────────
    # IMU callback
    # ────────────────────────────────────────────────────────────────────

    def _imu_cb(self, msg: Imu):
        self._imu_linear = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ]
        self._imu_angular = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ]

    # ────────────────────────────────────────────────────────────────────
    # Publish MicroSensorPacket
    # ────────────────────────────────────────────────────────────────────

    def _publish(self):
        pkt = MicroSensorPacket()

        pkt.wheel_encoder = [int(e) for e in self._enc]

        pkt.cap_front = self._cap_front
        pkt.cap_left = False
        pkt.cap_right = False
        pkt.cap_rear = False
        pkt.belly_contact = self._belly_contact

        pkt.imu_linear = [float(v) for v in self._imu_linear]
        pkt.imu_angular = [float(v) for v in self._imu_angular]

        pkt.motor_currents = [0.0] * 6  # not simulated
        pkt.tick_time = 0.02            # matches timer period

        self._pub.publish(pkt)


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
