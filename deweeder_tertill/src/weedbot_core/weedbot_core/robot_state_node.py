#!/usr/bin/env python3
"""
robot_state_node.py
───────────────────
Converts raw MicroSensorPacket data into standard ROS messages:

  /micro/sensor_packet  →  /imu        (sensor_msgs/Imu)
  /micro/sensor_packet  →  /odom       (nav_msgs/Odometry)
  /micro/sensor_packet  →  /robot_state (geometry_msgs/Twist)

Odometry is computed via differential-drive dead reckoning from
wheel encoder tick deltas.  Physical constants must match the actual
robot (or the URDF values used in simulation).
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from weedbot_msgs.msg import MicroSensorPacket


def _to_float(x):
    try:
        return float(x)
    except Exception:
        return 0.0


def _to_int(x):
    try:
        return int(x)
    except Exception:
        return 0


# ── Physical constants (match weedbot.urdf / weedbot.world) ─────────────────
_WHEEL_RADIUS = 0.035        # m
_WHEEL_SEP = 0.14            # m  (left-right centre-to-centre)
_TICKS_PER_REV = 2048        # encoder resolution
_TICKS_PER_METER = _TICKS_PER_REV / (2.0 * math.pi * _WHEEL_RADIUS)


class RobotStateNode(Node):
    def __init__(self):
        super().__init__('robot_state_node')

        # ── Odometry state ───────────────────────────────────────────────
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Previous encoder counts (int32 per wheel: FL, FR, RL, RR)
        self._prev_enc = [0, 0, 0, 0]
        self._first_packet = True

        # ── Subscribers ──────────────────────────────────────────────────
        self.sub = self.create_subscription(
            MicroSensorPacket,
            '/micro/sensor_packet',
            self._sensor_callback,
            10,
        )

        # ── Publishers ───────────────────────────────────────────────────
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.state_pub = self.create_publisher(Twist, '/robot_state', 10)

        # TF broadcaster for odom → base_link transform
        self._tf_broadcaster = TransformBroadcaster(self)

    # ────────────────────────────────────────────────────────────────────
    # Main callback
    # ────────────────────────────────────────────────────────────────────

    def _sensor_callback(self, msg: MicroSensorPacket):
        now = self.get_clock().now().to_msg()

        # ── IMU ──────────────────────────────────────────────────────────
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'imu_link'

        imu.linear_acceleration.x = _to_float(msg.imu_linear[0])
        imu.linear_acceleration.y = _to_float(msg.imu_linear[1])
        imu.linear_acceleration.z = _to_float(msg.imu_linear[2])

        imu.angular_velocity.x = _to_float(msg.imu_angular[0])
        imu.angular_velocity.y = _to_float(msg.imu_angular[1])
        imu.angular_velocity.z = _to_float(msg.imu_angular[2])

        # diagonal covariance — unknown, mark as -1 only for orientation
        imu.orientation_covariance[0] = -1.0
        self.imu_pub.publish(imu)

        # ── Odometry via encoder dead-reckoning ──────────────────────────
        enc = [_to_int(e) for e in msg.wheel_encoder]

        if self._first_packet:
            # Initialise reference counts without computing motion
            self._prev_enc = enc[:]
            self._first_packet = False
        else:
            # Tick deltas per side (average front + rear)
            d_fl = enc[0] - self._prev_enc[0]
            d_fr = enc[1] - self._prev_enc[1]
            d_rl = enc[2] - self._prev_enc[2]
            d_rr = enc[3] - self._prev_enc[3]

            d_left = (d_fl + d_rl) / 2.0    # average left-side ticks
            d_right = (d_fr + d_rr) / 2.0   # average right-side ticks

            # Convert ticks → metres
            dist_left = d_left / _TICKS_PER_METER
            dist_right = d_right / _TICKS_PER_METER

            # Differential-drive kinematics
            d = (dist_left + dist_right) / 2.0          # forward displacement
            dtheta = (dist_right - dist_left) / _WHEEL_SEP  # heading change

            # Midpoint integration (more accurate than simple Euler)
            mid_yaw = self.yaw + dtheta / 2.0
            self.x += d * math.cos(mid_yaw)
            self.y += d * math.sin(mid_yaw)
            self.yaw += dtheta

            self._prev_enc = enc[:]

        # Quaternion from yaw (roll=0, pitch=0)
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # ── Publish Odometry ─────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Velocity in robot frame (from IMU angular + cap_front as forward proxy)
        # In simulation the odom velocity is available; on hardware encoder deltas
        # divided by tick_time give a reasonable estimate.
        dt = _to_float(msg.tick_time) if msg.tick_time > 0.0 else 0.02
        if not self._first_packet:
            d_ticks_left = (_to_int(enc[0]) + _to_int(enc[2])) / 2.0 - (
                self._prev_enc[0] + self._prev_enc[2]) / 2.0
            d_ticks_right = (_to_int(enc[1]) + _to_int(enc[3])) / 2.0 - (
                self._prev_enc[1] + self._prev_enc[3]) / 2.0
            v_left = (d_ticks_left / _TICKS_PER_METER) / dt
            v_right = (d_ticks_right / _TICKS_PER_METER) / dt
            odom.twist.twist.linear.x = (v_left + v_right) / 2.0
            odom.twist.twist.angular.z = (v_right - v_left) / _WHEEL_SEP

        self.odom_pub.publish(odom)

        # ── Broadcast odom → base_link TF ────────────────────────────────
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)

        # ── Robot state (Twist used as a contact/signal bus) ─────────────
        #   linear.x  = cap_front  (1.0 = crop contact → avoid)
        #   linear.y  = belly      (1.0 = weed contact → cut)
        #   angular.z = side caps  (1.0 = left or right contact)
        state = Twist()
        state.linear.x = 1.0 if msg.cap_front else 0.0
        state.linear.y = 1.0 if msg.belly_contact else 0.0
        state.angular.z = 1.0 if (msg.cap_left or msg.cap_right) else 0.0
        self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
