#!/usr/bin/env python3
"""
safety_node.py
──────────────
Monitors robot state and IMU data; publishes a SafetyCmd whenever
the safety level changes.

Safety levels
─────────────
  0  NORMAL     → full speed, all operations permitted
  1  SLOW        → belly contact (weed) or mild tilt detected → halve speed
  2  STOP        → cap sensors triggered on sides/rear or steep tilt → stop
  3  EMERGENCY   → extreme tilt (tip-over risk) → emergency stop

Subscribers
───────────
  /robot_state  (geometry_msgs/Twist)   contact signals from robot_state_node
  /imu          (sensor_msgs/Imu)       orientation/angular-rate from IMU

Publisher
─────────
  /safety_cmd   (weedbot_msgs/SafetyCmd)   emitted on every cycle @ 20 Hz
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from weedbot_msgs.msg import SafetyCmd

# ── Safety level constants ───────────────────────────────────────────────────
SAFETY_NORMAL = 0
SAFETY_SLOW = 1
SAFETY_STOP = 2
SAFETY_EMERGENCY = 3

# ── Tilt thresholds (lateral / longitudinal angular velocity, rad/s) ─────────
# These trigger on dynamic tilt, not on static orientation.
# Tune these for your actual robot geometry.
TILT_SLOW_RAD_S = 0.30     # rad/s — reduces speed
TILT_STOP_RAD_S = 0.70     # rad/s — stops robot
TILT_EMERGENCY_RAD_S = 1.5  # rad/s — emergency stop (tip-over risk)


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # ── Internal state ───────────────────────────────────────────────
        self._contact_state = SAFETY_NORMAL   # from /robot_state
        self._tilt_state = SAFETY_NORMAL      # from /imu

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(
            Twist, '/robot_state', self._state_cb, 10)
        self.create_subscription(
            Imu, '/imu', self._imu_cb, 10)

        # ── Publisher ────────────────────────────────────────────────────
        self._pub = self.create_publisher(SafetyCmd, '/safety_cmd', 10)

        # Publish at 20 Hz so behavior_node always has a fresh safety level
        self.create_timer(0.05, self._publish)

        self.get_logger().info('SafetyNode started')

    # ────────────────────────────────────────────────────────────────────
    # Contact/cap sensor callback  (from robot_state_node /robot_state)
    # ────────────────────────────────────────────────────────────────────

    def _state_cb(self, state: Twist):
        """
        Signal mapping (set by robot_state_node):
          linear.x  > 0.5  →  cap_front  (crop → handled by behavior, not safety)
          linear.y  > 0.5  →  belly_contact (weed → SLOW so behavior can act)
          angular.z > 0.5  →  cap_left or cap_right (side contact → STOP)
        """
        belly = state.linear.y > 0.5
        side_cap = state.angular.z > 0.5

        if side_cap:
            self._contact_state = SAFETY_STOP
        elif belly:
            # Slow down so the behavior node can stop cleanly and start cutting
            self._contact_state = SAFETY_SLOW
        else:
            self._contact_state = SAFETY_NORMAL

    # ────────────────────────────────────────────────────────────────────
    # IMU tilt detection
    # ────────────────────────────────────────────────────────────────────

    def _imu_cb(self, msg: Imu):
        """
        Detect tilt from angular velocity (roll-rate and pitch-rate).
        Using angular velocity rather than orientation because the IMU
        in simulation may not publish orientation quaternions.
        """
        roll_rate = abs(msg.angular_velocity.x)
        pitch_rate = abs(msg.angular_velocity.y)
        max_rate = max(roll_rate, pitch_rate)

        if max_rate >= TILT_EMERGENCY_RAD_S:
            self._tilt_state = SAFETY_EMERGENCY
            self.get_logger().error(
                f'EMERGENCY: extreme tilt rate {max_rate:.2f} rad/s',
                throttle_duration_sec=1.0,
            )
        elif max_rate >= TILT_STOP_RAD_S:
            self._tilt_state = SAFETY_STOP
            self.get_logger().warn(
                f'STOP: tilt rate {max_rate:.2f} rad/s',
                throttle_duration_sec=1.0,
            )
        elif max_rate >= TILT_SLOW_RAD_S:
            self._tilt_state = SAFETY_SLOW
        else:
            self._tilt_state = SAFETY_NORMAL

    # ────────────────────────────────────────────────────────────────────
    # Periodic publisher
    # ────────────────────────────────────────────────────────────────────

    def _publish(self):
        # Always publish the most severe active safety level
        level = max(self._contact_state, self._tilt_state)
        cmd = SafetyCmd()
        cmd.state = level
        self._pub.publish(cmd)


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
