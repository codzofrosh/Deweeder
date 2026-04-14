#!/usr/bin/env python3
"""
data_logger_node.py
───────────────────
Logs motion data to a timestamped CSV file for offline hardware parameter
estimation (motor torque, power, speed requirements for CAD design).

What it records (at 10 Hz)
──────────────────────────
  ros_time_s, pos_x, pos_y, vel_linear, vel_angular,
  cmd_linear, cmd_angular, contact_front, contact_belly,
  joint_vel_fl, joint_vel_fr, joint_vel_rl, joint_vel_rr

Hardware sizing outputs (published at 1 Hz on /logger/stats)
────────────────────────────────────────────────────────────
  peak linear velocity     → choose motor no-load speed
  peak angular velocity    → informs turning torque
  distance travelled       → range / battery sizing
  estimated drive torque   → T = m * g * Crr * r  (rolling resistance model)
  estimated drive power    → P = T * ω_wheel = T * v / r

These are minimum estimates assuming flat ground and no acceleration.
Apply a safety factor of ≥ 2× for real-world conditions (wet grass, slopes).

Topics subscribed
─────────────────
  /odom           (nav_msgs/Odometry)
  /motion_cmd     (weedbot_msgs/MotionCmd)
  /robot_state    (geometry_msgs/Twist)
  /joint_states   (sensor_msgs/JointState)   — optional, present when URDF
                                               JointStatePublisher plugin is loaded

Parameters
──────────
  log_dir             (string,  default '~')   — output directory
  robot_mass_kg       (float,   default 5.0)
  wheel_radius_m      (float,   default 0.035)
  rolling_resistance  (float,   default 0.05)  — dimensionless Crr
"""

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from weedbot_msgs.msg import MotionCmd


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('log_dir', os.path.expanduser('~'))
        self.declare_parameter('robot_mass_kg', 5.0)
        self.declare_parameter('wheel_radius_m', 0.035)
        self.declare_parameter('rolling_resistance', 0.05)

        log_dir = str(self.get_parameter('log_dir').value)
        self._mass = float(self.get_parameter('robot_mass_kg').value)
        self._wheel_r = float(self.get_parameter('wheel_radius_m').value)
        self._Crr = float(self.get_parameter('rolling_resistance').value)

        # ── Open CSV ─────────────────────────────────────────────────────
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(log_dir, f'weedbot_log_{ts}.csv')
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow([
            'ros_time_s',
            'pos_x', 'pos_y',
            'vel_linear', 'vel_angular',
            'cmd_linear', 'cmd_angular',
            'contact_front', 'contact_belly',
            'joint_vel_fl', 'joint_vel_fr',
            'joint_vel_rl', 'joint_vel_rr',
        ])

        # ── State ─────────────────────────────────────────────────────────
        self._vel_linear = 0.0
        self._vel_angular = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._contact_front = 0
        self._contact_belly = 0
        self._joint_vels = {'wheel_fl_joint': 0.0, 'wheel_fr_joint': 0.0,
                            'wheel_rl_joint': 0.0, 'wheel_rr_joint': 0.0}

        # Running stats
        self._peak_linear = 0.0
        self._peak_angular = 0.0
        self._peak_wheel_vel = 0.0
        self._distance = 0.0
        self._prev_pos = None
        self._row_count = 0

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(MotionCmd, '/motion_cmd', self._motion_cmd_cb, 10)
        self.create_subscription(Twist, '/robot_state', self._robot_state_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        # ── Publishers ───────────────────────────────────────────────────
        self._stats_pub = self.create_publisher(String, '/logger/stats', 10)

        # ── Timers ───────────────────────────────────────────────────────
        self.create_timer(0.1, self._log_row)       # 10 Hz logging
        self.create_timer(1.0, self._publish_stats)  # 1 Hz stats

        self.get_logger().info(f'DataLogger: writing to {self._csv_path}')

    # ────────────────────────────────────────────────────────────────────
    # Callbacks
    # ────────────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._pos_x = msg.pose.pose.position.x
        self._pos_y = msg.pose.pose.position.y
        self._vel_linear = msg.twist.twist.linear.x
        self._vel_angular = msg.twist.twist.angular.z

        pos = (self._pos_x, self._pos_y)
        if self._prev_pos is not None:
            self._distance += math.hypot(
                pos[0] - self._prev_pos[0],
                pos[1] - self._prev_pos[1],
            )
        self._prev_pos = pos

        self._peak_linear = max(self._peak_linear, abs(self._vel_linear))
        self._peak_angular = max(self._peak_angular, abs(self._vel_angular))

    def _motion_cmd_cb(self, msg: MotionCmd):
        self._cmd_linear = msg.linear_x
        self._cmd_angular = msg.angular_z

    def _robot_state_cb(self, msg: Twist):
        self._contact_front = 1 if msg.linear.x > 0.5 else 0
        self._contact_belly = 1 if msg.linear.y > 0.5 else 0

    def _joint_state_cb(self, msg: JointState):
        for name, vel in zip(msg.name, msg.velocity):
            if name in self._joint_vels:
                self._joint_vels[name] = vel
                self._peak_wheel_vel = max(self._peak_wheel_vel, abs(vel))

    # ────────────────────────────────────────────────────────────────────
    # Logging and stats
    # ────────────────────────────────────────────────────────────────────

    def _log_row(self):
        t = self.get_clock().now().nanoseconds / 1e9
        self._writer.writerow([
            f'{t:.3f}',
            f'{self._pos_x:.4f}', f'{self._pos_y:.4f}',
            f'{self._vel_linear:.4f}', f'{self._vel_angular:.4f}',
            f'{self._cmd_linear:.4f}', f'{self._cmd_angular:.4f}',
            self._contact_front, self._contact_belly,
            f'{self._joint_vels["wheel_fl_joint"]:.4f}',
            f'{self._joint_vels["wheel_fr_joint"]:.4f}',
            f'{self._joint_vels["wheel_rl_joint"]:.4f}',
            f'{self._joint_vels["wheel_rr_joint"]:.4f}',
        ])
        self._row_count += 1
        if self._row_count % 600 == 0:   # flush every minute of data
            self._csv_file.flush()

    def _publish_stats(self):
        g = 9.81

        # Rolling-resistance torque per driven wheel pair (both sides combined)
        # T_roll = m * g * Crr * r
        torque_roll = self._mass * g * self._Crr * self._wheel_r

        # Peak drive power estimate: P = T * v / r
        power_est = torque_roll * self._peak_linear / self._wheel_r if self._wheel_r > 0 else 0.0

        # Peak wheel angular velocity from joint states (if available)
        omega_peak = (
            self._peak_wheel_vel
            if self._peak_wheel_vel > 0
            else self._peak_linear / self._wheel_r
        )

        stats = (
            f'rows={self._row_count} '
            f'dist={self._distance:.2f}m | '
            f'peak_v={self._peak_linear:.3f}m/s '
            f'peak_w={self._peak_angular:.3f}rad/s '
            f'peak_wheel_omega={omega_peak:.2f}rad/s | '
            f'[HW-SIZE] T_roll>={torque_roll * 2:.4f}Nm '
            f'P_drive>={power_est:.3f}W '
            f'(x2 safety factor recommended)'
        )
        msg = String()
        msg.data = stats
        self._stats_pub.publish(msg)

    # ────────────────────────────────────────────────────────────────────
    # Shutdown
    # ────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._csv_file.flush()
        self._csv_file.close()
        self.get_logger().info(
            f'DataLogger: saved {self._row_count} rows → {self._csv_path}'
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
