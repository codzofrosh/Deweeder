#!/usr/bin/env python3
"""
HAL hardware driver (GPIO/PWM) with heartbeat watchdog and safety guard.
Run in simulate mode by default. Add --hw to try pigpio mode.
"""
import argparse
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from weedbot_msgs.msg import MotionCmd, ToolCmd, SafetyCmd
from std_msgs.msg import Float32

MAX_PWM = 255.0
MIN_PWM = -255.0

class HalHW(Node):
    def __init__(self, hw_mode=False, hb_timeout_ms=500):
        super().__init__('hal_hw_gpio')
        self.hw_mode = hw_mode
        self.safety = 0
        self.latest_motion = MotionCmd()
        self.latest_tool = ToolCmd()
        self.last_heartbeat = self.get_clock().now()
        self.heartbeat_timeout_ms = float(hb_timeout_ms)

        # subscriptions
        self.create_subscription(MotionCmd, '/motion_cmd', self.cb_motion, 10)
        self.create_subscription(ToolCmd, '/tool_cmd', self.cb_tool, 10)
        self.create_subscription(SafetyCmd, '/safety_cmd', self.cb_safety, 10)
        self.create_subscription(Float32, '/heartbeat', self.cb_heartbeat, 10)

        # diagnostics
        self.diag_pub = self.create_publisher(String, '/hal/diag', 10)

        # pigpio init (best effort)
        if self.hw_mode:
            try:
                import pigpio
                self.pi = pigpio.pi()
                if not self.pi.connected:
                    self.get_logger().error("pigpio not connected; fallback to simulate")
                    self.hw_mode = False
            except Exception as e:
                self.get_logger().error(f"pigpio init failed: {e}; running in simulate mode")
                self.hw_mode = False

        self.get_logger().info(f"HAL hardware driver started (hw_mode={self.hw_mode} hb_timeout_ms={self.heartbeat_timeout_ms})")

    def cb_heartbeat(self, msg: Float32):
        # update last heartbeat time using node clock (monotonic)
        self.last_heartbeat = self.get_clock().now()

    def cb_safety(self, msg: SafetyCmd):
        self.safety = int(msg.state)

    def cb_motion(self, msg: MotionCmd):
        self.latest_motion = msg
        self.apply_motion(msg)

    def cb_tool(self, msg: ToolCmd):
        self.latest_tool = msg
        self.apply_tool(msg)

    def _heartbeat_stale(self):
        now = self.get_clock().now()
        elapsed_ms = (now - self.last_heartbeat).nanoseconds / 1e6
        return elapsed_ms > self.heartbeat_timeout_ms

    def apply_motion(self, motion: MotionCmd):
        # watchdog for heartbeat
        if self._heartbeat_stale():
            pwm_l = 0.0
            pwm_r = 0.0
            self.get_logger().warn("Heartbeat stale -> zeroing motion outputs")
        else:
            # normal safety guard
            if self.safety >= 2:
                pwm_l = 0.0; pwm_r = 0.0
            else:
                scale = 200.0
                turn_scale = 100.0
                base = motion.linear_x * scale
                diff = motion.angular_z * turn_scale
                pwm_l = max(min(base - diff, MAX_PWM), MIN_PWM)
                pwm_r = max(min(base + diff, MAX_PWM), MIN_PWM)

        if self.hw_mode:
            # hardware PWM handling here (user will implement)
            pass
        else:
            self.diag_pub.publish(String(data=f"SIM_MOTION pwm_l={pwm_l:.1f} pwm_r={pwm_r:.1f} safety={self.safety} hb_stale={self._heartbeat_stale()}"))

        self.get_logger().debug(f"apply_motion -> L={pwm_l:.1f} R={pwm_r:.1f} safety={self.safety}")

    def apply_tool(self, tool: ToolCmd):
        if self._heartbeat_stale():
            front = False
        else:
            front = False if self.safety >= 2 else bool(tool.front_trimmer_on)

        if self.hw_mode:
            pass
        else:
            self.diag_pub.publish(String(data=f"SIM_TOOL front={front} mode={tool.belly_trimmer_mode} safety={self.safety}"))

        self.get_logger().info(f"apply_tool -> front={front} mode={tool.belly_trimmer_mode} safety={self.safety}")

def main():
    import sys, argparse
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--hw', action='store_true', help='enable hardware pigpio mode')
    parser.add_argument('--heartbeat-timeout-ms', type=float, default=500.0)
    known, rest = parser.parse_known_args()
    rclpy.init(args=rest)
    node = HalHW(hw_mode=known.hw, hb_timeout_ms=known.heartbeat_timeout_ms)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
