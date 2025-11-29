#!/usr/bin/env python3
"""
motor_controller.py - PID velocity controller with runtime parameters and diagnostics.

Subscribes:
  - /motion_cmd (weedbot_msgs/MotionCmd) : desired linear_x, angular_z
  - /micro/sensor_packet (weedbot_msgs/MicroSensorPacket) : encoders & tick_time
  - /hal/ready (std_msgs/Bool) : HAL driver readiness (optional gating)

Publishes:
  - /motor_ctrl/diag (std_msgs/String) : diagnostics showing pwm setpoints
  - /motor_controller/ready (std_msgs/Bool) : whether controller is enabled for outputs
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from weedbot_msgs.msg import MotionCmd, MicroSensorPacket
from rcl_interfaces.msg import SetParametersResult


class PID:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, dt=0.05, out_min=-255.0, out_max=255.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.integral = 0.0
        self.prev = 0.0
        self.out_min = out_min
        self.out_max = out_max

    def reset(self):
        self.integral = 0.0
        self.prev = 0.0

    def step(self, setpoint, measurement):
        err = setpoint - measurement
        self.integral += err * self.dt
        deriv = (err - self.prev) / self.dt if self.dt > 0 else 0.0
        self.prev = err
        out = self.kp * err + self.ki * self.integral + self.kd * deriv
        if out > self.out_max: out = self.out_max
        if out < self.out_min: out = self.out_min
        return out


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # declare parameters with defaults
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.01)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('max_pwm', 255.0)
        self.declare_parameter('encoder_scale', 1.0)
        self.declare_parameter('wait_for_hal_ready', True)
        self.declare_parameter('hal_ready_timeout_ms', 1000.0)

        # publish readiness
        self._ready_pub = self.create_publisher(Bool, '/motor_controller/ready', 1)

        # read params
        kp = float(self.get_parameter('kp').value)
        ki = float(self.get_parameter('ki').value)
        kd = float(self.get_parameter('kd').value)
        dt = float(self.get_parameter('dt').value)
        max_pwm = float(self.get_parameter('max_pwm').value)
        enc_scale = float(self.get_parameter('encoder_scale').value)

        # internal state
        self.wait_for_hal_ready = bool(self.get_parameter('wait_for_hal_ready').value)
        self.hal_ready_timeout_ms = float(self.get_parameter('hal_ready_timeout_ms').value)
        self._hal_ready = False
        self._controller_enabled = False

        self.left_pid = PID(kp, ki, kd, dt, out_min=-max_pwm, out_max=max_pwm)
        self.right_pid = PID(kp, ki, kd, dt, out_min=-max_pwm, out_max=max_pwm)

        # register param callback
        self.add_on_set_parameters_callback(self._on_param_change)

        # runtime variables
        self.desired_linear = 0.0
        self.latest_enc = [0, 0, 0, 0]
        self.latest_tick_time = 0.05
        self.encoder_scale = enc_scale

        # subscriptions
        self.create_subscription(MotionCmd, '/motion_cmd', self.cb_motion, 10)
        self.create_subscription(MicroSensorPacket, '/micro/sensor_packet', self.cb_sensor, 10)
        # optional hal readiness gating
        self.create_subscription(Bool, '/hal/ready', self._on_hal_ready, 10)

        # timers + publishers
        self._ready_timer = self.create_timer(0.2, self._publish_ready_state)
        self.diag_pub = self.create_publisher(String, '/motor_ctrl/diag', 10)

        self.get_logger().info('MotorController started (PID params: kp=%s ki=%s kd=%s dt=%s)' %
                               (kp, ki, kd, dt))

    def _on_param_change(self, params):
        """
        Called when parameters are set. Update internal variables and return SetParametersResult.
        """
        try:
            for p in params:
                if p.name == 'kp':
                    self.left_pid.kp = float(p.value); self.right_pid.kp = float(p.value)
                elif p.name == 'ki':
                    self.left_pid.ki = float(p.value); self.right_pid.ki = float(p.value)
                elif p.name == 'kd':
                    self.left_pid.kd = float(p.value); self.right_pid.kd = float(p.value)
                elif p.name == 'dt':
                    val = float(p.value)
                    self.left_pid.dt = val; self.right_pid.dt = val
                elif p.name == 'max_pwm':
                    mx = float(p.value)
                    self.left_pid.out_max = mx; self.left_pid.out_min = -mx
                    self.right_pid.out_max = mx; self.right_pid.out_min = -mx
                elif p.name == 'encoder_scale':
                    self.encoder_scale = float(p.value)
                elif p.name == 'wait_for_hal_ready':
                    self.wait_for_hal_ready = bool(p.value)
                    # if user disables gating explicitly, immediately enable controller
                    if not self.wait_for_hal_ready:
                        self._controller_enabled = True
                        self.get_logger().info('wait_for_hal_ready disabled -> enabling controller outputs')
            return SetParametersResult(successful=True)
        except Exception as e:
            self.get_logger().error(f"Failed to apply parameters: {e}")
            return SetParametersResult(successful=False, reason=str(e))

    def _on_hal_ready(self, msg: Bool):
        self._hal_ready = bool(msg.data)
        if self.wait_for_hal_ready and self._hal_ready:
            self._controller_enabled = True

    def _publish_ready_state(self):
        b = Bool()
        b.data = bool(self._controller_enabled)
        self._ready_pub.publish(b)

    def _publish_zero_motor_outputs(self):
        diag = String()
        diag.data = "MOTOR_GATED zeroing_outputs"
        self.diag_pub.publish(diag)

    def cb_motion(self, msg: MotionCmd):
        self.desired_linear = float(msg.linear_x)

    def cb_sensor(self, msg: MicroSensorPacket):
        # store encoder data and tick time
        try:
            enc = msg.wheel_encoder
            self.latest_enc = [int(x) for x in enc]
            self.latest_tick_time = float(getattr(msg, 'tick_time', 0.05) or 0.05)
        except Exception:
            pass

        # gating: if configured to wait and controller not enabled, zero outputs and return
        if self.wait_for_hal_ready and not self._controller_enabled:
            self.get_logger().debug("MotorController gated: HAL not ready. Skipping motor output.")
            self._publish_zero_motor_outputs()
            return

        # compute simple speeds
        left_speed = (self.latest_enc[0] + self.latest_enc[1]) / 2.0 * self.encoder_scale
        right_speed = (self.latest_enc[2] + self.latest_enc[3]) / 2.0 * self.encoder_scale

        setpoint = self.desired_linear

        left_pwm = self.left_pid.step(setpoint, left_speed)
        right_pwm = self.right_pid.step(setpoint, right_speed)

        s = ("PID diag desired=%.3f encL=%.3f encR=%.3f pwmL=%.1f pwmR=%.1f "
             "kp=%.3f ki=%.3f kd=%.3f")
        msg = String(data=s % (setpoint, left_speed, right_speed, left_pwm, right_pwm,
                               self.left_pid.kp, self.left_pid.ki, self.left_pid.kd))
        self.diag_pub.publish(msg)
        self.get_logger().debug(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()