#!/usr/bin/env python3
"""
HAL hardware driver (GPIO/PWM) with heartbeat watchdog and safety guard.

Parameters:
  - hw_mode (bool): enable real hardware backend (otherwise mock)
  - hw_backend (string): name of backend ('mock','serial','socketcan')
  - heartbeat_timeout_ms (float): heartbeat stale timeout in milliseconds
  - pinmap_file (string): optional path to pinmap yaml

Publishes:
  - /hal/ready (std_msgs/Bool)    -> True when driver init succeeded
  - /hal/diag  (std_msgs/String)  -> lightweight diagnostic messages

Subscribes:
  - /motion_cmd (weedbot_msgs/MotionCmd)
  - /tool_cmd   (weedbot_msgs/ToolCmd)
  - /safety_cmd (weedbot_msgs/SafetyCmd)
  - /heartbeat  (std_msgs/Float32)
"""
from __future__ import annotations

import importlib
import traceback
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from weedbot_msgs.msg import MotionCmd, ToolCmd, SafetyCmd

MAX_PWM = 255.0
MIN_PWM = -255.0

_BACKEND_MAP = {
    "mock": "weedbot_core.drivers.mock_driver.MockDriver",
    "serial": "weedbot_core.drivers.serial_driver.SerialDriver",
    "socketcan": "weedbot_core.drivers.socketcan_driver.SocketCANDriver",
}


class HalHW(Node):
    def __init__(self) -> None:
        super().__init__("hal_hw_gpio")

        # --- declare parameters (single place) ---
        self.declare_parameter("hw_mode", False)
        self.declare_parameter("hw_backend", "mock")
        self.declare_parameter("heartbeat_timeout_ms", 500.0)
        self.declare_parameter("pinmap_file", "")

        # --- read raw values and coerce to expected types (robust to strings) ---
        def to_bool(v) -> bool:
            if isinstance(v, bool):
                return v
            if v is None:
                return False
            s = str(v).strip().lower()
            return s in ("1", "true", "yes", "on")

        raw_hw_mode = self.get_parameter("hw_mode").value
        raw_hw_backend = self.get_parameter("hw_backend").value
        raw_hb_timeout = self.get_parameter("heartbeat_timeout_ms").value
        raw_pinmap = self.get_parameter("pinmap_file").value

        self.hw_mode: bool = to_bool(raw_hw_mode)
        self.hw_backend: str = str(raw_hw_backend) if raw_hw_backend is not None else "mock"
        try:
            self.heartbeat_timeout_ms: float = float(raw_hb_timeout)
        except Exception:
            self.heartbeat_timeout_ms = 500.0
        self.pinmap_file: str = str(raw_pinmap or "")

        # publishers
        self.hal_ready_pub = self.create_publisher(Bool, "/hal/ready", 10)
        self.diag_pub = self.create_publisher(String, "/hal/diag", 10)

        # runtime state
        self.safety = 0
        self.latest_motion: MotionCmd = MotionCmd()
        self.latest_tool: ToolCmd = ToolCmd()
        self.last_heartbeat = self.get_clock().now()
        self._driver_ready = False
        self.driver = None

        # subscriptions
        self.create_subscription(MotionCmd, "/motion_cmd", self.cb_motion, 10)
        self.create_subscription(ToolCmd, "/tool_cmd", self.cb_tool, 10)
        self.create_subscription(SafetyCmd, "/safety_cmd", self.cb_safety, 10)
        self.create_subscription(Float32, "/heartbeat", self.cb_heartbeat, 10)

        # load backend driver
        backend_path = (
            _BACKEND_MAP.get(self.hw_backend, _BACKEND_MAP["mock"])
            if self.hw_mode
            else _BACKEND_MAP["mock"]
        )
        try:
            module_name, class_name = backend_path.rsplit(".", 1)
            module = importlib.import_module(module_name)
            driver_cls = getattr(module, class_name)
            cfg = {"pinmap_file": self.pinmap_file} if self.pinmap_file else {}
            self.driver = driver_cls(config=cfg)
            ok = bool(self.driver.init())
            self._driver_ready = ok
            self.get_logger().info(f"Loaded backend '{backend_path}' init -> {ok}")
        except Exception as e:
            self.get_logger().error(f"Failed to load backend '{backend_path}': {e}")
            self.get_logger().debug(traceback.format_exc())
            # fallback to mock driver
            try:
                mod = importlib.import_module("weedbot_core.drivers.mock_driver")
                Mock = getattr(mod, "MockDriver")
                self.driver = Mock(config={})
                self._driver_ready = bool(self.driver.init())
                self.get_logger().warning(
                    "Fell back to MockDriver (init -> %s)" % self._driver_ready
                )
            except Exception:
                self._driver_ready = False
                self.get_logger().error("MockDriver fallback failed")

        # periodic timers
        self.create_timer(0.5, self._publish_ready)
        self.get_logger().info(
            f"HAL hardware driver started (hw_mode={self.hw_mode} hw_backend={self.hw_backend} hb_timeout_ms={self.heartbeat_timeout_ms})"
        )

    # --- callbacks ---
    def cb_heartbeat(self, msg: Float32) -> None:
        # update a monotonic clock timestamp
        self.last_heartbeat = self.get_clock().now()

    def cb_safety(self, msg: SafetyCmd) -> None:
        # safety state integer; higher values = more restrictive
        try:
            self.safety = int(msg.state)
        except Exception:
            self.safety = 0

    def cb_motion(self, msg: MotionCmd) -> None:
        self.latest_motion = msg
        self.apply_motion(msg)

    def cb_tool(self, msg: ToolCmd) -> None:
        self.latest_tool = msg
        self.apply_tool(msg)

    # --- helpers ---
    def _heartbeat_stale(self) -> bool:
        now = self.get_clock().now()
        elapsed_ms = (now - self.last_heartbeat).nanoseconds / 1e6
        return elapsed_ms > float(self.heartbeat_timeout_ms)

    def _publish_ready(self) -> None:
        b = Bool()
        b.data = bool(self._driver_ready)
        self.hal_ready_pub.publish(b)
        # publish lightweight diag
        s = String()
        s.data = f"driver_ready={self._driver_ready} safety={self.safety} hb_stale={self._heartbeat_stale()}"
        self.diag_pub.publish(s)

    # core behavior: compute PWMs and either call driver or publish diag
    def apply_motion(self, motion: MotionCmd) -> None:
        if self._heartbeat_stale():
            pwm_l = 0.0
            pwm_r = 0.0
            self.get_logger().warning("Heartbeat stale -> zeroing motion outputs")
        else:
            if self.safety >= 2:
                pwm_l = 0.0
                pwm_r = 0.0
            else:
                scale = 200.0
                turn_scale = 100.0
                base = float(motion.linear_x) * scale
                diff = float(motion.angular_z) * turn_scale
                pwm_l = max(min(base - diff, MAX_PWM), MIN_PWM)
                pwm_r = max(min(base + diff, MAX_PWM), MIN_PWM)

        if self._driver_ready and self.driver is not None and hasattr(self.driver, "set_pwm"):
            try:
                # assume channel 0 = left, 1 = right in this HAL
                self.driver.set_pwm(0, pwm_l / MAX_PWM)  # normalized -1..1
                self.driver.set_pwm(1, pwm_r / MAX_PWM)
            except Exception as e:
                self.get_logger().error(f"Driver set_pwm failed: {e}")
                self.get_logger().debug(traceback.format_exc())
        else:
            # simulation diagnostic
            self.diag_pub.publish(
                String(data=f"SIM_MOTION pwm_l={pwm_l:.1f} pwm_r={pwm_r:.1f} safety={self.safety} hb_stale={self._heartbeat_stale()}")
            )

        self.get_logger().debug(f"apply_motion -> L={pwm_l:.1f} R={pwm_r:.1f} safety={self.safety}")

    def apply_tool(self, tool: ToolCmd) -> None:
        if self._heartbeat_stale():
            front = False
        else:
            front = False if self.safety >= 2 else bool(tool.front_trimmer_on)

        if self._driver_ready and self.driver is not None:
            try:
                if hasattr(self.driver, "apply_tool"):
                    self.driver.apply_tool(front=front, mode=int(tool.belly_trimmer_mode), safety=int(self.safety))
            except Exception as e:
                self.get_logger().error(f"Driver apply_tool failed: {e}")
                self.get_logger().debug(traceback.format_exc())
        else:
            self.diag_pub.publish(String(data=f"SIM_TOOL front={front} mode={tool.belly_trimmer_mode} safety={self.safety}"))

        self.get_logger().info(f"apply_tool -> front={front} mode={tool.belly_trimmer_mode} safety={self.safety}")

    def destroy_node(self) -> None:
        try:
            if self.driver:
                self.driver.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HalHW()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
