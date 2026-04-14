#!/usr/bin/env python3
"""
behavior_node.py
────────────────
High-level state machine that decides what the robot does based on
sensor signals forwarded through /robot_state (Twist).

State machine
─────────────

  MOVING_FORWARD  ──── cap_front ────►  AVOID_OBSTACLE
        ▲                                      │
        │                               reverse + turn
        │◄──────────────────────────────────────┘
        │
        │──── belly_contact ──►  WEED_DETECTED
                                       │
                                    stop
                                       │
                                       ▼
                                    CUTTING  (tool on, timer ~3 s)
                                       │
                                       ▼
                               MOVING_FORWARD

Signal mapping (from robot_state Twist):
  linear.x  > 0.5  →  cap_front   (crop → AVOID)
  linear.y  > 0.5  →  belly_contact (weed → WEED_DETECTED)

Safety gating:
  safety_state ≥ STOP  →  zero all commands
  safety_state == SLOW →  halve speed
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from weedbot_msgs.msg import MotionCmd, ToolCmd, SafetyCmd

# ── Safety states ────────────────────────────────────────────────────────────
SAFETY_NORMAL = 0
SAFETY_SLOW = 1
SAFETY_STOP = 2
SAFETY_EMERGENCY = 3

# ── Behaviour states ─────────────────────────────────────────────────────────
STATE_MOVING_FORWARD = 'MOVING_FORWARD'
STATE_AVOID_OBSTACLE = 'AVOID_OBSTACLE'
STATE_WEED_DETECTED = 'WEED_DETECTED'
STATE_CUTTING = 'CUTTING'

# ── Timing constants ─────────────────────────────────────────────────────────
REVERSE_DURATION = 1.2    # seconds to drive backwards when avoiding
TURN_DURATION = 1.0       # seconds to turn after reversing
CUTTING_DURATION = 3.0    # seconds the cutting tool stays active


class BehaviorNode(Node):
    def __init__(self):
        super().__init__('behavior_node')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('speed_scale', 0.2)
        self.declare_parameter('tool_enable', True)
        self.declare_parameter('heartbeat_hz', 10.0)

        # ── Internal state ───────────────────────────────────────────────
        self._state = STATE_MOVING_FORWARD
        self._safety_state = SAFETY_NORMAL
        self._phase_timer = 0.0      # seconds spent in current sub-phase
        self._avoid_phase = 'reverse'  # 'reverse' | 'turn'

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(
            Twist, '/robot_state', self._robot_state_cb, 10)
        self.create_subscription(
            SafetyCmd, '/safety_cmd', self._safety_cb, 10)

        # ── Publishers ───────────────────────────────────────────────────
        self._motion_pub = self.create_publisher(MotionCmd, '/motion_cmd', 10)
        self._tool_pub = self.create_publisher(ToolCmd, '/tool_cmd', 10)
        self._hb_pub = self.create_publisher(Float32, '/heartbeat', 10)

        # ── Timers ───────────────────────────────────────────────────────
        # Main control loop at 10 Hz
        self._dt = 0.1
        self.create_timer(self._dt, self._control_loop)

        # Heartbeat
        hz = float(self.get_parameter('heartbeat_hz').value or 10.0)
        self.create_timer(1.0 / hz, self._publish_heartbeat)

        self.get_logger().info('BehaviorNode started — state: MOVING_FORWARD')

    # ────────────────────────────────────────────────────────────────────
    # Subscribers
    # ────────────────────────────────────────────────────────────────────

    def _safety_cb(self, msg: SafetyCmd):
        prev = self._safety_state
        self._safety_state = int(msg.state)
        if prev != self._safety_state:
            self.get_logger().info(
                f'Safety state: {prev} → {self._safety_state}')

    def _robot_state_cb(self, state: Twist):
        """
        Receives sensor signals encoded as a Twist (see robot_state_node).
        We only use this to trigger state transitions — the actual motion
        commands are issued from the timed control loop.
        """
        cap_front = state.linear.x > 0.5
        belly = state.linear.y > 0.5

        if self._state == STATE_MOVING_FORWARD:
            if belly:
                self._transition(STATE_WEED_DETECTED)
            elif cap_front:
                self._transition(STATE_AVOID_OBSTACLE)

        elif self._state == STATE_WEED_DETECTED:
            # Already stopped; wait for cutting timer started in control loop
            pass

        elif self._state == STATE_CUTTING:
            # Don't interrupt cutting cycle on new contacts
            pass

        elif self._state == STATE_AVOID_OBSTACLE:
            # Don't interrupt avoidance on new contacts
            pass

    # ────────────────────────────────────────────────────────────────────
    # Control loop (10 Hz)
    # ────────────────────────────────────────────────────────────────────

    def _control_loop(self):
        scale = float(self.get_parameter('speed_scale').value)
        tool_enable = bool(self.get_parameter('tool_enable').value)

        motion = MotionCmd()
        tool = ToolCmd()

        if self._state == STATE_MOVING_FORWARD:
            motion.linear_x = scale
            motion.angular_z = 0.0
            tool.front_trimmer_on = False
            tool.belly_trimmer_mode = 0
            tool.agitator_pulse = False

        elif self._state == STATE_AVOID_OBSTACLE:
            self._phase_timer += self._dt

            if self._avoid_phase == 'reverse':
                motion.linear_x = -scale * 0.6   # reverse at 60 % speed
                motion.angular_z = 0.0
                if self._phase_timer >= REVERSE_DURATION:
                    self._phase_timer = 0.0
                    self._avoid_phase = 'turn'
                    self.get_logger().info('AVOID: reversing done → turning')

            else:  # 'turn'
                motion.linear_x = 0.0
                motion.angular_z = scale * 3.0   # turn in place (left)
                if self._phase_timer >= TURN_DURATION:
                    self._phase_timer = 0.0
                    self._avoid_phase = 'reverse'  # reset for next avoidance
                    self._transition(STATE_MOVING_FORWARD)

            tool.front_trimmer_on = False
            tool.belly_trimmer_mode = 0
            tool.agitator_pulse = False

        elif self._state == STATE_WEED_DETECTED:
            # Stop immediately, then let the next tick start cutting
            motion.linear_x = 0.0
            motion.angular_z = 0.0
            tool.front_trimmer_on = False
            tool.belly_trimmer_mode = 0
            tool.agitator_pulse = False
            self._transition(STATE_CUTTING)

        elif self._state == STATE_CUTTING:
            self._phase_timer += self._dt
            motion.linear_x = 0.0
            motion.angular_z = 0.0

            if tool_enable:
                tool.front_trimmer_on = True
                tool.belly_trimmer_mode = 2   # active cutting mode
                tool.agitator_pulse = (self._phase_timer < 0.5)  # short pulse
            else:
                tool.front_trimmer_on = False
                tool.belly_trimmer_mode = 0
                tool.agitator_pulse = False

            if self._phase_timer >= CUTTING_DURATION:
                self.get_logger().info('CUTTING done → MOVING_FORWARD')
                tool.front_trimmer_on = False
                tool.belly_trimmer_mode = 0
                tool.agitator_pulse = False
                self._transition(STATE_MOVING_FORWARD)

        # ── Apply safety gating ──────────────────────────────────────────
        if self._safety_state >= SAFETY_STOP:
            motion.linear_x = 0.0
            motion.angular_z = 0.0
            tool.front_trimmer_on = False
            tool.belly_trimmer_mode = 0
            tool.agitator_pulse = False
            self.get_logger().warn(
                f'Safety STOP (state={self._safety_state}) — commands zeroed',
                throttle_duration_sec=2.0,
            )
        elif self._safety_state == SAFETY_SLOW:
            motion.linear_x *= 0.5
            motion.angular_z *= 0.5

        self._motion_pub.publish(motion)
        self._tool_pub.publish(tool)

    # ────────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────────

    def _transition(self, new_state: str):
        if new_state != self._state:
            self.get_logger().info(f'State: {self._state} → {new_state}')
        self._state = new_state
        self._phase_timer = 0.0

    def _publish_heartbeat(self):
        msg = Float32()
        msg.data = float(self.get_clock().now().nanoseconds) / 1e9
        self._hb_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
