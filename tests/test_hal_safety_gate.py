"""Output-level interlock tests with a fake monotonic clock; no ROS runtime."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'deweeder_tertill/src/weedbot_core'))
from weedbot_core.drivers.mock_driver import MockDriver
from weedbot_core.hal.safety_gate import SafetyGate


class GateTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.driver = MockDriver()
        self.driver.init()
        self.gate = SafetyGate(self.driver, clock=lambda: self.now)

    def neutral(self):
        self.gate.heartbeat()
        self.gate.update_safety(0)
        self.gate.feedback(0.0)
        self.gate.command_motion(0.0, 0.0)
        self.gate.command_tool(False, 0, False)

    def running(self):
        self.neutral()
        self.assertTrue(self.gate.arm()[0])
        self.gate.command_motion(0.2, 0.0)
        self.gate.feedback(0.1)
        self.gate.command_tool(True, 2, True)
        self.assertTrue(all(v > 0 for v in self.driver._last_pwm))
        self.assertEqual(self.driver._last_tool, (True, 2, True))

    def stopped(self):
        self.assertFalse(self.gate.armed)
        self.assertEqual(self.driver._last_pwm, [0.0] * 4)
        self.assertEqual(self.driver._last_tool, (False, 0, False))

    def test_startup_missing_inputs_and_nonzero_arm_rejected(self):
        self.stopped()
        self.assertFalse(self.gate.arm()[0])
        self.neutral()
        self.gate.command_motion(0.2, 0.0)
        self.assertFalse(self.gate.arm()[0])
        self.stopped()

    def test_independent_loss_of_each_source_stops_without_new_commands(self):
        for source in ('heartbeat', 'safety', 'feedback', 'motion', 'tool'):
            with self.subTest(source=source):
                self.setUp()
                self.running()
                self.now += 0.5
                for other in self.gate.received:
                    if other != source:
                        self.gate.received[other] = self.now
                self.gate.tick()
                self.stopped()
                self.assertIn(source, self.gate.reason)

    def test_late_heartbeat_cannot_hide_expiry(self):
        self.running()
        self.now += 0.5
        self.gate.heartbeat()
        self.stopped()
        self.neutral()
        self.stopped()
        self.assertTrue(self.gate.arm()[0])
        self.assertEqual(self.driver._last_tool, (False, 0, False))

    def test_emergency_stop_immediate_latched_and_requires_neutral_rearm(self):
        self.running()
        self.gate.emergency_stop(True)
        self.stopped()
        self.neutral()
        self.assertFalse(self.gate.arm()[0])
        self.gate.emergency_stop(False)
        self.stopped()
        self.assertTrue(self.gate.arm()[0])

    def test_safety_stop_and_emergency_immediate(self):
        for state in (2, 3, 99):
            self.setUp()
            self.running()
            self.gate.update_safety(state)
            self.stopped()
            self.gate.update_safety(0)
            self.stopped()
            self.assertFalse(self.gate.arm()[0])

    def test_each_tool_channel_neutralized(self):
        for tool in ((True, 0, False), (False, 1, False), (False, 2, False), (False, 0, True)):
            self.setUp()
            self.running()
            self.gate.command_tool(*tool)
            self.gate.trip('test')
            self.stopped()

    def test_stationary_reverse_turn_and_feedback_loss_stop_tool(self):
        for motion in ((0.0, 0.0), (-0.1, 0.0), (0.2, 0.1)):
            self.setUp()
            self.running()
            self.gate.command_motion(*motion)
            self.stopped()
        self.setUp()
        self.running()
        self.gate.feedback(0.0)
        self.stopped()

    def test_tool_request_without_measured_movement_rejected(self):
        self.neutral()
        self.gate.arm()
        self.gate.command_motion(0.2, 0.0)
        self.gate.command_tool(False, 2, True)
        self.stopped()

    def test_invalid_motion_tool_feedback_and_health(self):
        actions = [lambda: self.gate.command_motion(float('nan'), 0),
                   lambda: self.gate.command_motion(0, float('inf')),
                   lambda: self.gate.command_tool(True, 3, True),
                   lambda: self.gate.feedback(float('nan')),
                   lambda: self.gate.feedback(3.0)]
        for action in actions:
            self.setUp()
            self.running()
            action()
            self.stopped()
        self.setUp()
        self.running()
        self.driver._alive = False
        self.gate.tick()
        self.stopped()

    def test_driver_failure_does_not_prevent_remaining_stop_attempts(self):
        self.running()
        original = self.driver.set_pwm
        def fail_active(channel, duty):
            if duty:
                raise OSError('write failed')
            original(channel, duty)
        self.driver.set_pwm = fail_active
        self.gate.command_motion(0.3, 0.0)
        self.stopped()
        self.driver.emergency_stop = lambda: (_ for _ in ()).throw(OSError('estop failed'))
        self.neutral()
        self.assertFalse(self.gate.arm()[0])
        self.stopped()
        self.assertTrue(self.gate.stop_errors)

    def test_slow_limits_all_wheels(self):
        self.running()
        before = self.driver._last_pwm[:]
        self.gate.update_safety(1)
        self.assertEqual(self.driver._last_pwm, [v / 2 for v in before])

    def test_shutdown_clears_every_output(self):
        self.running()
        self.gate.shutdown()
        self.stopped()
        self.assertFalse(self.driver.is_healthy())

    def test_unsupported_backend_cannot_arm(self):
        self.neutral()
        self.driver.supports_tool = False
        self.assertFalse(self.gate.arm()[0])

    def test_invalid_types_clock_and_measured_turn(self):
        for action in (lambda: self.gate.command_motion('bad', 0),
                       lambda: self.gate.command_motion(True, 0),
                       lambda: self.gate.update_safety(False),
                       lambda: self.gate.feedback(None),
                       lambda: self.gate.feedback(0.1, 0.2)):
            self.setUp()
            self.running()
            action()
            self.stopped()
        for bad_clock in (float('nan'), float('inf'), 1.0):
            self.setUp()
            self.running()
            self.now = bad_clock
            self.gate.tick()
            self.stopped()

    def test_rearm_rejects_measured_motion(self):
        self.neutral()
        self.gate.feedback(0.2)
        self.assertFalse(self.gate.arm()[0])
        self.gate.feedback(0.0, 0.2)
        self.assertFalse(self.gate.arm()[0])

    def test_invalid_timeout_rejected(self):
        for timeout in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                SafetyGate(self.driver, timeout=timeout)


if __name__ == '__main__':
    unittest.main()
