"""Deterministic pure-Python sensor safety tests; no ROS or hardware is loaded."""

import math
from pathlib import Path
import sys
import unittest

CORE = Path(__file__).resolve().parents[1] / 'deweeder_tertill' / 'src' / 'weedbot_core'
sys.path.insert(0, str(CORE))
from weedbot_core.hal.safety_monitor import (  # noqa: E402
    SAFETY_EMERGENCY, SAFETY_NORMAL, SAFETY_SLOW, SAFETY_STOP, SafetyMonitor,
)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class SafetyMonitorTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.monitor = SafetyMonitor(clock=self.clock)

    def healthy(self, monitor=None):
        monitor = monitor or self.monitor
        monitor.update_contacts()
        monitor.update_imu(0.0, 0.0, 9.81, 0.0, 0.0, 0.0)

    def tilt(self, degrees, monitor=None):
        radians = math.radians(degrees)
        (monitor or self.monitor).update_imu(
            9.81 * math.sin(radians), 0.0, 9.81 * math.cos(radians), 0.0, 0.0, 0.0)

    def test_boot_and_each_missing_input_fail_closed(self):
        self.assertEqual(self.monitor.evaluate().state, SAFETY_STOP)
        self.assertFalse(self.monitor.reset()[0])
        self.monitor.update_contacts()
        self.assertEqual(self.monitor.evaluate().state, SAFETY_STOP)
        other = SafetyMonitor(clock=self.clock)
        other.update_imu(0, 0, 9.81, 0, 0, 0)
        self.assertEqual(other.evaluate().state, SAFETY_STOP)
        self.healthy()
        self.assertEqual(self.monitor.evaluate().state, SAFETY_NORMAL)

    def test_expiry_occurs_without_another_sensor_callback(self):
        self.healthy()
        self.clock.now = 0.499
        self.assertEqual(self.monitor.evaluate().state, SAFETY_NORMAL)
        self.clock.now = 0.5
        self.assertEqual(self.monitor.evaluate().state, SAFETY_STOP)
        self.assertIn('stale', self.monitor.evaluate().reason)

    def test_input_deadlines_are_independent(self):
        monitor = SafetyMonitor(clock=self.clock, state_timeout=0.25, imu_timeout=0.5)
        self.healthy(monitor)
        self.clock.now = 0.25
        monitor.update_imu(0, 0, 9.81, 0, 0, 0)
        self.assertEqual(monitor.evaluate().state, SAFETY_STOP)
        self.assertIn('contact input missing or stale', monitor.evaluate().reason)
        monitor.update_contacts()
        self.assertEqual(monitor.evaluate().state, SAFETY_NORMAL)
        self.clock.now = 0.75
        monitor.update_contacts()
        self.assertEqual(monitor.evaluate().state, SAFETY_STOP)
        self.assertIn('IMU input missing or stale', monitor.evaluate().reason)

    def test_side_and_rear_stop_and_belly_slows(self):
        self.healthy()
        for contact in ('side', 'rear'):
            with self.subTest(contact=contact):
                self.monitor.update_contacts(**{contact: True})
                self.assertEqual(self.monitor.evaluate().state, SAFETY_STOP)
                self.assertFalse(self.monitor.reset()[0])
        self.monitor.update_contacts(belly=True)
        self.assertEqual(self.monitor.evaluate().state, SAFETY_SLOW)
        self.assertFalse(self.monitor.reset()[0])
        self.monitor.update_contacts()
        self.assertEqual(self.monitor.evaluate().state, SAFETY_NORMAL)

    def test_static_tilt_works_with_zero_angular_velocity(self):
        self.healthy()
        self.tilt(15)
        self.assertEqual(self.monitor.evaluate().state, SAFETY_SLOW)
        self.tilt(30)
        self.assertEqual(self.monitor.evaluate().state, SAFETY_STOP)
        self.assertAlmostEqual(self.monitor.evaluate().tilt_degrees, 30)
        self.tilt(60)
        self.assertEqual(self.monitor.evaluate().state, SAFETY_EMERGENCY)

    def test_stationary_upside_down_is_latched_emergency(self):
        self.healthy()
        self.monitor.update_imu(0, 0, -9.81, 0, 0, 0)
        status = self.monitor.evaluate()
        self.assertEqual(status.state, SAFETY_EMERGENCY)
        self.assertEqual(status.tilt_degrees, 180)
        self.assertTrue(status.emergency_latched)

    def test_healthy_message_cannot_clear_transient_emergency_before_tick(self):
        self.healthy()
        self.tilt(60)
        # No evaluate/publish between emergency and subsequent healthy sample.
        self.healthy()
        self.assertEqual(self.monitor.evaluate().state, SAFETY_EMERGENCY)
        success, message = self.monitor.reset()
        self.assertTrue(success)
        self.assertIn('HAL', message)
        self.assertEqual(self.monitor.evaluate().state, SAFETY_NORMAL)

    def test_reset_rejects_stale_inputs_and_active_tilt(self):
        self.healthy()
        self.tilt(60)
        self.assertFalse(self.monitor.reset()[0])
        self.clock.now = 1.0
        self.assertFalse(self.monitor.reset()[0])
        self.assertEqual(self.monitor.evaluate().state, SAFETY_EMERGENCY)
        self.healthy()
        self.monitor.update_contacts(rear=True)
        self.assertFalse(self.monitor.reset()[0])
        self.assertTrue(self.monitor.evaluate().emergency_latched)
        self.monitor.update_contacts()
        self.assertTrue(self.monitor.reset()[0])

    def test_angular_rate_emergency_preserves_existing_limit(self):
        for axis in (0, 1):
            for sign in (-1, 1):
                with self.subTest(axis=axis, sign=sign):
                    monitor = SafetyMonitor(clock=self.clock)
                    self.healthy(monitor)
                    rates = [0.0] * 3
                    rates[axis] = sign * 1.5
                    monitor.update_imu(0, 0, 9.81, *rates)
                    self.assertEqual(monitor.evaluate().state, SAFETY_EMERGENCY)

    def test_rate_emergency_is_not_hidden_by_ambiguous_gravity(self):
        self.healthy()
        self.monitor.update_imu(0, 0, 0, 2, 0, 0)
        self.assertEqual(self.monitor.evaluate().state, SAFETY_EMERGENCY)

    def test_invalid_and_gravity_removed_imu_fails_closed(self):
        cases = ((0, 0, 0, 0, 0, 0), (0, 0, 30, 0, 0, 0),
                 (float('nan'), 0, 9.81, 0, 0, 0),
                 (0, 0, 9.81, 0, 0, float('inf')),
                 (True, 0, 9.81, 0, 0, 0))
        for values in cases:
            with self.subTest(values=values):
                self.healthy()
                self.monitor.update_imu(*values)
                self.assertEqual(self.monitor.evaluate().state, SAFETY_STOP)
                self.assertFalse(self.monitor.reset()[0])
        self.healthy()
        self.monitor.update_imu(0, 0, 9.81, 0, 0, 0, valid=False)
        self.assertEqual(self.monitor.evaluate().state, SAFETY_STOP)

    def test_invalid_contact_data_fails_closed(self):
        for values in ({'valid': False}, {'side': float('nan')}, {'rear': 1}):
            with self.subTest(values=values):
                self.healthy()
                self.monitor.update_contacts(**values)
                self.assertEqual(self.monitor.evaluate().state, SAFETY_STOP)

    def test_clock_fault_and_regression_require_new_inputs(self):
        self.healthy()
        self.clock.now = float('nan')
        self.assertEqual(self.monitor.evaluate().state, SAFETY_STOP)
        self.clock.now = 0.1
        self.assertEqual(self.monitor.evaluate().state, SAFETY_STOP)
        self.healthy()
        self.assertEqual(self.monitor.evaluate().state, SAFETY_NORMAL)
        self.clock.now = 0.0
        self.assertEqual(self.monitor.evaluate().state, SAFETY_STOP)
        self.clock.now = 0.2
        self.assertEqual(self.monitor.evaluate().state, SAFETY_STOP)
        self.healthy()
        self.assertEqual(self.monitor.evaluate().state, SAFETY_NORMAL)

    def test_custom_limits_are_used(self):
        monitor = SafetyMonitor(clock=self.clock, tilt_stop_deg=35.0,
                                tilt_emergency_deg=70.0)
        self.healthy(monitor)
        self.tilt(30, monitor)
        self.assertEqual(monitor.evaluate().state, SAFETY_SLOW)
        self.tilt(60, monitor)
        self.assertEqual(monitor.evaluate().state, SAFETY_STOP)

    def test_invalid_limits_are_rejected(self):
        for limits in ({'state_timeout': 0}, {'imu_timeout': float('nan')},
                       {'tilt_stop_deg': 5}, {'tilt_emergency_deg': 181},
                       {'rate_stop_rad_s': 2}, {'gravity_min_m_s2': 20},
                       {'state_timeout': True}, {'unknown': 1}):
            with self.subTest(limits=limits), self.assertRaises(ValueError):
                SafetyMonitor(**limits)


if __name__ == '__main__':
    unittest.main()
