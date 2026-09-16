"""Fresh-source simulation assembly checks without ROS/Gazebo."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'deweeder_tertill/src/weedbot_core'))
from weedbot_core.sim_sensor_cache import SimSensorCache


class CacheTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.cache = SimSensorCache(clock=lambda: self.now)

    def fresh(self, stamp=1.0):
        self.cache.update_contact('front_contact', False)
        self.cache.update_contact('belly_contact', False)
        self.cache.update_imu([0.0, 0.0, 9.81], [0.0, 0.0, 0.0])
        self.cache.update_odom(stamp, 0.2, 0.0)

    def test_no_defaults_or_duplicate_republication(self):
        self.assertIsNone(self.cache.packet())
        self.fresh()
        self.assertEqual(self.cache.packet()['tick_time'], 0.0)
        self.assertIsNone(self.cache.packet())

    def test_every_source_required_fresh(self):
        for source in self.cache.SOURCES:
            self.setUp()
            self.fresh()
            self.cache.packet()
            self.now += 0.5
            self.fresh(1.1)
            self.cache._received[source] = self.now - 0.5
            self.assertIsNone(self.cache.packet(), source)

    def test_interval_tracks_odom_not_publish_timer(self):
        self.fresh()
        first = self.cache.packet()
        self.cache.update_odom(1.1, 0.2, 0.0)
        self.cache.update_odom(1.2, 0.2, 0.0)
        second = self.cache.packet()
        self.assertAlmostEqual(second['tick_time'], 0.2)
        self.assertTrue(all(b > a for a, b in zip(first['wheel_encoder'], second['wheel_encoder'])))

    def test_recovery_invalidates_velocity_reference(self):
        self.fresh()
        self.cache.packet()
        self.now += 1.0
        self.assertIsNone(self.cache.packet())
        self.fresh(2.0)
        self.assertIsNone(self.cache.packet())
        self.fresh(2.1)
        self.assertEqual(self.cache.packet()['tick_time'], 0.0)

    def test_invalid_imu_and_nonprogressing_odom_inhibit(self):
        self.fresh()
        self.cache.update_imu([0, 0, float('nan')], [0, 0, 0])
        self.assertIsNone(self.cache.packet())
        self.fresh(1.1)
        self.cache.update_odom(1.1, 0.2, 0.0)
        self.assertIsNone(self.cache.packet())

    def test_invalid_clock_inhibits(self):
        self.fresh()
        self.now = float('nan')
        self.assertIsNone(self.cache.packet())
