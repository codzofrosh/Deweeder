"""Independent analytic fixtures for the geometry experiment (stdlib unittest)."""

from dataclasses import replace
import json
import math
from pathlib import Path
import tempfile
import unittest

from simulation.geometry.config import Config, load_config
from simulation.geometry.field import Plant, generate_field
from simulation.geometry.kernel import simulate


class GeometryTests(unittest.TestCase):
    def config(self, **overrides):
        values = dict(row_length=1.0, run_in=0.2, vehicle_speed=1.0,
                      path_offset=0.25, weed_density=0.0, sample_period=0.1)
        values.update(overrides)
        return Config.from_mapping(values)

    def active_events(self, result):
        return [event for event in result['events'] if event['envelope'] == 'active']

    def test_disk_centre_contact_has_analytic_entry_and_exit(self):
        result = simulate(self.config(), [Plant('w', 'weed', 0.5, 0.25, 0.01)])
        events = self.active_events(result)
        self.assertEqual(len(events), 1)
        # Tool begins at x=-0.2; combined disk radius is 0.05 m.
        self.assertAlmostEqual(events[0]['entry_time'], 0.65)
        self.assertAlmostEqual(events[0]['exit_time'], 0.75)
        self.assertEqual(result['summary']['weed_exposures'], 1)

    def test_disk_oblique_contact_matches_pythagorean_chord(self):
        result = simulate(self.config(), [Plant('w', 'weed', 0.5, 0.28, 0.01)])
        event = self.active_events(result)[0]
        # Radius 0.05 and lateral offset 0.03 imply longitudinal reach 0.04.
        self.assertAlmostEqual(event['entry_time'], 0.66)
        self.assertAlmostEqual(event['exit_time'], 0.74)

    def test_tangent_counts_once_and_near_miss_does_not(self):
        # Binary-exact coordinates distinguish genuine tangency from rounding.
        config = self.config(tool_width=0.0625, tool_active_radius=0.03125)
        plants = [Plant('tangent', 'crop', 0.5, 0.296875, 0.015625),
                  Plant('miss', 'crop', 0.5, 0.296875 + 1e-6, 0.015625)]
        events = self.active_events(simulate(config, plants))
        self.assertEqual([event['plant_id'] for event in events], ['tangent'])
        self.assertAlmostEqual(events[0]['entry_time'], 0.7)
        self.assertEqual(events[0]['entry_time'], events[0]['exit_time'])

    def test_hoe_rounded_corner_rejects_bounding_box_false_positive(self):
        config = self.config(mechanism='hoe', command_off=0.3,
                             tool_width=0.08, tool_length=0.02)
        # At switch-off the tool centre is x=0.1. Corner offsets (0.008,
        # 0.008) exceed r=0.01, although each fits an expanded bounding box.
        miss = Plant('corner_miss', 'weed', 0.118, 0.298, 0.01)
        self.assertEqual(simulate(config, [miss])['summary']['weed_exposures'], 0)
        # Moving 1 mm longitudinally and 2 mm laterally inward intersects.
        hit = Plant('corner_hit', 'weed', 0.117, 0.296, 0.01)
        result = simulate(config, [hit])
        self.assertEqual(result['summary']['weed_exposures'], 1)
        self.assertAlmostEqual(self.active_events(result)[0]['entry_time'], 0.299)

    def test_hoe_centre_contact_includes_plant_radius(self):
        result = simulate(self.config(mechanism='hoe', tool_length=0.02),
                          [Plant('w', 'weed', 0.5, 0.25, 0.01)])
        event = self.active_events(result)[0]
        self.assertAlmostEqual(event['entry_time'], 0.68)
        self.assertAlmostEqual(event['exit_time'], 0.72)

    def test_delayed_start_and_coast_down_change_actual_not_commanded_state(self):
        config = self.config(command_on=0.2, tool_response_time=0.3,
                             command_off=0.7, tool_coast_down=0.2)
        plants = [Plant('before_onset', 'weed', 0.1, 0.25, 0.01),
                  Plant('during_coast', 'weed', 0.6, 0.25, 0.01),
                  Plant('after_coast', 'weed', 0.9, 0.25, 0.01)]
        result = simulate(config, plants)
        self.assertEqual([event['plant_id'] for event in self.active_events(result)],
                         ['during_coast'])
        delayed = [row for row in result['timeline']
                   if row['commanded_active'] and not row['actual_active']]
        coasting = [row for row in result['timeline']
                    if not row['commanded_active'] and row['actual_active']]
        self.assertTrue(delayed)
        self.assertTrue(coasting)
        self.assertTrue(all(row['timestamp'] < 0.5 for row in delayed))
        self.assertTrue(all(0.7 <= row['timestamp'] < 0.9 for row in coasting))

    def test_off_before_or_at_delayed_onset_cannot_create_coast_down_activation(self):
        plants = [Plant('w', 'weed', 0.5, 0.25, 0.01)]
        for off in (0.0, 0.4, 0.5):
            with self.subTest(command_off=off):
                result = simulate(self.config(command_off=off,
                                              tool_response_time=0.5,
                                              tool_coast_down=1.0), plants)
                self.assertFalse(self.active_events(result))
                self.assertFalse(any(row['actual_active'] for row in result['timeline']))

    def test_guard_contact_remains_when_tool_is_off(self):
        result = simulate(self.config(command_off=0.0),
                          [Plant('crop', 'crop', 0.5, 0.30, 0.01)])
        self.assertEqual(result['summary']['crop_intersections'], 0)
        self.assertEqual(result['summary']['guard_contacts'], 1)
        self.assertEqual([event['envelope'] for event in result['events']], ['guard'])

    def test_equal_trimmer_and_tine_disks_give_equal_geometry(self):
        plants = [Plant('c', 'crop', 0.2, 0.26, 0.01),
                  Plant('w', 'weed', 0.7, 0.29, 0.01)]
        trimmer = simulate(self.config(mechanism='trimmer'), plants)
        tine = simulate(self.config(mechanism='tine'), plants)
        self.assertEqual(trimmer['events'], tine['events'])
        self.assertEqual(trimmer['summary'], tine['summary'])
        self.assertEqual(trimmer['timeline'], tine['timeline'])

    def test_always_active_geometry_is_speed_invariant(self):
        plants = [Plant('c', 'crop', 0.2, 0.26, 0.01),
                  Plant('near', 'weed', 0.7, 0.29, 0.01),
                  Plant('far', 'weed', 0.7, 0.40, 0.01)]
        slow = simulate(self.config(vehicle_speed=0.25), plants)
        fast = simulate(self.config(vehicle_speed=1.0), plants)
        for metric in ('crop_intersections', 'weed_exposures', 'guard_contacts',
                       'weed_exposure_fraction', 'crop_intersection_fraction'):
            self.assertEqual(slow['summary'][metric], fast['summary'][metric])
        self.assertEqual([event['event_id'] for event in slow['events']],
                         [event['event_id'] for event in fast['events']])
        for slow_event, fast_event in zip(slow['events'], fast['events']):
            self.assertAlmostEqual(slow_event['entry_time'] / 4, fast_event['entry_time'])
            self.assertAlmostEqual(slow_event['exit_time'] / 4, fast_event['exit_time'])

    def test_counts_do_not_depend_on_visualization_sampling(self):
        plants = [Plant('w', 'weed', 0.513, 0.25, 0.001)]
        fine = simulate(self.config(sample_period=0.01), plants)
        coarse = simulate(self.config(sample_period=2.0), plants)
        self.assertEqual(fine['events'], coarse['events'])
        self.assertEqual(fine['summary'], coarse['summary'])

    def test_zero_denominators_and_unmeasured_biological_metrics_are_unavailable(self):
        result = simulate(self.config(), [])
        self.assertEqual(result['events'], [])
        self.assertEqual(result['summary']['crop_count'], 0)
        self.assertEqual(result['summary']['weed_count'], 0)
        for metric in ('weed_exposure_fraction', 'crop_intersection_fraction',
                       'weed_removal_efficiency', 'crop_damage_rate',
                       'energy_wh_per_m', 'cost_inr_per_m'):
            self.assertIsNone(result['summary'][metric])

    def test_no_crops_and_no_weeds_are_independent_denominators(self):
        for kind, absent_metric, present_metric in (
            ('crop', 'weed_exposure_fraction', 'crop_intersection_fraction'),
            ('weed', 'crop_intersection_fraction', 'weed_exposure_fraction'),
        ):
            with self.subTest(kind=kind):
                summary = simulate(self.config(), [Plant('p', kind, 0.5, 0.25, 0.01)])['summary']
                self.assertIsNone(summary[absent_metric])
                self.assertEqual(summary[present_metric], 1.0)

    def test_unique_counting_retains_overlapping_and_inaccessible_targets(self):
        plants = [Plant('c', 'crop', 0.5, 0.25, 0.01),
                  Plant('w1', 'weed', 0.5, 0.25, 0.01),
                  Plant('w2', 'weed', 0.5, 0.25, 0.01),
                  Plant('inaccessible', 'weed', 0.5, 0.45, 0.01)]
        result = simulate(self.config(sample_period=0.001), plants)
        self.assertEqual(result['summary']['crop_count'], 1)
        self.assertEqual(result['summary']['weed_count'], 3)
        self.assertEqual(result['summary']['weed_exposures'], 2)
        self.assertEqual(result['summary']['weed_exposure_fraction'], 2 / 3)
        self.assertEqual(result['summary']['crop_intersections'], 1)
        self.assertEqual(len({event['event_id'] for event in result['events']}),
                         len(result['events']))

    def test_signed_30_mm_lateral_bias_changes_which_crop_is_contacted(self):
        plants = [Plant('upper', 'crop', 0.5, 0.32, 0.01),
                  Plant('lower', 'crop', 0.5, 0.18, 0.01)]
        for error, expected in ((0.03, 'upper'), (-0.03, 'lower')):
            with self.subTest(error=error):
                result = simulate(self.config(robot_lateral_error=error), plants)
                self.assertEqual([event['plant_id'] for event in self.active_events(result)],
                                 [expected])

    def test_heading_uses_world_x_run_in_and_travel_aligned_geometry(self):
        angle = math.atan(0.1)
        plant = Plant('on_path', 'weed', 0.5, 0.32, 0.01)
        result = simulate(self.config(robot_heading_error=angle), [plant])
        event = self.active_events(result)[0]
        centre_time = 0.7 / math.cos(angle)
        self.assertAlmostEqual(event['entry_time'], centre_time - 0.05)
        self.assertAlmostEqual(event['exit_time'], centre_time + 0.05)
        self.assertAlmostEqual(result['timeline'][-1]['vehicle_x'], 1.2)

    def test_identical_seed_replays_full_result_and_input_order_is_irrelevant(self):
        config = self.config(seed=173, weed_density=40.0,
                             plant_spacing_variance=0.001)
        first = simulate(config)
        self.assertEqual(first, simulate(config))
        plants = generate_field(config)
        self.assertEqual(first, simulate(config, reversed(plants)))
        self.assertEqual(first['evidence_class'], 'SIMULATED')

    def test_generated_uniform_and_clustered_fields_keep_bounds_and_replay(self):
        for distribution in ('uniform', 'clustered'):
            with self.subTest(distribution=distribution):
                config = self.config(seed=12, weed_density=100.0,
                                     weed_distribution=distribution)
                plants = generate_field(config)
                self.assertEqual(plants, generate_field(config))
                self.assertTrue(any(plant.kind == 'weed' for plant in plants))
                self.assertTrue(all(0 <= plant.x <= config.row_length and
                                    0 <= plant.y <= config.field_width for plant in plants))

    def test_population_limit_fails_instead_of_returning_truncated_targets(self):
        with self.assertRaises(ValueError):
            generate_field(self.config(max_plants=1))
        with self.assertRaises(ValueError):
            generate_field(self.config(weed_density=1e6, max_plants=100))
        with self.assertRaises(ValueError):
            simulate(self.config(max_plants=1),
                     [Plant('a', 'weed', 0.5, 0.25, 0.01),
                      Plant('b', 'weed', 0.6, 0.25, 0.01)])

    def test_duplicate_ids_and_outside_fixture_fail(self):
        cases = ([Plant('same', 'weed', 0.5, 0.25, 0.01),
                  Plant('same', 'crop', 0.5, 0.25, 0.01)],
                 [Plant('outside_x', 'weed', 1.01, 0.25, 0.01)],
                 [Plant('outside_y', 'weed', 0.5, -0.01, 0.01)])
        for plants in cases:
            with self.subTest(plants=plants), self.assertRaises(ValueError):
                simulate(self.config(), plants)

    def test_invalid_configuration_values_fail(self):
        invalid = ({'vehicle_speed': 0}, {'vehicle_speed': -1},
                   {'vehicle_speed': True}, {'seed': True}, {'row_count': 1},
                   {'max_plants': 0}, {'weed_density': -1},
                   {'tool_response_time': -0.1}, {'tool_coast_down': -0.1},
                   {'sample_period': 0}, {'robot_heading_error': math.pi / 2},
                   {'crop_radius': float('nan')}, {'row_length': float('inf')},
                   {'tool_width': 0.1}, {'mechanism': 'unknown'},
                   {'weed_distribution': 'unknown'}, {'command_on': 2, 'command_off': 1},
                   {'plant_spacing_variance': -0.1}, {'typo_vehicle_speed': 1})
        for values in invalid:
            with self.subTest(values=values), self.assertRaises(ValueError):
                Config.from_mapping(values)

    def test_invalid_plant_values_fail(self):
        for args in (('', 'weed', 0, 0, 0.01), ('a', 'rock', 0, 0, 0.01),
                     ('a', 'weed', float('nan'), 0, 0.01),
                     ('a', 'crop', 0, 0, 0), ('a', 'crop', True, 0, 0.01)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                Plant(*args)

    def test_json_config_loads_without_optional_dependencies(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'fixture.json'
            path.write_text(json.dumps({'seed': 42, 'weed_density': 0}), encoding='utf-8')
            config = load_config(path)
        self.assertEqual(config.seed, 42)
        self.assertEqual(config.weed_density, 0)
        self.assertEqual(config, replace(Config(), seed=42, weed_density=0))


if __name__ == '__main__':
    unittest.main()
