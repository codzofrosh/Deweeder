"""Independent operation-level fixtures for repeated passes and guard geometry."""

import copy
import math
import unittest

from simulation.geometry.config import Config
from simulation.geometry.field import Plant
from simulation.geometry.multipass import (
    Guard, GuardComponent, simulate_operation, standard_guards,
)


class MultipassTests(unittest.TestCase):
    def config(self, **changes):
        values = dict(row_length=1.0, run_in=0.2, vehicle_speed=1.0,
                      path_offset=0.25, weed_density=0.0, sample_period=0.1)
        values.update(changes)
        return Config.from_mapping(values)

    def disk_guard(self, radius=0.04, **placement):
        return Guard('test_disk', (GuardComponent('disk', shape='disk',
                                                 radius=radius, **placement),))

    def events(self, result, envelope):
        return [event for event in result['events'] if event['envelope'] == envelope]

    def test_repeated_overlapping_passes_count_plants_once_but_keep_events(self):
        plants = [Plant('crop', 'crop', 0.5, 0.25, 0.01),
                  Plant('weed', 'weed', 0.5, 0.25, 0.01)]
        guard = Guard('overlapping', (
            GuardComponent('one', shape='disk', radius=0.04),
            GuardComponent('two', shape='disk', radius=0.04),
        ))
        result = simulate_operation(self.config(), plants, [0.25] * 3, guard)
        summary = result['summary']
        self.assertEqual(summary['crop_count'], 1)
        self.assertEqual(summary['weed_count'], 1)
        self.assertEqual(summary['crop_intersections'], 1)
        self.assertEqual(summary['weed_exposures'], 1)
        self.assertEqual(summary['guard_contacts'], 1)
        self.assertEqual(summary['weed_exposure_fraction'], 1)
        active = self.events(result, 'active')
        guard_crop = [event for event in self.events(result, 'guard')
                      if event['kind'] == 'crop']
        self.assertEqual(len(active), 6)
        self.assertEqual(len(guard_crop), 6)
        self.assertEqual(len({(event['pass_index'], event['component'])
                              for event in guard_crop}), 6)

    def test_distinct_offsets_add_new_targets_without_changing_denominators(self):
        plants = [Plant('low', 'weed', 0.4, 0.15, 0.01),
                  Plant('high', 'weed', 0.6, 0.35, 0.01),
                  Plant('missed', 'weed', 0.5, 0.5, 0.01)]
        first = simulate_operation(self.config(), plants, [0.15], self.disk_guard())
        both = simulate_operation(self.config(), plants, [0.15, 0.35], self.disk_guard())
        self.assertEqual(first['summary']['weed_exposures'], 1)
        self.assertEqual(both['summary']['weed_exposures'], 2)
        self.assertEqual(first['summary']['weed_count'], 3)
        self.assertEqual(both['summary']['weed_count'], 3)
        self.assertEqual(both['summary']['weed_exposure_fraction'], 2 / 3)

    def test_shifted_disk_tangent_and_near_miss(self):
        guard = self.disk_guard(radius=0.03125, x=0.125, y=0.0625)
        plants = [Plant('tangent', 'crop', 0.5, 0.359375, 0.015625),
                  Plant('miss', 'crop', 0.5, 0.359376, 0.015625)]
        result = simulate_operation(self.config(), plants, [0.25], guard)
        contacts = self.events(result, 'guard')
        self.assertEqual([event['plant_id'] for event in contacts], ['tangent'])
        self.assertAlmostEqual(contacts[0]['entry_time'], 0.575)
        self.assertAlmostEqual(contacts[0]['exit_time'], 0.575)

    def test_component_offset_rotates_with_heading(self):
        angle = math.atan(0.25)
        c, s = math.cos(angle), math.sin(angle)
        local_x, local_y = 0.125, 0.0625
        travel = 0.5
        # Place a plant exactly at the translated and rotated component centre
        # after 0.5 s. Forgetting to rotate the component would miss this disk.
        x = -0.2 + (travel + local_x) * c - local_y * s
        y = 0.2 + (travel + local_x) * s + local_y * c
        plant = Plant('centre', 'crop', x, y, 0.005)
        result = simulate_operation(self.config(robot_heading_error=angle), [plant],
                                    [0.2], self.disk_guard(radius=0.005,
                                                          x=local_x, y=local_y))
        event = self.events(result, 'guard')[0]
        self.assertAlmostEqual(event['entry_time'], 0.49)
        self.assertAlmostEqual(event['exit_time'], 0.51)

    def test_rectangle_component_uses_rounded_corner_not_expanded_box(self):
        # At travel start, the tool is x=0. A component ahead of the tool is
        # already past this crop; its nearest initial corner misses by sqrt(2).
        guard = Guard('rectangle', (GuardComponent('plate', shape='rectangle',
                                                   x=0.02, width=0.08, length=0.02),))
        plant = Plant('corner_miss', 'crop', 0.002, 0.298, 0.01)
        result = simulate_operation(self.config(run_in=0.0), [plant], [0.25], guard)
        self.assertEqual(result['summary']['guard_contacts'], 0)
        hit = Plant('corner_hit', 'crop', 0.003, 0.296, 0.01)
        result = simulate_operation(self.config(run_in=0.0), [hit], [0.25], guard)
        self.assertEqual(result['summary']['guard_contacts'], 1)

    def test_reposition_cost_is_between_passes_only(self):
        result = simulate_operation(self.config(), [], [0.15, 0.25, 0.35],
                                    self.disk_guard(), reposition_seconds=5)
        summary = result['summary']
        self.assertAlmostEqual(summary['travel_seconds'], 3 * 1.4)
        self.assertAlmostEqual(summary['total_seconds'], 3 * 1.4 + 2 * 5)
        self.assertAlmostEqual(summary['total_distance_m'], 3 * 1.4)
        single = simulate_operation(self.config(), [], [0.25], self.disk_guard(),
                                    reposition_seconds=100)
        self.assertAlmostEqual(single['summary']['total_seconds'], 1.4)
        self.assertEqual(single['summary']['area_m2'], summary['area_m2'])

    def test_global_event_times_include_travel_and_reposition(self):
        plants = [Plant('weed', 'weed', 0.5, 0.25, 0.01)]
        result = simulate_operation(self.config(), plants, [0.25, 0.25],
                                    self.disk_guard(), reposition_seconds=5)
        events = self.events(result, 'active')
        self.assertEqual(len(events), 2)
        first, second = sorted(events, key=lambda event: event['global_entry_time'])
        self.assertAlmostEqual(first['entry_time'], second['entry_time'])
        self.assertAlmostEqual(second['global_entry_time'] - first['global_entry_time'], 6.4)
        self.assertAlmostEqual(second['global_exit_time'] - first['global_exit_time'], 6.4)

    def test_guards_do_not_implicitly_protect_crops_or_change_weed_exposure(self):
        plants = [Plant('crop', 'crop', 0.5, 0.25, 0.01),
                  Plant('weed', 'weed', 0.5, 0.25, 0.01)]
        guards = list(standard_guards().values()) + [self.disk_guard(radius=0.2)]
        results = [simulate_operation(self.config(), plants, [0.25], guard)
                   for guard in guards]
        for result in results:
            self.assertEqual(result['summary']['crop_intersections'], 1)
            self.assertEqual(result['summary']['weed_exposures'], 1)
        reference = [(event['plant_id'], event['entry_time'], event['exit_time'])
                     for event in self.events(results[0], 'active')]
        for result in results[1:]:
            self.assertEqual(reference,
                             [(event['plant_id'], event['entry_time'], event['exit_time'])
                              for event in self.events(result, 'active')])

    def test_off_tool_still_records_guard_contacts(self):
        result = simulate_operation(self.config(command_off=0.0),
                                    [Plant('crop', 'crop', 0.5, 0.25, 0.01)],
                                    [0.25], self.disk_guard())
        self.assertEqual(result['summary']['crop_intersections'], 0)
        self.assertEqual(result['summary']['guard_contacts'], 1)
        self.assertFalse(self.events(result, 'active'))

    def test_per_pass_biases_override_base_bias(self):
        plants = [Plant('upper', 'weed', 0.5, 0.32, 0.01),
                  Plant('lower', 'weed', 0.5, 0.18, 0.01)]
        result = simulate_operation(self.config(robot_lateral_error=0.2), plants,
                                    [0.25, 0.25], self.disk_guard(),
                                    lateral_biases=[0.03, -0.03])
        self.assertEqual(result['summary']['weed_exposures'], 2)
        self.assertEqual({event['plant_id'] for event in self.events(result, 'active')},
                         {'upper', 'lower'})

    def test_omitted_bias_uses_base_bias_for_each_pass(self):
        plant = Plant('upper', 'weed', 0.5, 0.32, 0.01)
        result = simulate_operation(self.config(robot_lateral_error=0.03), [plant],
                                    [0.25, 0.25], self.disk_guard())
        self.assertEqual(result['summary']['weed_exposures'], 1)
        self.assertEqual(len(self.events(result, 'active')), 2)

    def test_input_field_config_and_pass_lists_are_not_modified(self):
        base = self.config()
        plants = [Plant('b', 'weed', 0.6, 0.25, 0.01),
                  Plant('a', 'crop', 0.4, 0.25, 0.01)]
        offsets, biases = [0.2, 0.3], [0.01, -0.01]
        before = copy.deepcopy((base, plants, offsets, biases))
        first = simulate_operation(base, plants, offsets, self.disk_guard(), biases)
        second = simulate_operation(base, plants, offsets, self.disk_guard(), biases)
        self.assertEqual(first, second)
        self.assertEqual((base, plants, offsets, biases), before)

    def test_zero_denominators_remain_unavailable(self):
        summary = simulate_operation(self.config(), [], [0.25], self.disk_guard())['summary']
        self.assertEqual(summary['crop_count'], 0)
        self.assertEqual(summary['weed_count'], 0)
        self.assertEqual(summary['guard_contacts'], 0)
        self.assertIsNone(summary['crop_intersection_fraction'])
        self.assertIsNone(summary['weed_exposure_fraction'])
        for metric in ('weed_removal_efficiency', 'crop_damage_rate',
                       'energy_wh_per_m', 'cost_inr_per_m'):
            self.assertIsNone(summary[metric])

    def test_invalid_operation_inputs_are_rejected(self):
        bad = ({'offsets': []}, {'offsets': [0.25] * 21},
               {'offsets': [float('nan')]}, {'offsets': [True]},
               {'lateral_biases': [0, 0]}, {'lateral_biases': [float('inf')]},
               {'lateral_biases': [True]}, {'reposition_seconds': -1},
               {'reposition_seconds': float('nan')}, {'reposition_seconds': True})
        for override in bad:
            args = dict(base=self.config(), plants=[], offsets=[0.25], guard=self.disk_guard())
            args.update(override)
            with self.subTest(override=override), self.assertRaises((ValueError, TypeError)):
                simulate_operation(**args)

    def test_invalid_guard_shapes_dimensions_and_names_are_rejected(self):
        bad = ({'shape': 'unknown'}, {'shape': 'disk', 'radius': 0},
               {'shape': 'disk', 'radius': -1},
               {'shape': 'disk', 'radius': 0.1, 'width': 0.1},
               {'shape': 'rectangle', 'width': 0.1, 'length': 0},
               {'shape': 'rectangle', 'width': 0.1, 'length': 0.1, 'radius': 0.1},
               {'shape': 'disk', 'radius': float('nan')},
               {'shape': 'disk', 'radius': 0.1, 'x': True})
        for values in bad:
            with self.subTest(values=values), self.assertRaises((ValueError, TypeError)):
                GuardComponent('bad', **values)
        with self.assertRaises((ValueError, TypeError)):
            Guard('empty', ())
        with self.assertRaises((ValueError, TypeError)):
            Guard('duplicate_names', (GuardComponent('same', shape='disk', radius=0.1),
                                      GuardComponent('same', shape='disk', radius=0.1)))


if __name__ == '__main__':
    unittest.main()
