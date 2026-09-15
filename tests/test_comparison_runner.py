"""Regression checks for paired geometry comparisons and replayable exports."""

import csv
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from simulation.geometry.config import Config
from simulation.geometry.field import Plant, generate_field
from simulation.geometry.kernel import simulate
from simulation.monte_carlo.runner import Comparison, export_comparison, run_comparison


COUNTS = (
    'crop_count', 'weed_count', 'crop_intersections', 'weed_exposures',
    'guard_contacts', 'missed_weeds', 'weed_exposure_fraction',
    'crop_intersection_fraction',
)


class ComparisonRunnerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base = Config(sample_period=.2)
        cls.plan = Comparison(
            fixture_count=3, bootstrap_samples=100, speeds=(.1, 1.0),
            path_offsets=(.08,), lateral_bounds=(.03,),
        )
        cls.result = run_comparison(cls.base, cls.plan)

    def test_pairing_preserves_fields_bias_and_denominators(self):
        self.assertEqual(self.result['simulation_count'], 18)
        self.assertEqual(self.result['independent_fixture_count'], 3)
        self.assertEqual(len(self.result['aggregates']), 6)
        self.assertEqual(len(self.result['paired_differences']), 6)
        self.assertEqual(len(self.result['fields']), 3)
        for fixture in self.result['fields']:
            with self.subTest(seed=fixture['seed']):
                rows = [row for row in self.result['rows'] if row['seed'] == fixture['seed']]
                self.assertEqual(len(rows), 6)
                ordered = sorted(fixture['plants'], key=lambda plant: plant['id'])
                expected_hash = hashlib.sha256(
                    json.dumps(ordered, sort_keys=True, allow_nan=False).encode()
                ).hexdigest()
                crops = sum(plant['kind'] == 'crop' for plant in ordered)
                weeds = sum(plant['kind'] == 'weed' for plant in ordered)
                self.assertEqual({row['field_hash'] for row in rows}, {expected_hash})
                self.assertEqual({row['crop_count'] for row in rows}, {crops})
                self.assertEqual({row['weed_count'] for row in rows}, {weeds})
                self.assertEqual(len({row['lateral_bias'] for row in rows}), 1)
                self.assertLessEqual(abs(rows[0]['lateral_bias']), .03)
                self.assertEqual({row['evidence_class'] for row in rows}, {'SIMULATED'})

    def test_always_active_counts_and_spatial_events_are_speed_invariant(self):
        for mechanism in self.plan.mechanisms:
            for seed in range(self.plan.seed_start, self.plan.seed_start + 3):
                with self.subTest(mechanism=mechanism, seed=seed):
                    rows = [row for row in self.result['rows']
                            if row['mechanism'] == mechanism and row['seed'] == seed]
                    self.assertEqual(len(rows), 2)
                    self.assertEqual({key: rows[0][key] for key in COUNTS},
                                     {key: rows[1][key] for key in COUNTS})
                    self.assertAlmostEqual(rows[0]['duration_s'] * rows[0]['vehicle_speed'],
                                           rows[1]['duration_s'] * rows[1]['vehicle_speed'])
                    arms = [[event for event in self.result['events']
                             if event['mechanism'] == mechanism and event['seed'] == seed
                             and event['vehicle_speed'] == speed] for speed in self.plan.speeds]
                    self.assertEqual(len(arms[0]), len(arms[1]))
                    for first, second in zip(*arms):
                        for key in ('event_id', 'plant_id', 'kind', 'envelope'):
                            self.assertEqual(first[key], second[key])
                        for key in ('entry_time', 'exit_time'):
                            self.assertAlmostEqual(first[key] * first['vehicle_speed'],
                                                   second[key] * second['vehicle_speed'])

    def test_identical_disk_envelopes_have_equal_events_and_zero_paired_delta(self):
        for speed in self.plan.speeds:
            for seed in range(self.plan.seed_start, self.plan.seed_start + 3):
                rows = {row['mechanism']: row for row in self.result['rows']
                        if row['vehicle_speed'] == speed and row['seed'] == seed}
                for key in COUNTS:
                    self.assertEqual(rows['trimmer'][key], rows['tine'][key])
                arms = [[{key: value for key, value in event.items() if key != 'mechanism'}
                         for event in self.result['events']
                         if event['mechanism'] == mechanism and event['seed'] == seed
                         and event['vehicle_speed'] == speed]
                        for mechanism in ('trimmer', 'tine')]
                self.assertEqual(*arms)
        disk_pairs = [row for row in self.result['paired_differences']
                      if (row['mechanism_a'], row['mechanism_b']) == ('trimmer', 'tine')]
        self.assertEqual(len(disk_pairs), 2)
        for row in disk_pairs:
            self.assertEqual(row['crop_delta'], 0)
            self.assertEqual(row['weed_delta'], 0)

    def test_omitting_timeline_does_not_change_events_or_summary(self):
        for mechanism in self.plan.mechanisms:
            with self.subTest(mechanism=mechanism):
                config = replace(self.base, mechanism=mechanism, path_offset=.08,
                                 robot_lateral_error=-.03, command_on=.2,
                                 command_off=2, tool_response_time=.1,
                                 tool_coast_down=.1)
                plants = generate_field(config)
                full = simulate(config, plants)
                compact = simulate(config, plants, include_timeline=False)
                self.assertTrue(full['timeline'])
                self.assertEqual(compact['timeline'], [])
                self.assertEqual(
                    {key: value for key, value in full.items() if key != 'timeline'},
                    {key: value for key, value in compact.items() if key != 'timeline'},
                )

    def test_comparison_replays_exactly(self):
        self.assertEqual(run_comparison(self.base, self.plan), self.result)

    def test_exports_have_verifiable_source_and_artifact_hashes_and_replay_fields(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            export_comparison(self.result, directory)
            metadata = json.loads((directory / 'metadata.json').read_text())
            self.assertEqual(metadata['evidence_class'], 'SIMULATED')
            self.assertEqual(metadata['simulation_count'], 18)
            source_root = Path(__file__).resolve().parents[1] / 'simulation'
            sources = metadata['source_files_sha256']
            self.assertIn('geometry/kernel.py', sources)
            self.assertIn('monte_carlo/runner.py', sources)
            expected_sources = {path.relative_to(source_root).as_posix()
                                for folder in ('geometry', 'monte_carlo')
                                for path in (source_root / folder).glob('*.py')}
            self.assertEqual(set(sources), expected_sources)
            for filename, digest in sources.items():
                self.assertEqual(hashlib.sha256((source_root / filename).read_bytes()).hexdigest(), digest)
            artifacts = metadata['artifact_sha256']
            self.assertEqual(set(artifacts), {
                'rows.csv', 'events.csv', 'aggregates.csv', 'paired_differences.csv', 'fields.json',
            })
            for filename, digest in artifacts.items():
                self.assertEqual(hashlib.sha256((directory / filename).read_bytes()).hexdigest(), digest)
            fixtures = json.loads((directory / 'fields.json').read_text())
            self.assertEqual(fixtures, self.result['fields'])
            by_seed = {fixture['seed']: [Plant(**plant) for plant in fixture['plants']]
                       for fixture in fixtures}
            for row in self.result['rows']:
                config = replace(self.base, seed=row['seed'], mechanism=row['mechanism'],
                                 path_offset=row['path_offset'], vehicle_speed=row['vehicle_speed'],
                                 robot_lateral_error=row['lateral_bias'])
                replay = simulate(config, by_seed[row['seed']], include_timeline=False)
                self.assertEqual(replay['field_hash'], row['field_hash'])
                self.assertEqual(replay['config_hash'], row['config_hash'])
                for key, value in replay['summary'].items():
                    self.assertEqual(value, row[key])
            with (directory / 'rows.csv').open(newline='') as source:
                rows = list(csv.DictReader(source))
            self.assertEqual(len(rows), 18)
            self.assertEqual({row['evidence_class'] for row in rows}, {'SIMULATED'})
            first = {path.relative_to(directory): path.read_bytes()
                     for path in directory.rglob('*') if path.is_file()}
            export_comparison(self.result, directory)
            self.assertEqual(first, {path.relative_to(directory): path.read_bytes()
                                    for path in directory.rglob('*') if path.is_file()})


class ComparisonValidationTests(unittest.TestCase):
    def test_invalid_plans_are_rejected(self):
        invalid = [
            {'fixture_count': 1}, {'fixture_count': True}, {'fixture_count': 3.0},
            {'fixture_count': 200001}, {'seed_start': -1}, {'seed_start': False},
            {'bootstrap_samples': 99}, {'bootstrap_samples': 100.0},
            {'bootstrap_seed': -1}, {'bootstrap_seed': True},
            {'speeds': ()}, {'speeds': (.1, .1)}, {'speeds': (0,)},
            {'speeds': (-.1,)}, {'speeds': (float('nan'),)},
            {'speeds': (float('inf'),)}, {'speeds': (True,)}, {'speeds': '.1'},
            {'path_offsets': []}, {'path_offsets': (.08, .08)},
            {'path_offsets': (float('-inf'),)}, {'path_offsets': (False,)},
            {'lateral_bounds': ()}, {'lateral_bounds': (-.01,)},
            {'lateral_bounds': (.03, .03)}, {'lateral_bounds': (float('nan'),)},
            {'mechanisms': ()}, {'mechanisms': ('trimmer', 'trimmer')},
            {'mechanisms': ('arm',)}, {'mechanisms': (True,)},
        ]
        for overrides in invalid:
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                Comparison(**overrides)

    def test_wrong_runner_argument_types_are_rejected(self):
        for base, plan in (({}, Comparison()), (Config(), {})):
            with self.subTest(base=type(base), plan=type(plan)), self.assertRaises(TypeError):
                run_comparison(base, plan)


if __name__ == '__main__':
    unittest.main()
