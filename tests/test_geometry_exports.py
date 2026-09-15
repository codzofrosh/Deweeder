"""File-level regression tests for reproducible experiment evidence."""

import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

from simulation.geometry.config import Config, load_config
from simulation.geometry.export import export_result
from simulation.geometry.field import Plant
from simulation.geometry.kernel import simulate


class ExportTests(unittest.TestCase):
    def test_round_trip_and_byte_replay(self):
        result = simulate(Config(), [Plant('weed,quoted', 'weed', 1, .25, .01)])
        with tempfile.TemporaryDirectory() as directory:
            paths = export_result(result, directory)
            first = {name: path.read_bytes() for name, path in paths.items()}
            export_result(result, directory)
            self.assertEqual(first, {name: path.read_bytes() for name, path in paths.items()})
            metadata = json.loads(paths['metadata.json'].read_text())
            self.assertEqual(metadata['evidence_class'], 'SIMULATED')
            self.assertNotIn('plants', metadata)
            with paths['plants.csv'].open(newline='') as source:
                plants = [Plant(row['id'], row['kind'], float(row['x']), float(row['y']),
                                float(row['radius'])) for row in csv.DictReader(source)]
            replay = simulate(Config.from_mapping(metadata['config']), plants)
            self.assertEqual(replay, result)

    def test_empty_csv_keeps_headers_and_json_nulls(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = export_result(simulate(Config(), []), directory)
            self.assertEqual(paths['plants.csv'].read_text().strip(), 'id,kind,x,y,radius')
            self.assertTrue(paths['events.csv'].read_text().startswith('event_id,plant_id'))
            data = json.loads(paths['metadata.json'].read_text())
            self.assertIsNone(data['summary']['weed_exposure_fraction'])

    def test_nonfinite_output_rejected_before_write(self):
        result = simulate(Config(), [])
        result['summary']['bad'] = float('nan')
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / 'invalid'
            with self.assertRaises(ValueError):
                export_result(result, target)
            self.assertFalse(target.exists())

    def test_duplicate_json_key_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'bad.json'
            path.write_text('{"vehicle_speed": 0.1, "vehicle_speed": 1}')
            with self.assertRaises(ValueError):
                load_config(path)

    @unittest.skipUnless(importlib.util.find_spec('yaml'), 'optional PyYAML unavailable')
    def test_duplicate_yaml_key_and_bad_yaml_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'bad.yaml'
            for text in ('seed: 1\nseed: 2\n', 'seed: [1'):
                path.write_text(text)
                with self.assertRaises(ValueError):
                    load_config(path)


if __name__ == '__main__':
    unittest.main()
