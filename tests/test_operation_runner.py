"""End-to-end operation evidence: pairing, event stream and repeatability."""

import copy
import csv
import gzip
import json
from pathlib import Path
import tempfile
import unittest

from simulation.geometry.config import Config
from simulation.monte_carlo.operations import run_operations, validate_plan


class OperationRunnerTests(unittest.TestCase):
    def plan(self):
        return {'fixture_count': 2, 'seed_start': 2000, 'bootstrap_samples': 100, 'bootstrap_seed': 2,
                'layouts': {'single':[.25], 'repeat':[.25,.25]}, 'lateral_bounds':[0,.03],
                'error_modes':['systematic','independent'], 'reposition_seconds':5,
                'guards':{'disk':[{'name':'body','shape':'disk','radius':.06}]}}

    def test_stream_replay_pairing_and_union(self):
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)
            aggregates,examples=run_operations(Config(),self.plan(),output)
            first={p.name:p.read_bytes() for p in output.iterdir()}
            run_operations(Config(),self.plan(),output)
            self.assertEqual(first,{p.name:p.read_bytes() for p in output.iterdir()})
            self.assertTrue(examples)
            with gzip.open(output/'events.csv.gz','rt',newline='') as stream:
                events=list(csv.DictReader(stream))
            self.assertTrue(events)
            self.assertTrue(all(e['evidence_class']=='SIMULATED' and e['event_id'] for e in events))
            with (output/'rows.csv').open(newline='', encoding='utf-8') as source:
                rows=list(csv.DictReader(source))
            for seed in {r['seed'] for r in rows}:
                paired=[r for r in rows if r['seed']==seed]
                self.assertEqual(len({r['field_hash'] for r in paired}),1)
                zero=[r for r in paired if float(r['lateral_bound'])==0]
                self.assertEqual(len({r['weed_exposures'] for r in zero}),1)
            meta=json.loads((output/'metadata.json').read_text())
            self.assertEqual(meta['independent_strip_count'],2)

    def test_examples_exist_for_nonzero_independent_only_plan(self):
        plan=self.plan(); plan['error_modes']=['independent']; plan['lateral_bounds']=[.01]
        with tempfile.TemporaryDirectory() as directory:
            _,examples=run_operations(Config(),plan,directory)
        self.assertEqual(len(examples),2)

    def test_bad_plans_rejected(self):
        changes=[{'fixture_count':True},{'lateral_bounds':[-.1]}, {'error_modes':['drift']},
                 {'layouts':{'bad':[]}}, {'reposition_seconds':float('inf')}, {'extra':0},
                 {'guards':{'empty':[]}}]
        for change in changes:
            plan=copy.deepcopy(self.plan());plan.update(change)
            with self.subTest(change=change),self.assertRaises(ValueError):validate_plan(plan)


if __name__=='__main__':unittest.main()
