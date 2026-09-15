"""Independent partition and provenance checks for missed-weed accounting."""

import json
from pathlib import Path
import tempfile
import unittest

from simulation.geometry.config import Config
from simulation.geometry.field import Plant
from simulation.geometry.misses import classify_misses, summarize_misses
from simulation.monte_carlo.operations import run_operations
from simulation.monte_carlo.miss_analysis import analyze


class MissTests(unittest.TestCase):
    def test_partition_preserves_every_target_and_category_precedence(self):
        plants=[Plant('crop','crop',1,0,.025),Plant('close','weed',1,.04,.01),
                Plant('gap','weed',1,.10,.01),Plant('deviation','weed',1,.25,.01),
                Plant('hit','weed',1,0,.01)]
        event={'plant_id':'hit','kind':'weed','envelope':'active'}
        records=classify_misses(Config(),plants,[event,event],[.25])
        self.assertEqual({r['id']:r['category'] for r in records},
                         {'close':'near_crop','gap':'nominal_gap','deviation':'path_error','hit':'exposed'})
        summary=summarize_misses(records)
        self.assertEqual(summary['target_count'],4)
        self.assertEqual(summary['miss_count'],3)
        self.assertEqual(sum(summary['counts'].values()),4)
        self.assertEqual(summary['exposed_near_crop_count'],1)

    def test_guard_contact_alone_never_exposes_weed(self):
        p=Plant('weed','weed',1,.25,.01)
        record=classify_misses(Config(),[p],[{'plant_id':p.id,'kind':'weed','envelope':'guard'}],[.25])[0]
        self.assertFalse(record['exposed']);self.assertEqual(record['category'],'path_error')
        self.assertIsNone(record['crop_surface_clearance'])

    def test_clearance_uses_crop_surface_and_threshold_is_inclusive(self):
        plants=[Plant('crop','crop',1,0,.015625),Plant('edge','weed',1,.0625,.015625)]
        record=classify_misses(Config(),plants,[],[.25],clearance_threshold=.03125)[0]
        self.assertEqual(record['crop_surface_clearance'],.03125)
        self.assertEqual(record['category'],'near_crop')

    def test_order_replay_and_nearest_crop_tie(self):
        plants=[Plant('b','crop',1,.5,.02),Plant('a','crop',1,0,.02),Plant('w','weed',1,.25,.01)]
        a=classify_misses(Config(),plants,[],[.25]);b=classify_misses(Config(),reversed(plants),[],[.25])
        self.assertEqual(a,b);self.assertEqual(a[0]['nearest_crop_id'],'a')

    def test_empty_targets_and_invalid_inputs(self):
        self.assertIsNone(summarize_misses([])['fractions']['exposed'])
        p=Plant('w','weed',1,.25,.01)
        for plants,events,offsets,threshold in [([p,p],[],[.25],.03),([p],[{'plant_id':'missing'}],[.25],.03),
                                                ([p],[],[],.03),([p],[],[.25],-1)]:
            with self.assertRaises(ValueError):classify_misses(Config(),plants,events,offsets,threshold)

    def test_verified_operation_replay_and_tamper_detection(self):
        plan={'fixture_count':2,'seed_start':3,'bootstrap_samples':100,'bootstrap_seed':0,
              'layouts':{'inset_four':[.12,.21,.29,.38]},'lateral_bounds':[.03],
              'error_modes':['independent'],'reposition_seconds':5,
              'guards':{'open_front_u':[{'name':'body','shape':'disk','radius':.06}]}}
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source=root/'source';output=root/'analysis'
            run_operations(Config(),plan,source)
            result=analyze(source,output,make_plots=False)
            self.assertEqual(sum(result['counts'].values()),result['target_count'])
            self.assertEqual(result,analyze(source,output,make_plots=False))
            (source/'fields.json').write_text('[]')
            with self.assertRaises(ValueError):analyze(source,output,make_plots=False)


if __name__=='__main__':unittest.main()
