"""Analytic statistical contracts; no optional testing dependency required."""

import copy
import unittest

from simulation.monte_carlo.statistics import paired_difference, summarize


def fixture(crops=10, crop_hits=0, weeds=10, weed_hits=0, **metadata):
    return dict(crop_count=crops, crop_intersections=crop_hits,
                weed_count=weeds, weed_exposures=weed_hits,
                guard_contacts=0, **metadata)


class MonteCarloStatisticsTests(unittest.TestCase):
    def test_rates_are_ratios_of_pooled_counts_not_mean_fixture_rates(self):
        result = summarize([fixture(1, 1, 1, 1), fixture(9, 0, 9, 0)])
        self.assertEqual(result['crop_rate'], 0.1)
        self.assertEqual(result['weed_rate'], 0.1)
        self.assertEqual(result['fixture_count'], 2)
        self.assertEqual(result['row_any_crop_rate'], 0.5)
        self.assertEqual(result['ci_status'], {'crop': 'ok', 'weed': 'ok'})

    def test_wilson_zero_hits_has_positive_upper_bound(self):
        result = summarize([fixture() for _ in range(100)], bootstrap_samples=20)
        self.assertAlmostEqual(result['row_any_crop_ci_high'], 0.03699349820698568)
        self.assertAlmostEqual(result['row_any_crop_ci_low'], 0)
        self.assertEqual(result['row_any_crop_fixture_count'], 100)
        self.assertEqual(result['crop_ci_low'], 0)
        self.assertEqual(result['crop_ci_high'], 0)
        self.assertEqual(result['ci_status']['crop'], 'degenerate')
        self.assertTrue(any('does not establish zero population risk' in text
                            for text in result['limitations']))

    def test_wilson_all_hits_has_subunit_lower_bound(self):
        result = summarize([fixture(crop_hits=1) for _ in range(100)],
                           bootstrap_samples=20)
        self.assertAlmostEqual(result['row_any_crop_ci_low'], 0.9630065017930143)
        self.assertAlmostEqual(result['row_any_crop_ci_high'], 1)
        self.assertEqual(result['row_any_crop_rate'], 1)

    def test_wilson_counts_fixtures_not_correlated_plants(self):
        one_crop = summarize([fixture(crops=1) for _ in range(100)], bootstrap_samples=10)
        many_crops = summarize([fixture(crops=1000) for _ in range(100)], bootstrap_samples=10)
        self.assertEqual(one_crop['row_any_crop_ci_high'], many_crops['row_any_crop_ci_high'])

    def test_no_targets_have_unavailable_rates_and_intervals(self):
        result = summarize([fixture(0, 0, 0, 0), fixture(0, 0, 0, 0)], bootstrap_samples=50)
        for metric in ('crop_rate', 'weed_rate', 'crop_ci_low', 'crop_ci_high',
                       'weed_ci_low', 'weed_ci_high', 'row_any_crop_rate',
                       'row_any_crop_ci_low', 'row_any_crop_ci_high'):
            self.assertIsNone(result[metric])
        self.assertEqual(result['ci_status'], {'crop': 'unavailable', 'weed': 'unavailable'})
        self.assertEqual(result['bootstrap_excluded_samples'], {'crop': 50, 'weed': 50})

    def test_empty_input_has_no_invented_bootstrap_draws(self):
        result = summarize([], bootstrap_samples=50)
        self.assertEqual(result['fixture_count'], 0)
        self.assertEqual(result['bootstrap_valid_samples'], {'crop': 0, 'weed': 0})
        self.assertEqual(result['bootstrap_excluded_samples'], {'crop': 0, 'weed': 0})
        self.assertIsNone(result['crop_rate'])
        self.assertIsNone(result['weed_rate'])

    def test_single_informative_fixture_has_no_bootstrap_confidence_claim(self):
        result = summarize([fixture(1, 1, 1, 1), fixture(0, 0, 0, 0)],
                           bootstrap_samples=100, seed=0)
        self.assertEqual(result['crop_rate'], 1)
        self.assertEqual(result['ci_status']['crop'], 'single_fixture')
        self.assertIsNone(result['crop_ci_low'])
        self.assertIsNone(result['crop_ci_high'])
        self.assertEqual(result['row_any_crop_fixture_count'], 1)
        for metric in ('crop', 'weed'):
            excluded = result['bootstrap_excluded_samples'][metric]
            valid = result['bootstrap_valid_samples'][metric]
            self.assertGreater(excluded, 0)
            self.assertGreater(valid, 0)
            self.assertEqual(excluded + valid, 100)

    def test_same_seed_replays_and_does_not_modify_inputs(self):
        rows = [fixture(1, 1, 2, 0), fixture(9, 0, 8, 6), fixture(4, 2, 3, 2)]
        original = copy.deepcopy(rows)
        self.assertEqual(summarize(rows, seed=71), summarize(rows, seed=71))
        self.assertEqual(rows, original)

    def test_identical_paired_arms_have_exact_zero_delta_interval(self):
        rows = [fixture(1, 1, 2, 0, seed=0, field_hash='a'),
                fixture(9, 0, 8, 6, seed=1, field_hash='b'),
                fixture(4, 2, 3, 2, seed=2, field_hash='c')]
        result = paired_difference(rows, copy.deepcopy(rows), seed=61)
        for metric in ('crop_delta', 'weed_delta', 'crop_delta_ci_low',
                       'crop_delta_ci_high', 'weed_delta_ci_low', 'weed_delta_ci_high'):
            self.assertEqual(result[metric], 0)
        self.assertEqual(result['ci_status'], {'crop': 'degenerate', 'weed': 'degenerate'})

    def test_paired_difference_preserves_same_fixture_indices(self):
        # Baseline outcomes vary greatly, but every paired crop-rate difference
        # is exactly 0.1. Independent arm resampling would widen this interval.
        a = [fixture(10, 1, 10, 0), fixture(10, 9, 10, 7)]
        b = [fixture(10, 0, 10, 1), fixture(10, 8, 10, 8)]
        result = paired_difference(a, b)
        self.assertEqual(result['crop_delta'], 0.1)
        self.assertEqual(result['crop_delta_ci_low'], 0.1)
        self.assertEqual(result['crop_delta_ci_high'], 0.1)
        self.assertEqual(result['weed_delta'], -0.1)
        self.assertEqual(result['weed_delta_ci_low'], -0.1)
        self.assertEqual(result['weed_delta_ci_high'], -0.1)
        self.assertEqual(result['difference_direction'], 'a_minus_b')

    def test_paired_pooled_difference_and_reversal(self):
        a = [fixture(1, 1, 1, 0), fixture(9, 0, 9, 9)]
        b = [fixture(1, 0, 1, 1), fixture(9, 0, 9, 0)]
        forward = paired_difference(a, b, seed=8)
        backward = paired_difference(b, a, seed=8)
        self.assertEqual(forward['crop_delta'], 0.1)
        self.assertEqual(forward['weed_delta'], 0.8)
        self.assertEqual(forward['crop_delta'], -backward['crop_delta'])
        self.assertAlmostEqual(forward['crop_delta_ci_low'], -backward['crop_delta_ci_high'])
        self.assertAlmostEqual(forward['crop_delta_ci_high'], -backward['crop_delta_ci_low'])

    def test_mismatched_pairing_is_rejected(self):
        base = fixture(seed=1, field_hash='same')
        bad_arms = ([], [fixture(crops=11, seed=1, field_hash='same')],
                    [fixture(weeds=11, seed=1, field_hash='same')],
                    [fixture(seed=2, field_hash='same')],
                    [fixture(seed=1, field_hash='different')],
                    [fixture(field_hash='same')], [fixture(seed=1)])
        for bad in bad_arms:
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                paired_difference([base], bad)

    def test_invalid_counts_and_options_fail(self):
        for update in ({'crop_count': -1}, {'crop_count': True}, {'weed_count': 1.5},
                       {'crop_intersections': 11}, {'weed_exposures': 11},
                       {'guard_contacts': 11}, {'weed_exposures': float('nan')}):
            row = fixture()
            row.update(update)
            with self.subTest(update=update), self.assertRaises(ValueError):
                summarize([row])
        for options in ({'bootstrap_samples': 0}, {'bootstrap_samples': True},
                        {'bootstrap_samples': 2.5}, {'seed': -1}, {'seed': True}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                summarize([fixture()], **options)
        for rows in ({}, None, [None], [{}]):
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                summarize(rows)


if __name__ == '__main__':
    unittest.main()
