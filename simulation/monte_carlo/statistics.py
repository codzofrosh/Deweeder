"""Pooled geometry rates with independent-fixture uncertainty, using stdlib.

A fixture is an entire independently seeded field strip, not a plant. Percentile
bootstrap intervals are pointwise descriptive intervals. In particular, a
degenerate zero-event bootstrap does not establish zero population risk.
"""

from collections.abc import Mapping
import math
import random


_FIELDS = ('crop_count', 'crop_intersections', 'weed_count',
           'weed_exposures', 'guard_contacts')
_Z95 = 1.959963984540054


def _options(bootstrap_samples, seed):
    if type(bootstrap_samples) is not int or bootstrap_samples < 1:
        raise ValueError('bootstrap_samples must be a positive integer')
    if type(seed) is not int or seed < 0:
        raise ValueError('seed must be a nonnegative integer')


def _rows(rows):
    if isinstance(rows, (Mapping, str, bytes)):
        raise ValueError('rows must be a sequence of fixture mappings')
    try:
        rows = list(rows)
    except TypeError as exc:
        raise ValueError('rows must be iterable') from exc
    for index, row in enumerate(rows):
        if not isinstance(row, Mapping):
            raise ValueError(f'fixture {index} must be a mapping')
        for field in _FIELDS:
            value = row.get(field)
            if type(value) is not int or value < 0:
                raise ValueError(f'fixture {index}: {field} must be a nonnegative integer')
        for numerator, denominator in (('crop_intersections', 'crop_count'),
                                       ('weed_exposures', 'weed_count'),
                                       ('guard_contacts', 'crop_count')):
            if row[numerator] > row[denominator]:
                raise ValueError(f'fixture {index}: {numerator} exceeds {denominator}')
    return rows


def _ratio(numerator, denominator):
    return numerator / denominator if denominator else None


def _quantile(values, probability):
    """Linearly interpolated sample quantile, including one-value samples."""
    position = (len(values) - 1) * probability
    low = math.floor(position)
    high = math.ceil(position)
    return values[low] + (values[high] - values[low]) * (position - low)


def _interval(original, samples, fixture_count):
    if original is None:
        return None, None, 'unavailable'
    if fixture_count < 2:
        return None, None, 'single_fixture'
    if not samples:
        return None, None, 'unavailable'
    ordered = sorted(samples)
    status = 'degenerate' if ordered[0] == ordered[-1] else 'ok'
    return _quantile(ordered, 0.025), _quantile(ordered, 0.975), status


def _wilson(successes, trials):
    if not trials:
        return None, None
    proportion = successes / trials
    z_squared = _Z95**2
    denominator = 1 + z_squared / trials
    centre = (proportion + z_squared / (2 * trials)) / denominator
    half_width = _Z95 * math.sqrt(
        proportion * (1 - proportion) / trials + z_squared / (4 * trials**2)
    ) / denominator
    return max(0.0, centre - half_width), min(1.0, centre + half_width)


def _bootstrap(vectors, bootstrap_samples, seed):
    """Resample whole vectors once per draw; share indices across both metrics."""
    count = len(vectors)
    result = {'crop': [], 'weed': []}
    if not count:
        return result
    rng = random.Random(seed)
    for _ in range(bootstrap_samples):
        crop_numerator = crop_denominator = weed_numerator = weed_denominator = 0
        for _ in range(count):
            crop_n, crop_d, weed_n, weed_d = vectors[rng.randrange(count)]
            crop_numerator += crop_n
            crop_denominator += crop_d
            weed_numerator += weed_n
            weed_denominator += weed_d
        if crop_denominator:
            result['crop'].append(crop_numerator / crop_denominator)
        if weed_denominator:
            result['weed'].append(weed_numerator / weed_denominator)
    return result


def _uncertainty(result, crop, weed, draws, fixture_count, bootstrap_samples,
                 contributing, difference=False):
    suffix = '_delta' if difference else ''
    status = {}
    valid = {}
    excluded = {}
    limitations = [
        '95% intervals are pointwise, not simultaneous across a parameter sweep.',
        'Bootstrap units are independent whole fixtures, not individual plants.',
    ]
    for metric, original in (('crop', crop), ('weed', weed)):
        low, high, state = _interval(original, draws[metric], contributing[metric])
        result[f'{metric}{suffix}_ci_low'] = low
        result[f'{metric}{suffix}_ci_high'] = high
        status[metric] = state
        valid[metric] = len(draws[metric])
        # No resampling is possible for a collection with no fixtures.
        excluded[metric] = (bootstrap_samples - valid[metric]) if fixture_count else 0
        if excluded[metric]:
            limitations.append(
                f'{metric}: excluded {excluded[metric]} bootstrap draws with zero target denominator.'
            )
        if state == 'unavailable':
            limitations.append(f'{metric}: rate or interval unavailable; no target denominator or valid draws.')
        elif state == 'single_fixture':
            limitations.append(f'{metric}: one fixture cannot estimate between-fixture bootstrap uncertainty.')
        elif state == 'degenerate':
            limitations.append(
                f'{metric}: bootstrap distribution is degenerate; this does not establish zero population risk or uncertainty.'
            )
    result.update(ci_status=status, bootstrap_samples=bootstrap_samples,
                  contributing_fixture_count=contributing,
                  bootstrap_valid_samples=valid, bootstrap_excluded_samples=excluded,
                  limitations=limitations)


def summarize(rows, bootstrap_samples=500, seed=0):
    """Summarize pooled crop/weed rates and fixture-level uncertainty.

    ``row_any_crop_*`` uses crop-bearing fixtures only, with its denominator
    exposed as ``row_any_crop_fixture_count``. Empty-crop fixtures cannot supply
    evidence about crop safety. Undefined pooled denominators stay ``None``.
    """
    _options(bootstrap_samples, seed)
    rows = _rows(rows)
    vectors = [(row['crop_intersections'], row['crop_count'],
                row['weed_exposures'], row['weed_count']) for row in rows]
    crop = _ratio(sum(row['crop_intersections'] for row in rows),
                  sum(row['crop_count'] for row in rows))
    weed = _ratio(sum(row['weed_exposures'] for row in rows),
                  sum(row['weed_count'] for row in rows))
    eligible = [row for row in rows if row['crop_count']]
    any_hits = sum(row['crop_intersections'] > 0 for row in eligible)
    any_low, any_high = _wilson(any_hits, len(eligible))
    result = {'fixture_count': len(rows), 'crop_rate': crop, 'weed_rate': weed,
              'row_any_crop_fixture_count': len(eligible),
              'row_any_crop_rate': _ratio(any_hits, len(eligible)),
              'row_any_crop_ci_low': any_low, 'row_any_crop_ci_high': any_high,
              'crop_count': sum(row['crop_count'] for row in rows),
              'crop_intersections': sum(row['crop_intersections'] for row in rows),
              'weed_count': sum(row['weed_count'] for row in rows),
              'weed_exposures': sum(row['weed_exposures'] for row in rows),
              'guard_contacts': sum(row['guard_contacts'] for row in rows)}
    draws = _bootstrap(vectors, bootstrap_samples, seed)
    contributing = {'crop': len(eligible),
                    'weed': sum(row['weed_count'] > 0 for row in rows)}
    _uncertainty(result, crop, weed, draws, len(rows), bootstrap_samples, contributing)
    result['bootstrap_seed'] = seed
    if len(eligible) < len(rows):
        result['limitations'].append(
            'row_any_crop excludes fixtures with no crops; its denominator is row_any_crop_fixture_count.'
        )
    return result


def paired_difference(a, b, bootstrap_samples=500, seed=0):
    """Return pooled A-minus-B rate differences with paired-fixture resampling.

    Pairing is by list position. Target counts must agree per pair; when either
    arm supplies seed/field_hash metadata, both arms must supply equal values.
    """
    _options(bootstrap_samples, seed)
    a, b = _rows(a), _rows(b)
    if len(a) != len(b):
        raise ValueError('paired arms must have equal fixture counts')
    missing_metadata = False
    vectors = []
    for index, (left, right) in enumerate(zip(a, b)):
        for field in ('crop_count', 'weed_count'):
            if left[field] != right[field]:
                raise ValueError(f'pair {index}: target denominators differ for {field}')
        for field in ('seed', 'field_hash'):
            if field in left or field in right:
                if field not in left or field not in right or left[field] != right[field]:
                    raise ValueError(f'pair {index}: mismatched {field}')
            else:
                missing_metadata = True
        vectors.append((left['crop_intersections'] - right['crop_intersections'],
                        left['crop_count'],
                        left['weed_exposures'] - right['weed_exposures'],
                        left['weed_count']))
    crop = _ratio(sum(vector[0] for vector in vectors), sum(vector[1] for vector in vectors))
    weed = _ratio(sum(vector[2] for vector in vectors), sum(vector[3] for vector in vectors))
    result = {'fixture_count': len(a), 'difference_direction': 'a_minus_b',
              'crop_delta': crop, 'weed_delta': weed}
    draws = _bootstrap(vectors, bootstrap_samples, seed)
    contributing = {'crop': sum(row['crop_count'] > 0 for row in a),
                    'weed': sum(row['weed_count'] > 0 for row in a)}
    _uncertainty(result, crop, weed, draws, len(a), bootstrap_samples,
                 contributing, difference=True)
    result['bootstrap_seed'] = seed
    if missing_metadata:
        result['limitations'].append(
            'Some pairing metadata is absent; matching fixture identity relies on caller-provided row order.'
        )
    return result
