"""Compare multi-pass layouts and explicit body envelopes on paired fields."""

import argparse
import csv
from dataclasses import asdict, replace
import gzip
import hashlib
import io
import json
from pathlib import Path
import random
import sys

from simulation.geometry.config import Config, load_config, load_mapping, _finite_number
from simulation.geometry.export import _csv_text
from simulation.geometry.field import generate_field
from simulation.geometry.multipass import Guard, GuardComponent, simulate_operation
from .statistics import summarize


def validate_plan(plan):
    allowed = {'fixture_count', 'seed_start', 'bootstrap_samples', 'bootstrap_seed',
               'layouts', 'lateral_bounds', 'error_modes', 'guards', 'reposition_seconds'}
    if not isinstance(plan, dict) or set(plan) != allowed:
        raise ValueError(f'experiment must specify exactly {sorted(allowed)}')
    for key in ('fixture_count', 'seed_start', 'bootstrap_samples', 'bootstrap_seed'):
        if type(plan[key]) is not int or plan[key] < 0:
            raise ValueError(f'{key} must be a nonnegative integer')
    if not 2 <= plan['fixture_count'] <= 10000 or not 100 <= plan['bootstrap_samples'] <= 10000:
        raise ValueError('require 2..10000 fixtures and 100..10000 bootstrap resamples')
    if not _finite_number(plan['reposition_seconds']) or plan['reposition_seconds'] < 0:
        raise ValueError('reposition_seconds must be nonnegative and finite')
    for key in ('layouts', 'guards'):
        if not isinstance(plan[key], dict) or not plan[key]:
            raise ValueError(f'{key} must be a nonempty mapping')
        if any(type(k) is not str or not k or any(c not in 'abcdefghijklmnopqrstuvwxyz0123456789_' for c in k)
               for k in plan[key]):
            raise ValueError(f'{key} names must use lowercase letters, digits and underscores')
    for name, offsets in plan['layouts'].items():
        if not isinstance(offsets, (list, tuple)) or not 1 <= len(offsets) <= 20:
            raise ValueError(f'{name} must contain 1..20 offsets')
        if any(not _finite_number(x) for x in offsets):
            raise ValueError('offsets must be finite numbers')
    for key in ('lateral_bounds', 'error_modes'):
        if not isinstance(plan[key], (list, tuple)) or not plan[key]:
            raise ValueError(f'{key} must be a nonempty sequence')
    if any(not _finite_number(x) or x < 0 for x in plan['lateral_bounds']):
        raise ValueError('lateral_bounds must be nonnegative finite numbers')
    if any(x not in ('systematic', 'independent') for x in plan['error_modes']):
        raise ValueError('error_modes must be systematic or independent')
    if len(set(plan['error_modes'])) != len(plan['error_modes']) or len(set(plan['lateral_bounds'])) != len(plan['lateral_bounds']):
        raise ValueError('sweep values must be unique')
    guards = {}
    for name, definitions in plan['guards'].items():
        if not isinstance(definitions, list) or not 1 <= len(definitions) <= 20:
            raise ValueError('guards need 1..20 component mappings')
        try:
            guards[name] = Guard(name, tuple(GuardComponent(**component) for component in definitions))
        except TypeError as exc:
            raise ValueError('invalid guard component fields') from exc
    cells = len(guards) * len(plan['layouts']) * len(plan['lateral_bounds']) * len(plan['error_modes'])
    if cells * plan['fixture_count'] * max(map(len, plan['layouts'].values())) > 100000:
        raise ValueError('operation plan exceeds 100000-pass workload limit')
    if cells * plan['fixture_count'] * plan['bootstrap_samples'] * 2 > 50000000:
        raise ValueError('operation plan exceeds bootstrap workload limit')
    return guards


def run_operations(base, plan, output_dir, progress=None):
    guards = validate_plan(plan)
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    rows, aggregates, examples, fields = [], [], [], []
    plant_count = 0
    for seed in range(plan['seed_start'], plan['seed_start'] + plan['fixture_count']):
        plants = generate_field(replace(base, seed=seed))
        plant_count += len(plants)
        if plant_count > 2000000:
            raise ValueError('too many stored plants; split experiment')
        fields.append((seed, plants))
    event_columns = ('cell_id', 'seed', 'layout', 'guard', 'error_mode', 'lateral_bound',
                     'evidence_class', 'pass_index', 'component', 'plant_id', 'kind', 'envelope',
                     'event_id', 'entry_time', 'exit_time', 'global_entry_time', 'global_exit_time')
    cell_index = 0
    # Stream large event tables instead of retaining every pass event in memory.
    with (output / 'events.csv.gz').open('wb') as raw:
        with gzip.GzipFile(filename='', mode='wb', fileobj=raw, mtime=0) as zipped:
            with io.TextIOWrapper(zipped, encoding='utf-8', newline='') as text:
                writer = csv.DictWriter(text, fieldnames=event_columns, extrasaction='ignore', lineterminator='\n')
                writer.writeheader()
                for name, offsets in plan['layouts'].items():
                    for guard_name, guard in guards.items():
                        for mode in plan['error_modes']:
                            for bound in plan['lateral_bounds']:
                                dims = {'cell_id': f'operation_{cell_index:03d}', 'layout': name,
                                        'guard': guard_name, 'error_mode': mode, 'lateral_bound': bound,
                                        'pass_count': len(offsets), 'evidence_class': 'SIMULATED'}
                                group = []
                                for seed, plants in fields:
                                    bias = [base.robot_lateral_error + bound * random.Random(
                                        f'operation-bias:{seed}:{0 if mode == "systematic" else i}').uniform(-1, 1)
                                            for i in range(len(offsets))]
                                    result = simulate_operation(replace(base, seed=seed), plants, offsets, guard,
                                                                lateral_biases=bias,
                                                                reposition_seconds=plan['reposition_seconds'])
                                    row = {**dims, 'seed': seed, 'biases': bias,
                                           'field_hash': result['field_hash'], 'config_hash': result['config_hash'],
                                           **result['summary']}
                                    rows.append(row)
                                    group.append(row)
                                    for event in result['events']:
                                        writer.writerow({**dims, 'seed': seed, **event})
                                    if seed == plan['seed_start'] and mode == plan['error_modes'][0] and bound == plan['lateral_bounds'][0]:
                                        examples.append({**dims, **result})
                                stats = summarize(group, plan['bootstrap_samples'], plan['bootstrap_seed'])
                                guard_stats = summarize([{**row, 'crop_intersections': row['guard_contacts']}
                                                         for row in group], plan['bootstrap_samples'], plan['bootstrap_seed'])
                                aggregates.append({**dims, **stats,
                                                   'guard_rate': guard_stats['crop_rate'],
                                                   'guard_ci_low': guard_stats['crop_ci_low'],
                                                   'guard_ci_high': guard_stats['crop_ci_high'],
                                                   'guard_ci_status': guard_stats['ci_status']['crop'],
                                                   'total_seconds': group[0]['total_seconds'],
                                                   'total_distance_m': group[0]['total_distance_m']})
                                cell_index += 1
                                if progress:
                                    progress(cell_index)
    for name, table in (('rows', rows), ('aggregates', aggregates)):
        (output / f'{name}.csv').write_text(_csv_text(table, tuple(table[0])), encoding='utf-8', newline='')
    for name, data in (('fields', [{'seed': seed, 'plants': [asdict(p) for p in plants]} for seed, plants in fields]),
                       ('examples', examples)):
        (output / f'{name}.json').write_text(json.dumps(data, sort_keys=True, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    root = Path(__file__).resolve().parents[1]
    metadata = {'schema_version': 1, 'evidence_class': 'SIMULATED', 'base_config': asdict(base), 'plan': plan,
                'operation_count': len(rows), 'independent_strip_count': len(fields), 'python_version': sys.version,
                'pass_count': sum(row['pass_count'] for row in rows),
                'source_sha256': {str(p.relative_to(root)).replace('\\', '/'): hashlib.sha256(p.read_bytes()).hexdigest()
                                  for folder in ('geometry', 'monte_carlo') for p in sorted((root / folder).glob('*.py'))},
                'assumptions': ['union of exposed plant IDs across passes, not sum of pass counts',
                                'same fields and latent biases paired across guards and layouts',
                                'all passes forward; inactive repositioning outside field is assumed, not simulated',
                                'reposition time is estimated, not measured; no injury or suppression model',
                                'guard projection cannot deflect a crop or inhibit the active tool',
                                'error is constant per pass; systematic shared across passes, independent resampled per pass'],
                'artifact_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                    for p in sorted(output.iterdir()) if p.is_file() and p.name in
                                    ('rows.csv', 'aggregates.csv', 'events.csv.gz', 'fields.json', 'examples.json')}}
    (output / 'metadata.json').write_text(json.dumps(metadata, sort_keys=True, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    return aggregates, examples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--no-plots', action='store_true')
    args = parser.parse_args()
    try:
        path = Path(args.config)
        data = load_mapping(path)
        if set(data) != {'base_config', 'base_overrides', 'experiment'}:
            raise ValueError('expected base_config, base_overrides, experiment')
        base = load_config(path.parent / data['base_config'])
        base = Config.from_mapping({**asdict(base), **data['base_overrides']})
        aggregate, examples = run_operations(base, data['experiment'], args.output,
                                             lambda n: print(f'Operation cells completed: {n}', flush=True))
        if not args.no_plots:
            from .operation_plots import plot_operations
            plot_operations(aggregate, examples, args.output)
    except (ValueError, TypeError, KeyError, OSError, ImportError) as exc:
        parser.exit(2, f'Operation comparison failed: {exc}\n')


if __name__ == '__main__':
    main()
