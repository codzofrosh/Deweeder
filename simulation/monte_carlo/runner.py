"""Paired comparisons; no biological response or powered robot dependencies."""

from dataclasses import asdict, dataclass, replace
import hashlib
import itertools
import importlib.metadata
import json
from pathlib import Path
import random
import sys

from simulation.geometry.config import Config, _finite_number, load_config
from simulation.geometry.export import _csv_text, export_result
from simulation.geometry.field import generate_field
from simulation.geometry.kernel import simulate
from .statistics import summarize, paired_difference


@dataclass(frozen=True)
class Comparison:
    fixture_count: int = 100
    seed_start: int = 1000
    bootstrap_samples: int = 500
    bootstrap_seed: int = 2026
    speeds: tuple = (0.1, 0.25, 0.5, 1.0)
    path_offsets: tuple = (0.25, 0.08)
    lateral_bounds: tuple = (0.0, 0.01, 0.03, 0.05)
    mechanisms: tuple = ('trimmer', 'tine', 'hoe')

    def __post_init__(self):
        for name in ('fixture_count', 'seed_start', 'bootstrap_samples', 'bootstrap_seed'):
            if type(getattr(self, name)) is not int or getattr(self, name) < 0:
                raise ValueError(f'{name} must be a nonnegative integer')
        if self.fixture_count < 2 or self.bootstrap_samples < 100:
            raise ValueError('need at least 2 fixtures and 100 bootstrap replicates')
        if self.fixture_count > 10000 or self.bootstrap_samples > 10000:
            raise ValueError('pilot limits: at most 10000 fixtures and 10000 bootstrap replicates')
        for name in ('speeds', 'path_offsets', 'lateral_bounds', 'mechanisms'):
            values = getattr(self, name)
            if not isinstance(values, (list, tuple)) or not values:
                raise ValueError(f'{name} must be a nonempty list')
            if name == 'mechanisms':
                if any(type(v) is not str or v not in ('trimmer', 'tine', 'hoe') for v in values):
                    raise ValueError('unsupported mechanism')
            elif any(not _finite_number(v) for v in values):
                raise ValueError(f'{name} must contain finite numbers, not booleans')
            if len(set(values)) != len(values):
                raise ValueError(f'{name} must not contain duplicates')
            object.__setattr__(self, name, tuple(values))
        if any(v <= 0 for v in self.speeds) or any(v < 0 for v in self.lateral_bounds):
            raise ValueError('speeds must be positive and lateral bounds nonnegative')
        cells = len(self.speeds) * len(self.path_offsets) * len(self.lateral_bounds)
        if cells * len(self.mechanisms) * self.fixture_count > 200000:
            raise ValueError('comparison exceeds 200000-run workload limit; split experiments')
        arms = len(self.mechanisms)
        if cells * (arms + arms * (arms - 1) // 2) * self.fixture_count * self.bootstrap_samples > 50000000:
            raise ValueError('comparison exceeds 50000000 bootstrap draws; split experiments')


def run_comparison(base, plan, progress=None):
    """Use each independently seeded strip once per cell and mechanism.

    The same uniform variate controls the signed lateral bias across all cells
    for a seed. It is constant within a pass; this is not a drift process.
    """
    if not isinstance(base, Config) or not isinstance(plan, Comparison):
        raise TypeError('expected Config and Comparison')
    fields = []
    total_plants = 0
    for seed in range(plan.seed_start, plan.seed_start + plan.fixture_count):
        plants = generate_field(replace(base, seed=seed))
        total_plants += len(plants)
        if total_plants > 2000000:
            raise ValueError('comparison exceeds 2000000 stored plants; split experiments')
        fields.append((seed, plants))
    rows, events, aggregates, differences, examples = [], [], [], [], []
    cells = list(itertools.product(plan.path_offsets, plan.speeds, plan.lateral_bounds))
    for cell_index, (offset, speed, bound) in enumerate(cells):
        cell_id = f'cell_{cell_index:03d}'
        arms = {}
        dimensions = {'cell_id': cell_id, 'path_offset': offset,
                      'vehicle_speed': speed, 'lateral_bound': bound}
        for mechanism in plan.mechanisms:
            arm = []
            for seed, plants in fields:
                variate = random.Random(f'path-bias:{seed}').uniform(-1.0, 1.0)
                bias = base.robot_lateral_error + bound * variate
                config = replace(base, seed=seed, mechanism=mechanism, path_offset=offset,
                                 vehicle_speed=speed, robot_lateral_error=bias)
                result = simulate(config, plants, include_timeline=False)
                row = {**dimensions, 'mechanism': mechanism, 'seed': seed,
                       'evidence_class': 'SIMULATED', 'lateral_bias': bias,
                       'field_hash': result['field_hash'], 'config_hash': result['config_hash'],
                       **result['summary']}
                rows.append(row)
                arm.append(row)
                events.extend({**dimensions, 'mechanism': mechanism, 'seed': seed,
                               'evidence_class': 'SIMULATED', **event} for event in result['events'])
                # Full timeline for a representative field per path/tool only.
                if seed == plan.seed_start and speed == plan.speeds[0] and bound == plan.lateral_bounds[0]:
                    example = simulate(config, plants)
                    example['scenario'] = f'path y={offset:g} m'
                    examples.append(example)
            arms[mechanism] = arm
            aggregates.append({**dimensions, 'mechanism': mechanism, 'evidence_class': 'SIMULATED',
                               **summarize(arm, plan.bootstrap_samples, plan.bootstrap_seed)})
        for a, b in itertools.combinations(plan.mechanisms, 2):
            differences.append({**dimensions, 'mechanism_a': a, 'mechanism_b': b,
                                'evidence_class': 'SIMULATED',
                                **paired_difference(arms[a], arms[b], plan.bootstrap_samples, plan.bootstrap_seed)})
        if progress:
            progress(cell_index + 1, len(cells))
    return {'schema_version': 1, 'evidence_class': 'SIMULATED',
            'base_config': asdict(base), 'plan': asdict(plan),
            'independent_fixture_count': plan.fixture_count,
            'simulation_count': len(rows), 'python_version': sys.version,
            'assumptions': ['independent unit is a seeded multi-row strip, not a plant',
                            'same field seeds reused across cells and mechanisms',
                            'signed uniform lateral bias shared across mechanisms, constant per pass',
                            'pointwise intervals; grid cells are correlated and not independent evidence',
                            'full active envelopes; no mechanical protection control or biology',
                            'bootstrap cannot bound unseen failure in an all-zero sample'],
            'rows': rows, 'events': events, 'aggregates': aggregates,
            'paired_differences': differences, 'examples': examples,
            'fields': [{'seed': seed, 'plants': [asdict(p) for p in plants]} for seed, plants in fields]}


def export_comparison(result, output_dir):
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    for table in ('rows', 'events', 'aggregates', 'paired_differences'):
        data = result[table]
        headers = tuple(data[0]) if data else ('cell_id', 'evidence_class')
        (directory / f'{table}.csv').write_text(_csv_text(data, headers), encoding='utf-8', newline='')
    fields_path = directory / 'fields.json'
    fields_path.write_text(json.dumps(result['fields'], sort_keys=True, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    metadata = {k: v for k, v in result.items()
                if k not in ('rows', 'events', 'aggregates', 'paired_differences', 'examples', 'fields')}
    metadata['package_versions'] = {}
    for package in ('matplotlib', 'numpy', 'PyYAML'):
        try:
            metadata['package_versions'][package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            metadata['package_versions'][package] = None
    root = Path(__file__).resolve().parents[1]
    metadata['source_files_sha256'] = {
        str(path.relative_to(root)).replace('\\', '/'): hashlib.sha256(path.read_bytes()).hexdigest()
        for folder in ('geometry', 'monte_carlo') for path in sorted((root / folder).glob('*.py'))}
    metadata['artifact_sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in sorted(directory.glob('*.csv'))}
    metadata['artifact_sha256']['fields.json'] = hashlib.sha256(fields_path.read_bytes()).hexdigest()
    (directory / 'metadata.json').write_text(json.dumps(metadata, sort_keys=True, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for index, example in enumerate(result['examples']):
        export_result(example, directory / 'examples' / f'{index:02d}_{example["config"]["mechanism"]}')
