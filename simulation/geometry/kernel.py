"""Exact intersections for constant-heading translation of conservative envelopes.

The hoe rectangle stays aligned with travel. Disks represent full rotational
sweeps; neither rotational phase nor soil/plant response is modeled.
"""

from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path

from .config import Config
from .field import generate_field


def _source_hash():
    """Fingerprint implementation bytes even before a Git commit exists."""
    digest = hashlib.sha256()
    for name in ('config.py', 'field.py', 'kernel.py'):
        digest.update(name.encode())
        digest.update(Path(__file__).with_name(name).read_bytes())
    return digest.hexdigest()


def _local(plant, config):
    """Plant position relative to starting tool pose in travel coordinates."""
    x = plant.x + config.run_in
    y = plant.y - config.path_offset - config.robot_lateral_error
    c, s = math.cos(config.robot_heading_error), math.sin(config.robot_heading_error)
    return x * c + y * s, -x * s + y * c


def _intersection(plant, config, start, end, guard=False):
    if start > end:
        return None
    x, y = _local(plant, config)
    if guard or config.mechanism != 'hoe':
        radius = (config.guard_radius if guard else config.tool_active_radius) + plant.radius
        if abs(y) > radius:
            return None
        reach = math.sqrt(max(0.0, radius * radius - y * y))
    else:
        # Moving rectangle Minkowski-summed with the plant disk: rounded
        # corners, not a bounding box that would overcount corner contacts.
        gap = max(0.0, abs(y) - config.tool_width / 2)
        if gap > plant.radius:
            return None
        reach = config.tool_length / 2 + math.sqrt(max(0.0, plant.radius**2 - gap**2))
    entry = max(start, (x - reach) / config.vehicle_speed)
    leave = min(end, (x + reach) / config.vehicle_speed)
    return (entry, leave) if entry <= leave else None


def _clearance(plant, config, timestamp):
    x, y = _local(plant, config)
    dx = x - config.vehicle_speed * timestamp
    if config.mechanism == 'hoe':
        return math.hypot(max(0.0, abs(dx) - config.tool_length / 2),
                          max(0.0, abs(y) - config.tool_width / 2)) - plant.radius
    return math.hypot(dx, y) - config.tool_active_radius - plant.radius


def simulate(config, plants=None, *, include_timeline=True):
    """Return reproducible geometry events and descriptive sample trajectory.

    Every supplied crop/weed belongs to the declared denominator, even when
    inaccessible. Explicit fixtures may overlap but must lie in the observation
    rectangle. Trajectory samples never determine contact counts.
    """
    if not isinstance(config, Config):
        raise TypeError('config must be a validated Config')
    # Revalidate direct construction as well as mapping construction.
    config = Config.from_mapping(asdict(config))
    plants = list(generate_field(config) if plants is None else plants)
    if len(plants) > config.max_plants:
        raise ValueError('field exceeds max_plants; targets were not truncated')
    duration = (config.row_length + 2 * config.run_in) / (
        config.vehicle_speed * math.cos(config.robot_heading_error))
    sample_count = math.ceil(duration / config.sample_period)
    # Bound diagnostic work as well as target count. This does not change event
    # geometry; callers can reduce sampling without changing contact results.
    if include_timeline and len(plants) * (sample_count + 5) > 10000000:
        raise ValueError('trajectory exceeds 10000000 clearance evaluations; increase sample_period')
    if len({p.id for p in plants}) != len(plants):
        raise ValueError('plant IDs must be unique')
    for p in plants:
        if not 0 <= p.x <= config.row_length or not 0 <= p.y <= (config.row_count - 1) * config.row_spacing:
            raise ValueError('plant centre outside observation rectangle')

    # run_in is longitudinal world-X distance before/after the observed row.
    off = duration if config.command_off is None else config.command_off
    onset = config.command_on + config.tool_response_time
    active_end = min(duration, off + config.tool_coast_down)
    active_start = max(0.0, onset)
    # A cancelled start cannot create a coast-down hazard from a tool that
    # never became active. Equality has no energized interval.
    can_activate = off > onset and active_start <= active_end

    events = []
    for plant in sorted(plants, key=lambda p: p.id):
        for envelope in ('active', 'guard'):
            if envelope == 'active' and not can_activate:
                continue
            interval = _intersection(plant, config,
                                     active_start if envelope == 'active' else 0.0,
                                     active_end if envelope == 'active' else duration,
                                     guard=envelope == 'guard')
            if interval is not None:
                events.append({'event_id': f'{plant.id}:{envelope}',
                               'plant_id': plant.id, 'kind': plant.kind,
                               'envelope': envelope, 'entry_time': interval[0],
                               'exit_time': interval[1]})
    events.sort(key=lambda e: (e['entry_time'], e['event_id']))
    crops = [p for p in plants if p.kind == 'crop']
    weeds = [p for p in plants if p.kind == 'weed']
    crop_hits = sum(e['kind'] == 'crop' and e['envelope'] == 'active' for e in events)
    weed_hits = sum(e['kind'] == 'weed' and e['envelope'] == 'active' for e in events)
    guard_hits = sum(e['kind'] == 'crop' and e['envelope'] == 'guard' for e in events)

    sample_count = math.ceil(duration / config.sample_period)
    if sample_count > 100000:
        raise ValueError('trajectory exceeds 100000 samples; increase sample_period')
    times = set()
    if include_timeline:
        times = {min(i * config.sample_period, duration) for i in range(sample_count + 1)}
        times.update(t for t in (config.command_on, off, onset, active_end) if 0 <= t <= duration)
    timeline = []
    c, s = math.cos(config.robot_heading_error), math.sin(config.robot_heading_error)
    for timestamp in sorted(times):
        x = -config.run_in + config.vehicle_speed * timestamp * c
        y = config.path_offset + config.robot_lateral_error + config.vehicle_speed * timestamp * s
        timeline.append({'timestamp': timestamp, 'vehicle_x': x, 'vehicle_y': y,
                         'tool_x': x, 'tool_y': y,
                         'commanded_active': config.command_on <= timestamp < off,
                         'actual_active': can_activate and active_start <= timestamp < active_end,
                         'min_crop_clearance': min((_clearance(p, config, timestamp) for p in crops), default=None),
                         'min_weed_clearance': min((_clearance(p, config, timestamp) for p in weeds), default=None)})
    config_dict = asdict(config)
    config_hash = hashlib.sha256(json.dumps(config_dict, sort_keys=True, allow_nan=False).encode()).hexdigest()
    plant_data = [asdict(p) for p in sorted(plants, key=lambda p: p.id)]
    field_hash = hashlib.sha256(json.dumps(plant_data, sort_keys=True, allow_nan=False).encode()).hexdigest()
    intervals = []
    for y in sorted({p.y for p in crops}):
        positions = sorted(p.x for p in crops if p.y == y)
        intervals.extend(b - a for a, b in zip(positions, positions[1:]))
    spacing_mean = sum(intervals) / len(intervals) if intervals else None
    spacing_variance = (sum((value - spacing_mean)**2 for value in intervals) / len(intervals)
                        if intervals else None)
    return {'schema_version': 1, 'evidence_class': 'SIMULATED',
            'config': config_dict, 'config_hash': config_hash, 'field_hash': field_hash,
            'source_hash': _source_hash(),
            'field_statistics': {'crop_interval_count': len(intervals),
                                 'crop_interval_mean_m': spacing_mean,
                                 'crop_interval_population_variance_m2': spacing_variance,
                                 'realized_weed_density_per_m2': len(weeds) / (config.row_length * config.field_width)},
            'assumptions': ['constant heading and speed', 'full conservative tool sweep',
                            'guard is a solid conservative disk, not crop protection',
                            'no plant biology, soil response or hardware safety simulation',
                            'all observation plants remain in denominators',
                            'tangent contact counts; switching endpoints conservatively included'],
            'summary': {'crop_count': len(crops), 'weed_count': len(weeds),
                        'crop_intersections': crop_hits, 'weed_exposures': weed_hits,
                        'guard_contacts': guard_hits,
                        'weed_exposure_fraction': weed_hits / len(weeds) if weeds else None,
                        'crop_intersection_fraction': crop_hits / len(crops) if crops else None,
                        'missed_weeds': len(weeds) - weed_hits,
                        'duration_s': duration, 'travel_speed_m_min': 60 * config.vehicle_speed,
                        'weed_removal_efficiency': None, 'crop_damage_rate': None,
                        'energy_wh_per_m': None, 'cost_inr_per_m': None},
            'plants': plant_data,
            'events': events, 'timeline': timeline}
