"""Explain unexposed targets without dropping crop-zone weeds from denominators."""

from dataclasses import replace
import math

from .config import Config, _finite_number
from .field import Plant
from .kernel import _intersection


def classify_misses(config, plants, events, nominal_offsets, clearance_threshold=.03):
    """Partition weeds: exposed, missed near crop, nominal gap, or path error.

    Near-crop distance is disk surface-to-surface clearance, not row distance.
    Threshold is a diagnostic bin, not validated safe clearance. ``path_error``
    means loss relative to ideal zero-bias/heading, continuously active passes;
    causal attribution requires the experiment's other conditions to match.
    """
    if not isinstance(config, Config):
        raise ValueError('config must be Config')
    if not _finite_number(clearance_threshold) or clearance_threshold < 0:
        raise ValueError('clearance threshold must be finite and nonnegative')
    plants = list(plants)
    if not all(isinstance(p, Plant) for p in plants) or len(plants) > config.max_plants:
        raise ValueError('invalid plant population')
    by_id = {p.id: p for p in plants}
    if len(by_id) != len(plants):
        raise ValueError('duplicate plant IDs')
    if any(not 0 <= p.x <= config.row_length or not 0 <= p.y <= config.field_width for p in plants):
        raise ValueError('plant outside observation rectangle')
    offsets = tuple(nominal_offsets)
    if not 1 <= len(offsets) <= 20 or any(not _finite_number(x) for x in offsets):
        raise ValueError('need 1..20 finite offsets')
    exposed = set()
    for event in events:
        if not isinstance(event, dict) or event.get('plant_id') not in by_id:
            raise ValueError('event references unknown plant')
        if event.get('kind') != by_id[event['plant_id']].kind or event.get('envelope') not in ('active', 'guard'):
            raise ValueError('event kind or envelope invalid')
        if event['envelope'] == 'active' and event['kind'] == 'weed':
            exposed.add(event['plant_id'])
    nominal = [replace(config, path_offset=x, robot_lateral_error=0., robot_heading_error=0.,
                       command_on=0., command_off=None, tool_response_time=0., tool_coast_down=0.) for x in offsets]
    duration = (config.row_length+2*config.run_in)/config.vehicle_speed
    crops = sorted((p for p in plants if p.kind == 'crop'), key=lambda p: p.id)
    records = []
    for weed in sorted((p for p in plants if p.kind == 'weed'), key=lambda p: p.id):
        nearest = min(((math.hypot(weed.x-crop.x, weed.y-crop.y)-weed.radius-crop.radius, crop.id)
                       for crop in crops), default=(None, None))
        near = nearest[0] is not None and nearest[0] <= clearance_threshold
        reachable = any(_intersection(weed, c, 0, duration) is not None for c in nominal)
        hit = weed.id in exposed
        category = 'exposed' if hit else 'near_crop' if near else 'path_error' if reachable else 'nominal_gap'
        records.append({'id':weed.id, 'x':weed.x, 'y':weed.y, 'radius':weed.radius,
                        'exposed':hit, 'near_crop':near, 'nominal_exposed':reachable,
                        'category':category, 'crop_surface_clearance':nearest[0],
                        'nearest_crop_id':nearest[1], 'evidence_class':'SIMULATED'})
    return records


def summarize_misses(records):
    records=list(records)
    categories=('exposed','near_crop','nominal_gap','path_error')
    if len({r['id'] for r in records}) != len(records) or any(r['category'] not in categories for r in records):
        raise ValueError('records must have unique IDs and known categories')
    counts={category:sum(r['category']==category for r in records) for category in categories}
    total=len(records)
    return {'target_count':total, 'counts':counts,
            'fractions':{k:v/total if total else None for k,v in counts.items()},
            'miss_count':total-counts['exposed'],
            'diagnostic_near_crop_count':sum(r['near_crop'] for r in records),
            'exposed_near_crop_count':sum(r['near_crop'] and r['exposed'] for r in records)}
