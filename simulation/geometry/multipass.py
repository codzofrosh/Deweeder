"""Geometric union of controlled passes and explicit passive guard components."""

from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
from pathlib import Path

from .config import Config, _finite_number
from .field import Plant
from .kernel import _intersection, simulate


@dataclass(frozen=True)
class GuardComponent:
    """Tool-local offsets in m; rectangle length along X and width along Y."""

    name: str
    shape: str = "rectangle"
    x: float = 0.0
    y: float = 0.0
    width: float = 0.0
    length: float = 0.0
    radius: float = 0.0

    def __post_init__(self):
        if type(self.name) is not str or not self.name.strip():
            raise ValueError("guard component name must be nonempty")
        if self.shape not in ("rectangle", "disk"):
            raise ValueError("guard component shape must be rectangle or disk")
        for name in ("x", "y", "width", "length", "radius"):
            if not _finite_number(getattr(self, name)):
                raise ValueError(f"guard component {name} must be finite and not boolean")
        if self.shape == "rectangle":
            if self.width <= 0 or self.length <= 0 or self.radius != 0:
                raise ValueError("rectangle needs positive width/length and zero radius")
        elif self.radius <= 0 or self.width != 0 or self.length != 0:
            raise ValueError("disk needs positive radius and zero width/length")


@dataclass(frozen=True)
class Guard:
    name: str
    components: tuple[GuardComponent, ...]

    def __post_init__(self):
        if type(self.name) is not str or not self.name.strip():
            raise ValueError("guard name must be nonempty")
        if type(self.components) is not tuple or not self.components:
            raise ValueError("guard components must be a nonempty immutable tuple")
        if not all(isinstance(component, GuardComponent) for component in self.components):
            raise ValueError("guard components must be GuardComponent instances")
        if len({component.name for component in self.components}) != len(self.components):
            raise ValueError("guard component names must be unique")


def standard_guards() -> dict[str, Guard]:
    """Synthetic comparison fixtures, not validated protective hardware."""
    return {
        "legacy_disk": Guard("legacy_disk", (GuardComponent("body", "disk", radius=.06),)),
        "open_front_u": Guard("open_front_u", (
            GuardComponent("left_rail", x=0, y=.05, width=.008, length=.14),
            GuardComponent("right_rail", x=0, y=-.05, width=.008, length=.14),
            GuardComponent("rear_crossbar", x=-.074, y=0, width=.108, length=.008),
        )),
    }


def _hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def simulate_operation(base: Config, plants: list[Plant], offsets, guard: Guard,
                       lateral_biases=None, reposition_seconds=5) -> dict:
    """Deduplicate target exposure/contact across passes on one unchanged field.

    Every pass travels in the same positive-X direction. Manual repositioning
    occurs outside the crop area and is not geometrically modeled. Reposition
    time is charged only between passes. Guards never alter active-tool events.
    Local command times reset at each pass. Duplicate offsets are intentional
    repeat traversals, never new target denominators.
    """
    if not isinstance(base, Config) or not isinstance(guard, Guard):
        raise ValueError("base must be Config and guard must be Guard")
    if not _finite_number(reposition_seconds) or reposition_seconds < 0:
        raise ValueError("reposition_seconds must be finite and nonnegative")
    try:
        offsets = tuple(offsets)
    except TypeError as exc:
        raise ValueError("offsets must be a finite sequence") from exc
    if not 1 <= len(offsets) <= 20 or not all(_finite_number(x) for x in offsets):
        raise ValueError("offsets must contain 1 to 20 finite numbers")
    if lateral_biases is None:
        biases = (base.robot_lateral_error,) * len(offsets)
    else:
        try:
            biases = tuple(lateral_biases)
        except TypeError as exc:
            raise ValueError("lateral_biases must be a finite sequence") from exc
        if len(biases) != len(offsets) or not all(_finite_number(x) for x in biases):
            raise ValueError("lateral_biases must match offsets and contain finite numbers")
    plants = list(plants)
    if not all(isinstance(plant, Plant) for plant in plants):
        raise ValueError("plants must contain Plant instances")
    events, passes = [], []
    elapsed = 0.0
    travel_seconds = 0.0
    kernel_hash = None
    for pass_index, (offset, bias) in enumerate(zip(offsets, biases)):
        config = replace(base, path_offset=offset, robot_lateral_error=bias)
        result = simulate(config, plants, include_timeline=False)
        kernel_hash = result["source_hash"]
        duration = result["summary"]["duration_s"]

        def append_event(plant_id, kind, envelope, component, entry, leave):
            events.append({"event_id": json.dumps([pass_index, envelope, component, plant_id]),
                           "pass_index": pass_index, "component": component,
                           "plant_id": plant_id, "kind": kind, "envelope": envelope,
                           "entry_time": entry, "exit_time": leave,
                           "global_entry_time": elapsed + entry,
                           "global_exit_time": elapsed + leave})

        for event in result["events"]:
            if event["envelope"] == "active":
                append_event(event["plant_id"], event["kind"], "active", "tool",
                             event["entry_time"], event["exit_time"])
        c, s = math.cos(config.robot_heading_error), math.sin(config.robot_heading_error)
        for component in guard.components:
            # Translate the plant by the negative world-frame component offset.
            # _intersection then evaluates the component centred on the base path.
            dx = component.x*c - component.y*s
            dy = component.x*s + component.y*c
            if component.shape == "disk":
                component_config = replace(config, mechanism="trimmer",
                                           tool_active_radius=component.radius,
                                           tool_width=2*component.radius)
            else:
                component_config = replace(config, mechanism="hoe",
                                           tool_length=component.length, tool_width=component.width)
            for plant in plants:
                shifted = replace(plant, x=plant.x-dx, y=plant.y-dy)
                interval = _intersection(shifted, component_config, 0, duration, guard=False)
                if interval is not None:
                    append_event(plant.id, plant.kind, "guard", component.name, *interval)
        passes.append({"pass_index": pass_index, "config": asdict(config),
                       "duration_s": duration, "start_time": elapsed, "end_time": elapsed+duration})
        travel_seconds += duration
        elapsed += duration + (reposition_seconds if pass_index < len(offsets)-1 else 0)
        if not math.isfinite(elapsed):
            raise ValueError("operation duration must remain finite")

    events.sort(key=lambda event: (event["global_entry_time"], event["event_id"]))
    crops = {plant.id for plant in plants if plant.kind == "crop"}
    weeds = {plant.id for plant in plants if plant.kind == "weed"}
    crop_hits = {event["plant_id"] for event in events if event["envelope"] == "active" and event["kind"] == "crop"}
    weed_hits = {event["plant_id"] for event in events if event["envelope"] == "active" and event["kind"] == "weed"}
    guard_hits = {event["plant_id"] for event in events if event["envelope"] == "guard" and event["kind"] == "crop"}
    distance = travel_seconds * base.vehicle_speed
    area = base.row_length * base.field_width
    capacity = area / elapsed * 3600
    if not all(math.isfinite(value) for value in (distance, capacity)):
        raise ValueError("operation distance and capacity must remain finite")
    summary = {"crop_count": len(crops), "weed_count": len(weeds),
               "crop_intersections": len(crop_hits), "weed_exposures": len(weed_hits),
               "guard_contacts": len(guard_hits), "missed_weeds": len(weeds-weed_hits),
               "crop_intersection_fraction": len(crop_hits)/len(crops) if crops else None,
               "weed_exposure_fraction": len(weed_hits)/len(weeds) if weeds else None,
               "guard_contact_fraction": len(guard_hits)/len(crops) if crops else None,
               "miss_fraction": len(weeds-weed_hits)/len(weeds) if weeds else None,
               "total_distance_m": distance, "travel_seconds": travel_seconds,
               "total_seconds": elapsed, "duration_s": elapsed,
               "observation_length": base.row_length, "area_m2": area,
               "theoretical_area_m2_per_hour": capacity,
               "travel_speed_m_min": 60*base.vehicle_speed,
               "weed_removal_efficiency": None, "crop_damage_rate": None,
               "energy_wh_per_m": None, "cost_inr_per_m": None}
    plant_data = [asdict(plant) for plant in sorted(plants, key=lambda item: item.id)]
    operation_config = {"base": asdict(base), "guard": asdict(guard), "offsets": list(offsets),
                        "biases": list(biases), "reposition_seconds": reposition_seconds}
    source_hash = hashlib.sha256((kernel_hash + Path(__file__).read_text(encoding="utf-8")).encode()).hexdigest()
    return {"schema_version": 1, "evidence_class": "SIMULATED", "config": asdict(base),
            "guard": asdict(guard), "offsets": list(offsets), "biases": list(biases),
            "reposition_seconds": reposition_seconds, "passes": passes,
            "summary": summary, "events": events, "plants": plant_data,
            "field_hash": _hash(plant_data), "config_hash": _hash(operation_config),
            "source_hash": source_hash,
            "assumptions": ["constant positive-X pass orientation; no reversing",
                            "manual reposition outside crop area is unmodeled",
                            "guard contact is geometry, not protection or damage",
                            "union counts retain one original field denominator",
                            "area capacity credits observation area, not proven effective treatment",
                            "no plant biology, efficacy or hardware validation"]}
