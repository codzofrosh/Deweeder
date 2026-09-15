"""Validated SI configuration for the synthetic geometry experiment."""

from dataclasses import dataclass, fields
import json
import math
from pathlib import Path
from collections.abc import Mapping


def _finite_number(value):
    if type(value) not in (float, int):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


@dataclass(frozen=True)
class Config:
    """Synthetic fixture, not an agronomic recommendation.

    Spacing variance is in m². Path errors are signed constant biases (m/rad).
    Tool depth is metadata: positive below soil; it predicts no biological effect.
    Command times are seconds from the start of run-in; None means travel end.
    """

    row_spacing: float = 0.5
    plant_spacing: float = 0.3
    plant_spacing_variance: float = 0.0
    crop_radius: float = 0.025
    crop_height: float = 0.15
    weed_density: float = 20.0
    weed_distribution: str = "uniform"
    weed_radius: float = 0.01
    robot_lateral_error: float = 0.0
    robot_heading_error: float = 0.0
    tool_width: float = 0.08
    tool_depth: float = 0.0
    tool_active_radius: float = 0.04
    tool_response_time: float = 0.0
    vehicle_speed: float = 0.25
    row_length: float = 2.5
    row_count: int = 2
    path_offset: float = 0.25
    guard_radius: float = 0.06
    tool_length: float = 0.02
    mechanism: str = "trimmer"
    seed: int = 0
    run_in: float = 0.2
    command_on: float = 0.0
    command_off: float | None = None
    tool_coast_down: float = 0.0
    sample_period: float = 0.05
    max_plants: int = 100000

    def __post_init__(self):
        integer_fields = {"seed", "row_count", "max_plants"}
        string_fields = {"mechanism", "weed_distribution"}
        for item in fields(self):
            value = getattr(self, item.name)
            if item.name in integer_fields:
                if type(value) is not int:
                    raise ValueError(f"{item.name} must be an integer, not a boolean")
            elif item.name in string_fields:
                if type(value) is not str:
                    raise ValueError(f"{item.name} must be a string")
            elif item.name == "command_off" and value is None:
                continue
            elif not _finite_number(value):
                raise ValueError(f"{item.name} must be a finite number, not a boolean")

        positive = ("row_spacing", "plant_spacing", "crop_radius", "crop_height",
                    "weed_radius", "tool_width", "tool_active_radius", "vehicle_speed",
                    "row_length", "tool_length", "sample_period")
        nonnegative = ("plant_spacing_variance", "weed_density", "tool_depth",
                       "tool_response_time", "guard_radius", "run_in", "command_on",
                       "tool_coast_down")
        for name in positive:
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        for name in nonnegative:
            if getattr(self, name) < 0:
                raise ValueError(f"{name} must be nonnegative")
        if self.row_count < 2:
            raise ValueError("row_count must be at least 2")
        if self.max_plants < 1 or self.seed < 0:
            raise ValueError("max_plants must be positive and seed nonnegative")
        if self.weed_distribution not in ("uniform", "clustered"):
            raise ValueError("weed_distribution must be uniform or clustered")
        if self.mechanism not in ("trimmer", "tine", "hoe"):
            raise ValueError("mechanism must be trimmer, tine or hoe")
        if abs(self.robot_heading_error) >= math.pi / 2:
            raise ValueError("robot_heading_error must lie strictly between -pi/2 and pi/2")
        if self.command_off is not None and self.command_off < self.command_on:
            raise ValueError("command_off must be no earlier than command_on")
        if self.mechanism in ("trimmer", "tine") and not math.isclose(
            self.tool_active_radius, self.tool_width / 2, rel_tol=1e-12, abs_tol=1e-15
        ):
            raise ValueError("disk tool_active_radius must equal tool_width / 2")
        # Derived products must also remain finite before generation or simulation.
        try:
            derived = (self.field_width, self.field_width * self.row_length,
                       self.field_width * self.row_length * self.weed_density,
                       (self.row_length + 2 * self.run_in)
                       / (self.vehicle_speed * math.cos(self.robot_heading_error)),
                       2 * self.field_width, 2 * self.row_length)
        except (OverflowError, ZeroDivisionError) as exc:
            raise ValueError("derived field geometry overflows") from exc
        if not all(math.isfinite(value) for value in derived):
            raise ValueError("derived field area, weed count and travel time must be finite")
        if self.field_width <= 0 or self.field_width * self.row_length <= 0 or derived[3] <= 0:
            raise ValueError("field area must remain positive at floating-point precision")
        duration = derived[3]
        timing = (self.command_on + self.tool_response_time,
                  (duration if self.command_off is None else self.command_off)
                  + self.tool_coast_down, duration / self.sample_period)
        if not all(math.isfinite(value) for value in timing):
            raise ValueError("derived actuator times and sample count must be finite")
        if math.ceil(duration / self.sample_period) > 100000:
            raise ValueError("sample_period would exceed the 100000-step workload limit")
        # Canonical numbers survive JSON/CSV replay with stable hashes.
        for item in fields(self):
            if item.name not in integer_fields | string_fields and getattr(self, item.name) is not None:
                object.__setattr__(self, item.name, float(getattr(self, item.name)))

    @property
    def field_width(self):
        return (self.row_count - 1) * self.row_spacing

    @classmethod
    def from_mapping(cls, mapping):
        if not isinstance(mapping, Mapping):
            raise ValueError("configuration must be a mapping")
        allowed = {item.name for item in fields(cls)}
        unknown = set(mapping) - allowed
        if unknown:
            raise ValueError(f"unknown configuration fields: {sorted(map(str, unknown))}")
        return cls(**mapping)


def load_mapping(path):
    """Read JSON or YAML; YAML requires PyYAML, with no automatic installation."""
    path = Path(path)
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        def unique_pairs(pairs):
            result = {}
            for key, value in pairs:
                if key in result:
                    raise ValueError(f"duplicate configuration key: {key}")
                result[key] = value
            return result
        mapping = json.loads(text, object_pairs_hook=unique_pairs)
    else:
        try:
            import yaml
        except ImportError as exc:
            raise ValueError("YAML configuration requires PyYAML; JSON uses only stdlib") from exc

        class UniqueKeyLoader(yaml.SafeLoader):
            pass

        def unique_mapping(loader, node):
            result = {}
            for key_node, value_node in node.value:
                key = loader.construct_object(key_node, deep=True)
                try:
                    if key in result:
                        raise ValueError(f"duplicate configuration key: {key}")
                    result[key] = loader.construct_object(value_node, deep=True)
                except TypeError as exc:
                    raise ValueError("configuration keys must be scalar field names") from exc
            return result

        UniqueKeyLoader.add_constructor(
            yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping
        )
        try:
            mapping = yaml.load(text, Loader=UniqueKeyLoader)
        except yaml.YAMLError as exc:
            raise ValueError(f"invalid YAML configuration: {exc}") from exc
    if not isinstance(mapping, Mapping):
        raise ValueError('configuration must be a mapping')
    return mapping


def load_config(path):
    """Load and validate a geometry configuration."""
    return Config.from_mapping(load_mapping(path))
