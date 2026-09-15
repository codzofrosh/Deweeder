"""Seeded crop intervals and Poisson weed counts within a fixed target area."""

from dataclasses import dataclass
import math
import random

from .config import Config, _finite_number


@dataclass(frozen=True)
class Plant:
    id: str
    kind: str
    x: float
    y: float
    radius: float

    def __post_init__(self):
        if type(self.id) is not str or not self.id.strip():
            raise ValueError("plant id must be a nonempty string")
        if self.kind not in ("crop", "weed"):
            raise ValueError("plant kind must be crop or weed")
        for name in ("x", "y", "radius"):
            value = getattr(self, name)
            if not _finite_number(value):
                raise ValueError(f"plant {name} must be a finite number, not a boolean")
            object.__setattr__(self, name, float(value))
        if self.radius <= 0:
            raise ValueError("plant radius must be positive")


def generate_field(config: Config) -> list[Plant]:
    """Return repeatable crops and weeds, preserving close and overlapping plants.

    Crop rows include x=0 then normal spacing intervals conditioned on >0.
    Configured mean/variance describe the underlying normal, not the truncated
    realized intervals. Weeds use a Poisson count with expected density * area.
    Uniform positions cover the full rectangle. Clustered positions use three
    seeded centres and Gaussian offsets (sigma=row_spacing/5), reflected at
    rectangle boundaries. This is a fixed-count-mixture spatial cluster model,
    not a calibrated agronomic distribution. No target is silently discarded.
    """
    if not isinstance(config, Config):
        raise ValueError("generate_field requires a validated Config")
    rng = random.Random(config.seed)
    plants = []
    expected = config.weed_density * config.row_length * config.field_width
    # Explicit workload policy, not clipping a Poisson draw or its target count.
    if expected > config.max_plants:
        raise ValueError("expected weed count exceeds max_plants workload limit")

    def append(plant):
        if len(plants) >= config.max_plants:
            raise ValueError("generated field exceeds max_plants; no truncated field returned")
        plants.append(plant)

    if config.row_count > config.max_plants:
        raise ValueError("crop row count already exceeds max_plants")
    sigma = math.sqrt(config.plant_spacing_variance)
    for row in range(config.row_count):
        x = 0.0
        index = 0
        while x <= config.row_length:
            append(Plant(f"crop_{row}_{index}", "crop", x,
                         row * config.row_spacing, config.crop_radius))
            interval = config.plant_spacing
            if sigma:
                while True:
                    interval = rng.gauss(config.plant_spacing, sigma)
                    if interval > 0:
                        break
            next_x = x + interval
            if not math.isfinite(next_x) or next_x <= x:
                raise ValueError("crop spacing cannot advance position at floating-point precision")
            x = next_x
            index += 1

    # Unit-rate Poisson arrivals produce the count without a NumPy dependency.
    count = 0
    arrival = rng.expovariate(1.0)
    while arrival < expected:
        count += 1
        if len(plants) + count > config.max_plants:
            raise ValueError("generated field exceeds max_plants; no truncated field returned")
        arrival += rng.expovariate(1.0)

    centres = []
    if config.weed_distribution == "clustered" and count:
        centres = [(rng.uniform(0, config.row_length), rng.uniform(0, config.field_width))
                   for _ in range(3)]

    def reflect(value, extent):
        folded = value % (2 * extent)
        return folded if folded <= extent else 2 * extent - folded

    for index in range(count):
        if centres:
            cx, cy = rng.choice(centres)
            x = reflect(rng.gauss(cx, config.row_spacing / 5), config.row_length)
            y = reflect(rng.gauss(cy, config.row_spacing / 5), config.field_width)
        else:
            x = rng.uniform(0, config.row_length)
            y = rng.uniform(0, config.field_width)
        append(Plant(f"weed_{index}", "weed", x, y, config.weed_radius))
    return plants
