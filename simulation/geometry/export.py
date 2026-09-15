"""Deterministic, dependency-free exports for geometric simulation results."""

import csv
import io
import json
from pathlib import Path


PLANT_FIELDS = ("id", "kind", "x", "y", "radius")
EVENT_FIELDS = (
    "event_id", "plant_id", "kind", "envelope", "entry_time", "exit_time",
)
TRAJECTORY_FIELDS = (
    "timestamp", "vehicle_x", "vehicle_y", "tool_x", "tool_y",
    "commanded_active", "actual_active", "min_crop_clearance",
    "min_weed_clearance",
)


def _csv_text(rows, base_fields):
    """Keep source row order and append additional columns alphabetically."""
    extra_fields = sorted({key for row in rows for key in row} - set(base_fields))
    buffer = io.StringIO(newline="")
    writer = csv.DictWriter(
        buffer, fieldnames=[*base_fields, *extra_fields], lineterminator="\n",
    )
    writer.writeheader()
    for row in rows:
        writer.writerow({
            key: json.dumps(value, sort_keys=True, allow_nan=False)
            if isinstance(value, (dict, list)) else value
            for key, value in row.items()
        })
    return buffer.getvalue()


def export_result(result: dict, output_dir):
    """Write metadata and three CSVs; return their paths keyed by filename.

    Empty tables retain their standard headers. Additional fields are preserved.
    Missing values and ``None`` become empty CSV cells. Row order is retained so
    event and trajectory ordering remains the kernel's responsibility. The full
    result is checked for JSON compatibility and finite numbers before writing.
    Existing files of the same names in the selected directory are replaced.
    """
    json.dumps(result, sort_keys=True, allow_nan=False)
    metadata = {
        key: value for key, value in result.items()
        if key not in {"events", "timeline", "plants"}
    }
    contents = {
        "metadata.json": json.dumps(
            metadata, sort_keys=True, indent=2, allow_nan=False,
        ) + "\n",
        "plants.csv": _csv_text(result["plants"], PLANT_FIELDS),
        "events.csv": _csv_text(result["events"], EVENT_FIELDS),
        "trajectory.csv": _csv_text(result["timeline"], TRAJECTORY_FIELDS),
    }
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    paths = {}
    for filename, content in contents.items():
        path = directory / filename
        with path.open("w", encoding="utf-8", newline="") as output:
            output.write(content)
        paths[filename] = path
    return paths
