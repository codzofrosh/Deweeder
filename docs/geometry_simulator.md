# Deterministic geometry simulator

This implementation compares tool clearance and geometric weed exposure on a synthetic row. Results are **SIMULATED**. Tool overlap does not establish weed removal, suppression, crop injury, root damage or hardware safety. A [paired Monte Carlo pilot with plots and uncertainty](comparison_pilot.md) now builds on this kernel. CAD, biological response models and ROS/Webots integration remain future work.

Run from the repository root:

```console
python -m simulation.geometry --config configs/simulation_baseline.yaml --output simulation/experiments/baseline
```

Python 3.10+ is required. The geometry kernel and tests use the standard library. YAML loading uses PyYAML (already available in the audit environment); a `.json` configuration needs no third-party dependency. Configuration mappings may omit fields to use documented defaults in `simulation/geometry/config.py`. Unknown fields, duplicate keys, booleans in numeric fields, invalid dimensions and nonfinite inputs are rejected.

Run the new geometry suite independently of the historical ROS tests:

```console
python -m unittest discover -s tests -v
```

The tests also work under pytest when it is installed. The current local validation used unittest; it does not establish that the old ROS packages build or their safety tests pass.

Use the baseline as an invented engineering fixture, not an agronomic crop profile. Internal geometry uses metres and seconds, with x along travel and y across the row. Keep the same plants, path and target boundaries when comparing mechanism envelopes. A rotating line and rotating tine represented by the same disk must produce identical geometric exposure under identical timing; the simulator cannot distinguish their biological effects.

Inspect configuration validation errors before interpreting results. Check deterministic contact, tangent, missed-target, tool-off and delayed-action cases before making comparisons. Include weeds inside crop protection zones in the predefined target denominator; show their misses rather than dropping them. Compare crop-envelope intersections and guard contacts separately from target exposure. Refer to [the pivot architecture](pivot_architecture.md) for the intended experiment and evidence boundaries.

Each output directory contains:

| File | Content |
|---|---|
| `metadata.json` | Configuration, schema version, evidence class, summary and any other run metadata |
| `plants.csv` | Plant ID, kind, centre coordinates and radius |
| `events.csv` | Event ID, plant ID, kind, envelope and entry/exit times |
| `trajectory.csv` | Sample timestamp, vehicle/tool positions, commanded/actual activation and nearest crop/weed clearance |

CSV tables retain headers even when no records exist. Missing values are blank, not zero. Additional kernel fields are retained as alphabetically ordered extra columns. Metadata uses sorted JSON keys and rejects nonfinite numbers. Table order follows the deterministic kernel output. Exporting again to the same directory replaces these four files; use separate directories to retain comparisons.

Reproduce a run with the same configuration and source revision. Inspect event intervals as well as trajectory samples: a sampled trajectory alone can miss a brief collision. Report geometric exposure and crop-envelope intersection with their original denominators; empty populations have unavailable rates. No simulated rate in this workflow is WRE or CDR, and no deterministic fixture establishes a population collision probability.

## Model contract and limits

- Crops start at x=0 in each row; subsequent spacings use the configured normal mean/variance conditioned on positive intervals. Close or overlapping plants are retained. Metadata includes realized crop interval statistics. Weeds use a Poisson count over the entire observation rectangle, with uniform positions or a synthetic three-centre Gaussian mixture reflected at the boundaries. Neither distribution is calibrated to a farm.
- Rows span y=0 through `(row_count-1)*row_spacing`; at least two rows define the target area. Every crop and weed in that rectangle remains in the denominator, including targets inaccessible to the current pass. Explicit `Plant` fixtures may omit crops or overlap crops and weeds, but must have unique IDs and centres within the rectangle.
- The tool and vehicle share a position in this first adapter. Travel starts at `(-run_in, path_offset + robot_lateral_error)` and ends at x=`row_length+run_in`. Heading is constant in radians, lateral error is a fixed signed bias in metres, and speed is along the path. Correlated drift, longitudinal uncertainty and crop-sensing error are future model extensions; they are not implied by these fixed-bias inputs.
- Trimmer and tine use a full active disk with `tool_width=2*tool_active_radius`. Hoe uses a travel-aligned rectangle, with `tool_length` along travel and `tool_width` across travel; plant radius rounds the contact corners. Intersections use exact interval geometry, independent of `sample_period`. Rotational phase, blade frequency and dwell-to-cut efficacy are not modeled.
- The guard/body proxy is a solid disk of `guard_radius`, checked throughout travel even with the tool off. It is a conservative contact envelope, not a simulated crop-protection device. No collision changes the trajectory or automatically inhibits cutting. Height/depth are validated metadata only; stem projection contact cannot establish root clearance.
- Command times are relative to travel start. Actual activation begins at `command_on+tool_response_time`; it ends at `command_off+tool_coast_down`, clipped to travel end. A start cancelled before or at onset never activates. With `command_off: null`, command-off is at travel end. Motion after travel end and remaining coast-down outside the observation run are not simulated.
- Tangency counts as contact. Event intervals conservatively include switch boundaries, while sampled actuator states use half-open intervals; an instantaneous contact at shutoff can be counted even if the sampled state at that exact timestamp is off. Contact counts are deduplicated by plant and envelope, not timesteps.
- Trajectory clearance is computed to the potential active envelope even when inactive. For the hoe it is centre-to-rectangle distance minus plant radius, so negative values indicate overlap but are not a full penetration-depth calculation. `guard_contacts` counts crops touched by the guard; weed guard events remain in the event table.
- Default maximum population is 100,000; additional limits are 100,000 trajectory steps and 10 million sampled clearance evaluations. Exceeding a limit fails explicitly without returning a truncated field or biased result. Reduce diagnostic sampling frequency without changing exact contact counts.
- Metadata fingerprints configuration, actual field and kernel/config/generator source bytes. Numeric inputs are canonicalized so CSV replay preserves hashes. Keep the source files and Python environment with results; a hash identifies an implementation but does not archive it.

## First saved baseline and QA gate

The [saved baseline metadata](../simulation/experiments/baseline/metadata.json) records seed 0, 2.5 m observation length, 0.25 m/s travel, 80 mm disk width and a centred inter-row path. It contains 18 crops and 19 weeds: 4 geometric weed exposures, 15 misses, no crop-envelope intersections and no crop guard contacts. Travel including run-in/run-out is 11.6 s at 15 m/min. The 21.1% exposure fraction is for this single fixture and the full declared target area; it is not a population estimate or a removal result.

QA on 2026-09-14: **28/28 unittest tests PASS**, including analytic disk/hoe contacts, tangency, rounded-corner rejection, missing/overlapping populations, +/-30 mm fixed lateral errors, timing and coast-down, cancelled starts, guard contact with tool off, speed/sample invariance, high-density failure, field replay, CSV replay, empty exports and invalid configuration. The numeric canonicalization repair was verified by a previously failing CSV round-trip test.

Tech-lead decision: P1-1 configuration, field generation and deterministic event kernel are complete, with basic P1-2 exports. Proceed next to paired-seed trials, plots and row-level uncertainty. No mechanism selection or powered integration is justified by this baseline; the audit's ROS/HAL safety repairs remain outstanding.
