# Experiment artifacts

Git retains compact aggregate tables, metadata, PNG previews and the small deterministic baseline. Detailed event/row tables, saved field geometry, example trajectories and SVG exports are generated locally and ignored. They have not been deleted from the working directory.

To regenerate the detailed artifacts referenced by the reports:

```powershell
.\.venv-geometry\Scripts\python.exe -m simulation.geometry --config configs/simulation_baseline.yaml --output simulation/experiments/baseline
.\.venv-geometry\Scripts\python.exe -m simulation.monte_carlo --config configs/comparison_pilot.yaml --output simulation/experiments/comparison_pilot
.\.venv-geometry\Scripts\python.exe -m simulation.monte_carlo.operations --config configs/multipass_pilot.yaml --output simulation/experiments/multipass_pilot
```

Run from the repository root. Environment setup and pinned dependencies are documented in `docs/comparison_pilot.md`. Metadata hashes describe the source and generated data at the time of each run; historical source fingerprints are not automatically updated when code evolves. Missing detailed artifacts in a fresh checkout are expected until regeneration.
