"""Run a paired mechanism comparison from a JSON or YAML experiment plan."""

import argparse
import json
from pathlib import Path

from simulation.geometry.config import Config, load_config, load_mapping
from .runner import Comparison, run_comparison, export_comparison


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--no-plots', action='store_true')
    args = parser.parse_args()
    try:
        path = Path(args.config)
        data = load_mapping(path)
        if not isinstance(data, dict) or set(data) - {'base_config', 'base_overrides', 'comparison'}:
            raise ValueError('expected base_config, base_overrides and comparison mapping')
        base = load_config(path.parent / data['base_config'])
        base = Config.from_mapping({**base.__dict__, **data.get('base_overrides', {})})
        plan = Comparison(**data['comparison'])
        plotter = None
        if not args.no_plots:
            from .plots import plot_comparison
            plotter = plot_comparison
        result = run_comparison(base, plan, lambda done, total: print(f'Cells {done}/{total}', flush=True))
        export_comparison(result, args.output)
        if plotter:
            plotter(result['aggregates'], result['examples'], args.output)
        print(f'SIMULATED: {result["simulation_count"]} passes; {plan.fixture_count} independent strips')
    except (ValueError, TypeError, KeyError, OSError, ImportError) as exc:
        parser.exit(2, f'Comparison failed: {exc}\n')


if __name__ == '__main__':
    main()
