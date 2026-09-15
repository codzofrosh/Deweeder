"""Run a reproducible geometry pass without ROS or Webots."""

import argparse
import json

from .config import load_config
from .export import export_result
from .kernel import simulate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    try:
        result = simulate(load_config(args.config))
        export_result(result, args.output)
    except (ValueError, TypeError, OSError) as exc:
        parser.exit(2, f'Geometry experiment failed: {exc}\n')
    print(json.dumps({'evidence_class': result['evidence_class'], **result['summary']}, indent=2))


if __name__ == '__main__':
    main()
