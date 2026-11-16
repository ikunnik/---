#!/usr/bin/env python3
"""Generate or verify ROS defaults without loading hardware or ROS."""
import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rover_x1.defaults import render_defaults


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Fail on generated file drift')
    parser.add_argument('--output', type=Path, default=ROOT / 'config/rover.yaml')
    args = parser.parse_args()
    expected = render_defaults()
    if args.check:
        if not args.output.is_file() or args.output.read_text() != expected:
            parser.exit(1, 'ROS defaults differ; run python3 tools/generate_ros_config.py\n')
        print('ROS defaults match config.py')
    else:
        args.output.write_text(expected)
        print(args.output)


if __name__ == '__main__':
    main()
