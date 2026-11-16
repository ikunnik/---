#!/usr/bin/env python3
"""Capture/check migration input hashes; a snapshot is not an acceptance result."""
import argparse
import hashlib
import json
import os
import platform
import sys
import xml.etree.ElementTree as ET
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config import CFG


def input_files(root):
    paths = set(root.glob('*.py'))
    for directory in ('app', 'runtime', 'motion', 'decisions', 'workers', 'rover_x1', 'tools', 'launch'):
        paths.update((root / directory).rglob('*.py'))
    paths.update((root / 'tools').glob('*.sh'))
    paths.update((root / 'config').glob('*.yaml'))
    paths.update(p for p in (root / 'assets').rglob('*') if p.is_file())
    paths.update(p for p in (root / 'ros2').rglob('*')
                 if p.is_file() and (p.suffix in ('.msg', '.srv', '.action', '.xml')
                                    or p.name == 'CMakeLists.txt'))
    paths.update(root / name for name in ('package.xml', 'setup.cfg', 'requirements.txt'))
    return sorted(p for p in paths if p.is_file())


def fingerprint(root):
    hashes = {p.relative_to(root).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in input_files(root)}
    digest = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()
    return hashes, digest


def capture(root):
    hashes, digest = fingerprint(root)
    return {
        'schema_version': 1, 'interface_contract_version': 1,
        'acceptance': 'pending; consult validation report',
        'captured_utc': datetime.now(timezone.utc).isoformat(),
        'package_version': ET.parse(root / 'package.xml').getroot().findtext('version'),
        'environment': {'python': platform.python_version(), 'platform': platform.platform(),
                        'ros_distro': os.environ.get('ROS_DISTRO', 'not_sourced')},
        'config_defaults': asdict(CFG), 'inputs_sha256': digest, 'files_sha256': hashes,
    }


def differences(root, snapshot):
    current, _ = fingerprint(root)
    saved = snapshot['files_sha256']
    return sorted(name for name in set(saved) | set(current)
                  if saved.get(name) != current.get(name))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--output', type=Path, help='Capture a candidate baseline')
    mode.add_argument('--check', type=Path, help='Verify source/config/assets against snapshot')
    args = parser.parse_args()
    if args.check:
        snapshot = json.loads(args.check.read_text())
        if snapshot.get('schema_version') != 1:
            parser.exit(1, 'Unsupported baseline schema\n')
        changed = differences(ROOT, snapshot)
        if changed:
            parser.exit(1, 'Baseline drift:\n' + '\n'.join(changed) + '\n')
        print('Baseline source/config/assets unchanged (does not imply acceptance)')
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(capture(ROOT), ensure_ascii=False, indent=2) + '\n')
        print(args.output)


if __name__ == '__main__':
    main()
