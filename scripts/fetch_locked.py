#!/usr/bin/env python3
"""Fetch only missing locked repositories; never reset or clean an existing tree."""
from pathlib import Path
import subprocess
import sys
import yaml

ROOT = Path(__file__).resolve().parents[3]


def run(*args, cwd=None):
    subprocess.run(args, cwd=cwd, check=True)


def main():
    for relative, spec in yaml.safe_load((ROOT / '.repos').read_text())['repositories'].items():
        path = ROOT / 'src' / relative
        if path.exists():
            actual = subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()
            if actual != spec['version']:
                raise SystemExit(f'{relative}: HEAD {actual} != locked {spec["version"]}; preserved')
            continue
        path.mkdir(parents=True)
        run('git', 'init', '-q', str(path))
        run('git', 'remote', 'add', 'origin', spec['url'], cwd=path)
        run('git', 'fetch', '-q', '--depth', '1', 'origin', spec['version'], cwd=path)
        run('git', 'checkout', '-q', '--detach', 'FETCH_HEAD', cwd=path)
        print(f'Fetched {relative} {spec["version"]}')


if __name__ == '__main__':
    sys.exit(main())
