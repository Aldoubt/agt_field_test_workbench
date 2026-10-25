#!/usr/bin/env python3
"""Apply exact, reviewed dependency patches; preserve all other work."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

REPO = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', type=Path, default=REPO.parent.parent)
    parser.add_argument('--check', action='store_true', help='validate without writing')
    parser.add_argument('--manifest', type=Path, default=REPO/'patches/control_manifest.json')
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    experiment_path=REPO/'patches/sc_kiss_manifest.json'
    experiment=json.loads(experiment_path.read_text()) if experiment_path.is_file() else {}
    pending = []
    # Validate every dependency and file before making any changes.
    for relative, spec in manifest.items():
        root = args.workspace/'src'/relative
        head = subprocess.check_output(['git', '-c', 'safe.directory='+str(root), '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
        if head != spec['commit']:
            raise SystemExit(f'{relative}: unexpected HEAD {head}; preserved')
        names = []
        for name, hashes in spec['files'].items():
            actual = sha(root/name)
            if actual == hashes['before']:
                names.append(name)
            elif actual != hashes['after']:
                followup=experiment.get(relative,{}).get('files',{}).get(name,{})
                if not (args.manifest==REPO/'patches/control_manifest.json' and
                        followup.get('before')==hashes['after'] and followup.get('after')==actual):
                    raise SystemExit(f'{relative}/{name}: differs from reviewed source and patch; preserved')
        if names:
            command = ['git', '-C', str(root), 'apply', '--whitespace=error']
            for name in names:
                command += ['--include='+name]
            command += [str(REPO/spec['patch'])]
            subprocess.run([*command[:4], '--check', *command[4:]], check=True)
            pending.append(command)
    if not args.check:
        for command in pending:
            subprocess.run(command, check=True)
    print('Control integration '+('validated' if args.check else 'applied')+'; pinned HEADs preserved')


if __name__ == '__main__':
    main()
