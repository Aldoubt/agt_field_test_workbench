#!/usr/bin/env python3
"""Read-only SHA, dirty-tree and duplicate ROS package audit."""
import json
import hashlib
import filecmp
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
INTEGRATION_PATCH_SHA = {
    'bringup/agt_mapping_bringup/agt_mapping_bringup/session_launch.py':
        'fd7a0625da99d1ee4f30dbc1db5dab98dc1afe7d24bab01756167463aa09e74c',
    'bringup/agt_mapping_bringup/agt_mapping_bringup/live_launch.py':
        '64240ebe9b23971f30dabb386688aa7a5a33b92b5d0dc945c6451cbd2d54492b',
}


def git(path, *args):
    return subprocess.run(['git', '-c', 'safe.directory='+str(path), '-C', str(path), *args], capture_output=True,
                          text=True, check=True).stdout.rstrip('\n')


def main():
    import yaml
    manifest = yaml.safe_load((ROOT / '.repos').read_text())['repositories']
    control_patches={}
    for name in ('control','frontend','map_cleanup','sc_kiss','relocalization_self_filter','navigation_perception','photo_mission','map_tracking_support','ugv_protocol_lifetime'):
        entries=json.loads((Path(__file__).resolve().parents[1]/f'patches/{name}_manifest.json').read_text())
        for relative,spec in entries.items():
            control_patches.setdefault(relative,dict(files={}))['files'].update(spec['files'])
    repos = []
    packages = {}
    for relative, spec in manifest.items():
        path = ROOT / 'src' / relative
        row = {'path': relative, 'expected': spec['version'], 'url': spec['url']}
        if (path / '.git').exists():
            changes = git(path, 'status', '--porcelain').splitlines()
            patched = (relative == 'agt-lio-pgo-mapping' and
                       {line[3:] for line in changes} == set(INTEGRATION_PATCH_SHA) and
                       all(hashlib.sha256((path / name).read_bytes()).hexdigest() == digest
                           for name, digest in INTEGRATION_PATCH_SHA.items()))
            patched = patched or (relative == 'external/kiss_matcher' and
                {line[3:] for line in changes} == {'COLCON_IGNORE'} and (path/'COLCON_IGNORE').read_bytes() == b'')
            control_spec = control_patches.get(relative, {})
            # Reviewed patches may add a header or regression test. Git's
            # default porcelain groups those as untracked directories.
            control_changes = (set(git(path, 'diff', '--name-only', 'HEAD').splitlines()) |
                               set(git(path, 'ls-files', '--others', '--exclude-standard').splitlines())) if control_spec else set()
            patched = patched or (bool(control_spec) and
                control_changes == set(control_spec['files']) and
                all(hashlib.sha256((path/name).read_bytes()).hexdigest() == hashes['after']
                    for name, hashes in control_spec['files'].items()))
            generated_livox = (relative == 'external/livox_ros_driver2' and
                               {line[3:] for line in changes} in ({'launch/'}, {'package.xml', 'launch/'}) and
                               filecmp.cmp(path / 'package.xml', path / 'package_ROS2.xml', shallow=False) and
                               sorted(p.name for p in (path / 'launch').iterdir()) ==
                               sorted(p.name for p in (path / 'launch_ROS2').iterdir()) and
                               all(filecmp.cmp(p, path / 'launch_ROS2' / p.name, shallow=False)
                                   for p in (path / 'launch').iterdir()))
            row.update(head=git(path, 'rev-parse', 'HEAD'),
                       branch=git(path, 'branch', '--show-current'),
                       dirty=bool(changes) and not patched and not generated_livox,
                       integration_patch='EXPECTED' if patched else None,
                       ros2_manifest='EXPECTED' if generated_livox else None)
        else:
            row['missing'] = True
        repos.append(row)
        if path.is_dir():
            for manifest_path in path.rglob('package.xml'):
                if '.git' in manifest_path.parts:
                    continue
                try:
                    name = ET.parse(manifest_path).getroot().findtext('name')
                except ET.ParseError:
                    name = None
                if name:
                    packages.setdefault(name.strip(), []).append(str(manifest_path.relative_to(ROOT)))
    duplicates = {name: paths for name, paths in packages.items() if len(paths) > 1}
    report = {'repositories': repos, 'package_count': len(packages),
              'duplicate_packages': duplicates,
              'mismatched_sha': [r['path'] for r in repos if r.get('head', r['expected']) != r['expected']],
              'dirty': [r['path'] for r in repos if r.get('dirty')]}
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 1 if duplicates or report['mismatched_sha'] or report['dirty'] else 0


if __name__ == '__main__':
    sys.exit(main())
