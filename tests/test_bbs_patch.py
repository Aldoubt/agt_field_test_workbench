"""Pinned patch ordering, idempotence and local-edit preservation."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess

import pytest

REPO = Path(__file__).resolve().parents[1]


def test_only_reviewed_followup_hashes_are_accepted():
    spec = importlib.util.spec_from_file_location('patch_applier', REPO / 'scripts/apply_control_integration.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    sc = json.loads((REPO / 'patches/sc_kiss_manifest.json').read_text())['agt_navigation_v3']
    bbs = json.loads((REPO / 'patches/bbs_recovery_manifest.json').read_text())['agt_navigation_v3']
    wrapper = 'navigation/localization/agt_global_relocalization/agt_global_relocalization/global_relocalization.py'
    original, final = sc['files'][wrapper]['after'], bbs['files'][wrapper]['after']
    args = (REPO / 'patches/sc_kiss_manifest.json', 'agt_navigation_v3', wrapper)
    assert module.reviewed_followup(*args, original, final, sc['commit'])
    assert not module.reviewed_followup(*args, original, 'unreviewed', sc['commit'])
    assert not module.reviewed_followup(*args, original, final, 'another commit')
    assert not module.reviewed_followup(Path('/tmp/sc_kiss_manifest.json'), *args[1:],
                                       original, final, sc['commit'])


def test_pinned_bbs_patch_composes_and_preserves_operator_edits(tmp_path):
    source = Path(os.environ.get('BBS_NAV_SOURCE', str(REPO.parent / 'agt_navigation_v3')))
    if not (source / '.git').exists():
        pytest.skip('set BBS_NAV_SOURCE to the locked agt_navigation_v3 checkout')
    destination = tmp_path / 'src/agt_navigation_v3'
    destination.parent.mkdir(parents=True)
    manifest = json.loads((REPO / 'patches/bbs_recovery_manifest.json').read_text())['agt_navigation_v3']
    subprocess.run(['git', 'clone', '-q', '--shared', '--no-checkout', str(source), str(destination)], check=True)
    subprocess.run(['git', '-C', str(destination), 'checkout', '-q', '--detach', manifest['commit']], check=True)
    subprocess.run(['git', '-C', str(destination), 'apply', str(REPO / 'patches/fwmini_guard.patch')], check=True)

    def apply(script, *extra):
        return subprocess.run(['python3', str(REPO / 'scripts' / script), '--workspace', str(tmp_path), *extra],
                              capture_output=True, text=True)

    assert apply('apply_sc_kiss_integration.py').returncode == 0
    assert apply('apply_bbs_recovery.py', '--check').returncode == 0
    assert apply('apply_bbs_recovery.py').returncode == 0
    assert apply('apply_bbs_recovery.py').returncode == 0
    assert apply('apply_bbs_recovery.py', '--check').returncode == 0
    assert apply('apply_sc_kiss_integration.py', '--check').returncode == 0
    assert apply('apply_sc_kiss_integration.py').returncode == 0
    for name, hashes in manifest['files'].items():
        assert hashlib.sha256((destination / name).read_bytes()).hexdigest() == hashes['after']
    assert subprocess.check_output(['git', '-C', str(destination), 'rev-parse', 'HEAD'], text=True).strip() == manifest['commit']
    wrapper = destination / next(name for name in manifest['files'] if name.endswith('/global_relocalization.py'))
    wrapper.write_text(wrapper.read_text() + '\n# existing operator edit\n')
    previous = wrapper.read_bytes()
    result = apply('apply_bbs_recovery.py')
    assert result.returncode != 0 and 'preserved' in result.stderr
    assert wrapper.read_bytes() == previous
