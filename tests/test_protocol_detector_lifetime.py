"""Compile the real detector with an offline receive fixture, never CAN."""
from pathlib import Path
import os
import shutil
import subprocess

import pytest

from workbench.runtime import ROOT, REPO


@pytest.mark.parametrize('original', [True, False], ids=['upstream_reproduces', 'patched_stops_receiver'])
def test_callback_cannot_outlive_protocol_detector(tmp_path, original):
    compiler=shutil.which('g++')
    if not compiler:
        pytest.skip('g++ required for native lifetime regression')
    sdk=ROOT/'src/external/ugv_sdk'
    if not sdk.is_dir():
        pytest.skip('locked ugv_sdk checkout required')
    fixture=Path(__file__).parent/'fixtures/protocol_detector'
    include=tmp_path/'include'
    fake=include/'ugv_sdk/details/async_port/async_can.hpp'
    fake.parent.mkdir(parents=True)
    shutil.copy2(fixture/'async_can.hpp',fake)
    source=sdk/'src/utilities/protocol_detector.cpp'
    if original:
        # Exact pinned upstream sources provide a negative control for the
        # delayed callback, without reverting or modifying the checkout.
        for name in ('include/ugv_sdk/utilities/protocol_detector.hpp',
                     'src/utilities/protocol_detector.cpp'):
            target=tmp_path/name
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes(subprocess.check_output(
                ['git','-C',str(sdk),'show','HEAD:'+name]))
        source=tmp_path/'src/utilities/protocol_detector.cpp'
    executable=tmp_path/'lifetime'
    subprocess.run([compiler,'-std=c++11','-O1','-g','-fno-omit-frame-pointer',
        '-fsanitize=address','-fsanitize-address-use-after-scope','-no-pie',
        '-I'+str(include),'-I'+str(sdk/'include'),'-I'+str(sdk/'src'),
        str(source),str(fixture/'lifetime.cpp'),'-pthread','-o',str(executable)],
        check=True,capture_output=True,text=True)
    result=subprocess.run([str(executable)],capture_output=True,text=True,
        env={**os.environ,'ASAN_OPTIONS':'detect_stack_use_after_return=1'},timeout=10)
    if original:
        assert result.returncode != 0
        assert 'stack-use-after-scope' in result.stderr, result.stderr
    else:
        assert result.returncode == 0, result.stderr


def test_protocol_patch_replays_on_locked_checkout_and_is_idempotent(tmp_path):
    import hashlib
    import json
    import sys
    sdk=ROOT/'src/external/ugv_sdk'
    if not sdk.is_dir():
        pytest.skip('locked ugv_sdk checkout required')
    target=tmp_path/'src/external/ugv_sdk'
    target.parent.mkdir(parents=True)
    subprocess.run(['git','clone','--quiet','--shared','--no-checkout',str(sdk),str(target)],check=True)
    subprocess.run(['git','-C',str(target),'checkout','--quiet','--detach','HEAD'],check=True)
    manifest=REPO/'patches/ugv_protocol_lifetime_manifest.json'
    command=[sys.executable,str(REPO/'scripts/apply_control_integration.py'),
             '--workspace',str(tmp_path),'--manifest',str(manifest)]
    original=(target/'src/utilities/protocol_detector.cpp').read_bytes()
    subprocess.run(command+['--check'],check=True,capture_output=True)
    assert (target/'src/utilities/protocol_detector.cpp').read_bytes()==original
    for _ in range(2):subprocess.run(command,check=True,capture_output=True)
    spec=json.loads(manifest.read_text())['external/ugv_sdk']
    assert subprocess.check_output(['git','-C',str(target),'rev-parse','HEAD'],text=True).strip()==spec['commit']
    for name,hashes in spec['files'].items():
        assert hashlib.sha256((target/name).read_bytes()).hexdigest()==hashes['after']
