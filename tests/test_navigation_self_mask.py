"""Validate exact pinned native-mask delivery and profile compatibility."""
import json
from pathlib import Path
import subprocess

import yaml

from agt_field_test_bringup import profiles

REPO = Path(__file__).resolve().parents[1]


def test_native_mask_patch_applies_to_locked_source_and_is_idempotent(tmp_path):
    manifest = REPO/'patches/navigation_perception_manifest.json'
    spec = json.loads(manifest.read_text())['agt_navigation_v3']
    destination = tmp_path/'src/agt_navigation_v3'
    destination.parent.mkdir()
    subprocess.run(['git','clone','-q','--shared','--no-checkout',str(REPO.parent/'agt_navigation_v3'),str(destination)],check=True)
    subprocess.run(['git','-C',str(destination),'checkout','-q','--detach',spec['commit']],check=True)
    command = ['python3',str(REPO/'scripts/apply_control_integration.py'),
               '--workspace',str(tmp_path),'--manifest',str(manifest)]
    subprocess.run(command,check=True,capture_output=True)
    subprocess.run(command,check=True,capture_output=True)


def test_profile_without_extra_geometry_does_not_emit_untyped_empty_ros_array(tmp_path):
    selected = yaml.safe_load((REPO/'templates/config/robots/bunker.yaml').read_text())
    selected.pop('navigation_self_mask_boxes')
    path = tmp_path/'profile.yaml';path.write_text(yaml.safe_dump(selected))
    profile = profiles.load(path)
    output = tmp_path/'nav.yaml'
    profiles.navigation(profile,REPO.parent/'agt_navigation_v3/config',output)
    config = yaml.safe_load(output.read_text())
    assert 'extra_boxes' not in config['agt_pointcloud_preprocessor']['ros__parameters']['self_filter']
