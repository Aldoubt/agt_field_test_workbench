"""Use the same calibration implementation as installed ROS launch."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'agt_field_test_bringup'))
from agt_field_test_bringup.fwmini import (  # noqa: E402,F401
    load_profile, require_mode, verify_profile, digest, sha256, robot_description,
)


def candidate_path(root):
    local = Path(root) / 'calibration/fwmini/fwmini_mid360_candidate.yaml'
    return local if local.is_file() else Path(__file__).resolve().parents[1] / \
        'agt_field_test_bringup/config/fwmini_mid360_candidate.yaml'


def resolve_profile(root, path=None):
    target = Path(path).expanduser().resolve() if path else candidate_path(root).resolve()
    allowed = (Path(root)/'calibration/fwmini').resolve()
    if target != candidate_path(root).resolve() and not target.is_relative_to(allowed):
        raise ValueError('mount profile must be in calibration/fwmini/')
    load_profile(target)
    return target


def profiles(root):
    paths = {candidate_path(root).resolve(), *(Path(root)/'calibration/fwmini').glob('*.yaml')}
    records = []
    for path in sorted(paths):
        # Legacy *.fastlio.yaml are internal calibrations, not chassis mounts.
        if '.fastlio.' in path.name:
            continue
        record = load_profile(path)
        records.append(dict(record, path=str(path), file_sha256=sha256(path), calibration_sha256=digest(record)))
    return records
