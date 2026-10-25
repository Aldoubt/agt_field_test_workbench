"""Lossless pose-format adapter for the existing Polar Context native builder."""
import json
import math
from pathlib import Path
from .maps import hash_file


def prepare_builder_input(keyframes, destination):
    """Remove the timestamp column only; preserve poses, patch identities and order."""
    source = Path(keyframes).expanduser().resolve()
    poses = source/'poses_timed.txt'
    if not poses.is_file():
        poses = source/'poses.txt'
    patches = (source/'patches').resolve()
    if not poses.is_file() or not patches.is_dir():
        raise ValueError('Polar Context requires recorded keyframe poses and patches')
    before = hash_file(poses)
    rows, names = [], set()
    for number, line in enumerate(poses.read_text().splitlines(), 1):
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        fields = line.split()
        if len(fields) not in (8, 9):
            raise ValueError(f'invalid keyframe pose columns at line {number}')
        name = fields[0]
        patch = (patches/name).resolve()
        if name in names or not patch.is_relative_to(patches) or not patch.is_file():
            raise ValueError(f'duplicate, missing or invalid keyframe patch at line {number}: {name}')
        numbers = [float(value) for value in fields[1:]]
        if not all(math.isfinite(value) for value in numbers):
            raise ValueError(f'non-finite keyframe pose at line {number}')
        quaternion = [float(value) for value in fields[-4:]]
        if not .99 < math.sqrt(sum(value*value for value in quaternion)) < 1.01:
            raise ValueError(f'invalid keyframe quaternion at line {number}')
        rows.append(' '.join([name, *fields[-7:]])+'\n')
        names.add(name)
    if not rows or hash_file(poses) != before:
        raise ValueError('empty or changed keyframe poses')
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    (destination/'patches').symlink_to(patches, target_is_directory=True)
    (destination/'poses.txt').write_text(''.join(rows))
    provenance = dict(source_keyframes=str(source), source_poses=str(poses),
        source_poses_sha256=before, normalized_poses_sha256=hash_file(destination/'poses.txt'),
        entries=len(rows), conversion='name [stamp] x y z qw qx qy qz -> name x y z qw qx qy qz; numeric tokens unchanged')
    (destination/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    return provenance
