"""Offline ROS2 CLI. No rclpy executor, no live point-cloud subscription."""
import argparse
import hashlib
import json
import math
import shutil
from pathlib import Path

import numpy as np
import yaml

from .core import Settings, classify
from .pcd import read_xyz, write_xyz


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def keyframes(directory):
    root = Path(directory).resolve()
    pose_file = root / 'poses_timed.txt'
    if not pose_file.is_file() or not (root/'patches').is_dir():
        raise ValueError('requires poses_timed.txt and patches/ from verified frame-index export')
    previous = None
    for line_number, line in enumerate(pose_file.read_text().splitlines(), start=1):
        if not line.strip():
            continue
        words = line.split()
        if len(words) != 9:
            raise ValueError(f'invalid keyframe entry line {line_number}')
        name = words[0]
        if name != Path(name).name or not name.endswith('.pcd'):
            raise ValueError('unsafe keyframe filename')
        stamp, x, y, z, qw, qx, qy, qz = map(float, words[1:])
        if not all(math.isfinite(v) for v in (stamp,x,y,z,qw,qx,qy,qz)):
            raise ValueError('nonfinite keyframe timestamp or pose')
        if previous is not None and stamp <= previous:
            raise ValueError('nonmonotonic keyframe timestamps')
        previous = stamp
        return_file = root / 'patches' / name
        if not return_file.is_file():
            raise ValueError('missing keyframe patch: ' + name)
        yield (stamp, (x, y, z), (qx, qy, qz, qw), read_xyz(return_file))


def run(static_pcd, keyframe_dir, output, config, cleanup_manifest=None):
    original = Path(static_pcd).expanduser().resolve()
    frames_dir = Path(keyframe_dir).expanduser().resolve()
    dest = Path(output).expanduser().resolve()
    if not original.is_file():
        raise ValueError('existing static_map.pcd required')
    if dest.exists():
        raise ValueError('output already exists; never overwrite an old map')
    params = yaml.safe_load(Path(config).expanduser().read_text())
    if not isinstance(params, dict) or set(params) - set(Settings.__dataclass_fields__):
        raise ValueError('unknown clearance setting(s)')
    settings = Settings(**params)
    settings.validate()
    parent_meta = {}
    if cleanup_manifest:
        candidate = json.loads(Path(cleanup_manifest).read_text())
        if candidate.get('cleaned_sha256') != sha256(original):
            raise ValueError('static PCD differs from temporal cleanup manifest SHA')
        parent_meta = {
            'parent_cleanup_manifest': str(Path(cleanup_manifest).resolve()),
            'parent_cleanup_sha256': sha256(cleanup_manifest),
        }
    original_sha = sha256(original)
    static = read_xyz(original)
    keep, report = classify(static, keyframes(frames_dir), settings)
    # Stage in sibling path and atomically publish: a failed run may not look
    # like a complete candidate. No changes to existing map or BBS assets.
    tmp = dest.with_name(dest.name + '.building')
    if tmp.exists():
        raise ValueError('stale staging directory exists: ' + str(tmp))
    tmp.mkdir(parents=True)
    try:
        retained_path = tmp / 'static_candidate.pcd'
        removed_path = tmp / 'removed_candidates.pcd'
        write_xyz(retained_path, static[keep])
        write_xyz(removed_path, static[~keep])
        report.update({
            'source_pcd': str(original), 'source_sha256': original_sha,
            'keyframe_dir': str(frames_dir),
            'poses_sha256': sha256(frames_dir/'poses_timed.txt'),
            'config_sha256': sha256(config),
            'candidate_pcd_sha256': sha256(retained_path),
            'removed_pcd_sha256': sha256(removed_path),
            'qa_required': True,
            'map_update_allowed': False,
            'bbs_rebuild_required_if_accepted': True,
            **parent_meta,
        })
        (tmp / 'manifest.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        tmp.rename(dest)
    except BaseException:
        shutil.rmtree(tmp)
        raise
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--static-pcd', required=True)
    parser.add_argument('--keyframes', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--cleanup-manifest', default=None)
    args = parser.parse_args()
    result = run(args.static_pcd, args.keyframes, args.output, args.config, args.cleanup_manifest)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
