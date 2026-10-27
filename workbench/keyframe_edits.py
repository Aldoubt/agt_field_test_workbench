"""Apply map-frame ROI edits to immutable body-frame keyframe records.

The fused map and its descriptors must see the same geometric deletion rules.
Only XYZ is transformed for selection: retained PCD records, point order and
pose/time tokens stay in their original representation.  This is an offline
export operation; it never alters the measurements supplied to LIO.
"""
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import tempfile

import numpy as np


class UnsupportedPCDEncoding(ValueError):
    """A keyframe cannot be safely edited by the standalone PCD reader."""

    state = 'UNSUPPORTED'


def _sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _operations(items):
    if not isinstance(items, (list, tuple)):
        raise ValueError('keyframe operations must be a list')
    applied, sampling = [], []
    for item in items:
        if not isinstance(item, dict):
            raise ValueError('keyframe operation must be an object')
        kind = item.get('type')
        if kind in ('crop', 'delete'):
            limits = []
            for key in ('min', 'max'):
                values = item.get(key)
                if (not isinstance(values, (list, tuple)) or len(values) != 3
                        or any(isinstance(v, bool) for v in values)):
                    raise ValueError('ROI requires three numeric bounds')
                try:
                    values = [float(v) for v in values]
                except (TypeError, ValueError) as exc:
                    raise ValueError('ROI requires three numeric bounds') from exc
                if not all(math.isfinite(v) for v in values):
                    raise ValueError('ROI bounds must be finite')
                limits.append(values)
            if any(lo >= hi for lo, hi in zip(*limits)):
                raise ValueError('ROI minimum must be below maximum')
            applied.append(dict(type=kind, min=limits[0], max=limits[1]))
        elif kind == 'voxel':
            try:
                size = float(item['size'])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError('voxel size must be positive and finite') from exc
            if isinstance(item['size'], bool) or not math.isfinite(size) or size <= 0:
                raise ValueError('voxel size must be positive and finite')
            sampling.append(dict(type='voxel', size=size))
        else:
            raise ValueError('unsupported keyframe operation: ' + str(kind))
    # Map voxel centroids can move across a subsequent ROI boundary. Replaying
    # that sequence on raw body records would claim a different deletion rule.
    seen_sampling = False
    for item in items:
        seen_sampling |= item.get('type') == 'voxel'
        if seen_sampling and item.get('type') in ('crop', 'delete'):
            raise ValueError('apply ROI edits before voxel sampling; voxel then ROI is ambiguous for keyframes')
    return applied, sampling


def _rotation(qw, qx, qy, qz):
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or not .99 < norm < 1.01:
        raise ValueError('invalid keyframe quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


@dataclass
class _PCD:
    header: list
    mode: str
    count: int
    record_size: int
    payload: object
    xyz: np.ndarray
    sha256: str

    def write_selected(self, path, keep):
        count = int(np.count_nonzero(keep))
        replacements = {'WIDTH': str(count), 'HEIGHT': '1', 'POINTS': str(count)}
        with Path(path).open('wb') as stream:
            for raw in self.header:
                token = raw.strip().split(None, 1)
                key = token[0].decode('ascii').upper() if token else ''
                stream.write((key + ' ' + replacements[key] + '\n').encode('ascii')
                             if key in replacements else raw)
            if self.mode == 'binary':
                records = np.frombuffer(self.payload, dtype=np.uint8).reshape(self.count, self.record_size)
                stream.write(records[keep].tobytes())
            else:
                for raw, selected in zip(self.payload, keep):
                    if selected:
                        stream.write(raw)


def _read_pcd(path):
    """Read standard ASCII/binary PCD without coercing non-XYZ fields."""
    raw = Path(path).read_bytes()
    header, values, offset = [], {}, 0
    while True:
        end = raw.find(b'\n', offset)
        if end < 0 or end > 1024 * 1024:
            raise ValueError('PCD DATA header missing or too large: ' + str(path))
        line = raw[offset:end + 1]
        offset = end + 1
        header.append(line)
        text = line.decode('ascii', 'strict').strip()
        if not text or text.startswith('#'):
            continue
        key, *tokens = text.split()
        key = key.upper()
        if key in values:
            raise ValueError('duplicate PCD header key: ' + key)
        values[key] = tokens
        if key == 'DATA':
            break
    mode = values['DATA']
    if len(mode) != 1 or mode[0].lower() not in ('ascii', 'binary'):
        raise UnsupportedPCDEncoding(
            'unsupported keyframe PCD DATA ' + ' '.join(mode)
            + '; use ASCII/binary keyframes; original patches will not be reused')
    mode = mode[0].lower()
    fields = values.get('FIELDS', values.get('FIELD', []))
    try:
        sizes = [int(v) for v in values['SIZE']]
        kinds = [v.upper() for v in values['TYPE']]
        counts = [int(v) for v in values.get('COUNT', ['1'] * len(fields))]
        count = int(values['POINTS'][0])
        width, height = int(values['WIDTH'][0]), int(values['HEIGHT'][0])
    except (KeyError, IndexError, ValueError) as exc:
        raise ValueError('invalid PCD field/count header: ' + str(path)) from exc
    if (not fields or not len(fields) == len(sizes) == len(kinds) == len(counts)
            or count < 0 or width < 0 or height < 1 or count != width * height):
        raise ValueError('invalid PCD layout or point count: ' + str(path))
    if any(fields.count(axis) != 1 for axis in ('x', 'y', 'z')):
        raise ValueError('PCD requires exactly one x, y and z field')
    position = column = 0
    xyz_layout = {}
    ascii_types = []
    for name, size, kind, components in zip(fields, sizes, kinds, counts):
        if (components < 1 or kind not in ('F', 'I', 'U')
                or size not in ((4, 8) if kind == 'F' else (1, 2, 4, 8))):
            raise ValueError('unsupported PCD field descriptor')
        if name in ('x', 'y', 'z'):
            if components != 1 or kind != 'F':
                raise ValueError('PCD XYZ must be scalar float32/float64 fields')
            xyz_layout[name] = (position, '<f' + str(size), column)
        position += size * components
        column += components
        ascii_types.extend([(kind, size)] * components)
    payload = raw[offset:]
    if mode == 'binary':
        if len(payload) != count * position:
            raise ValueError('PCD binary byte count mismatch')
        xyz = np.column_stack([
            np.ndarray((count,), dtype=xyz_layout[axis][1], buffer=payload,
                       offset=xyz_layout[axis][0], strides=(position,))
            for axis in ('x', 'y', 'z')
        ]).astype(np.float64) if count else np.empty((0, 3), dtype=np.float64)
    else:
        records = []
        points = []
        for line in payload.splitlines(keepends=True):
            tokens = line.split()
            if not tokens or line.lstrip().startswith(b'#'):
                continue
            if len(tokens) != column:
                raise ValueError('PCD ASCII field count mismatch')
            try:
                for token, (kind, size) in zip(tokens, ascii_types):
                    if kind == 'F':
                        float(token)
                    else:
                        number = int(token)
                        minimum = 0 if kind == 'U' else -(1 << (size * 8 - 1))
                        maximum = (1 << (size * 8 - (kind == 'I'))) - 1
                        if not minimum <= number <= maximum:
                            raise ValueError('PCD integer field is outside its declared type')
                points.append([float(tokens[xyz_layout[axis][2]]) for axis in ('x', 'y', 'z')])
            except ValueError as exc:
                raise ValueError('invalid PCD ASCII point record') from exc
            records.append(line)
        if len(records) != count:
            raise ValueError('PCD ASCII point count mismatch')
        payload = records
        xyz = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if not np.isfinite(xyz).all():
        raise ValueError('non-finite keyframe XYZ')
    return _PCD(header, mode, count, position, payload, xyz, hashlib.sha256(raw).hexdigest())


def _pose_entries(source):
    poses = source / 'poses_timed.txt'
    if not poses.is_file():
        poses = source / 'poses.txt'
    patches = (source / 'patches').resolve()
    if not poses.is_file() or not patches.is_dir():
        raise ValueError('keyframe editing requires recorded poses and patches')
    pose_bytes = poses.read_bytes()
    lines = pose_bytes.splitlines(keepends=True)
    entries, names = [], set()
    for number, raw in enumerate(lines, 1):
        text = raw.decode('utf-8').strip()
        if not text or text.startswith('#'):
            continue
        tokens = text.split()
        if len(tokens) not in (8, 9):
            raise ValueError(f'invalid keyframe pose columns at line {number}')
        name = tokens[0]
        relative = Path(name)
        path = (patches / relative).resolve()
        if (relative.is_absolute() or '..' in relative.parts or str(relative) != name or name in names
                or not path.is_relative_to(patches) or not path.is_file()):
            raise ValueError(f'duplicate, missing or invalid patch at line {number}: {name}')
        try:
            numbers = [float(v) for v in tokens[1:]]
        except ValueError as exc:
            raise ValueError(f'invalid keyframe pose at line {number}') from exc
        if not all(math.isfinite(v) for v in numbers):
            raise ValueError(f'non-finite keyframe pose at line {number}')
        x, y, z, qw, qx, qy, qz = numbers[-7:]
        rotation = _rotation(qw, qx, qy, qz)
        entries.append((number - 1, name, path, rotation, np.array([x, y, z])))
        names.add(name)
    if not entries:
        raise ValueError('no keyframe poses to edit')
    return poses, pose_bytes, lines, entries


def _file_revision(path):
    stat = path.stat()
    return stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns


def keyframes_identity(root):
    """Hash the pose index and exactly the patches it references, in pose order.

    Directory paths and unrelated files do not affect the content identity.
    Call before and after a builder to detect source changes across the build.
    No PCD parsing is needed here, so identity checks also support compressed
    sources even though the standalone editing operation explicitly rejects them.
    """
    source = Path(root).expanduser().resolve()
    poses, pose_bytes, _, entries = _pose_entries(source)
    paths = [poses, *[entry[2] for entry in entries]]
    revisions = [_file_revision(path) for path in paths]
    pose_sha = hashlib.sha256(pose_bytes).hexdigest()
    patches = [dict(name=name, sha256=_sha(path)) for _, name, path, _, _ in entries]
    if (_sha(poses) != pose_sha
            or any(_file_revision(path) != revision for path, revision in zip(paths, revisions))):
        raise ValueError('keyframe source changed while computing its identity')
    identity = dict(schema_version=1, poses_file=poses.name,
                    poses_sha256=pose_sha, patches=patches, entries=len(entries))
    encoded = json.dumps(identity, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    return dict(identity, sha256=hashlib.sha256(encoded).hexdigest())


def sync_keyframe_edits(source_dir, output_dir, operations, self_filter=None):
    """Return a READY provenance report only after every edited patch is saved.

    Source poses describe T_map_body, with ``name [stamp] x y z qw qx qy qz``.
    An optional normalized base_link mask filters body records first; it does
    not claim to apply a moving vehicle mask to the already fused global map.
    Empty frames are omitted from the new pose index and listed in the report.
    Sampling-only voxel operations remain the descriptor preprocessor's job.
    A failed/unsupported conversion leaves no published output directory and
    never silently falls back to unedited patches.
    """
    applied, sampling = _operations(operations)
    mask_spec = None
    if self_filter is not None and self_filter != {}:
        from agt_field_test_bringup.relocalization_self_filter import (
            self_filter_keep_mask, validate_self_filter_spec)
        mask_spec = validate_self_filter_spec(self_filter)
    source = Path(source_dir).expanduser().resolve()
    destination = Path(output_dir).expanduser().absolute()
    if destination.exists() or destination.is_symlink():
        raise ValueError('keyframe edit output already exists; select a new directory')
    if destination.resolve().is_relative_to(source):
        raise ValueError('keyframe edit output must be outside the immutable source')
    poses, pose_bytes, lines, entries = _pose_entries(source)
    pose_sha = hashlib.sha256(pose_bytes).hexdigest()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix='.' + destination.name + '-', dir=destination.parent))
    published = False
    try:
        (temporary / 'patches').mkdir()
        frame_reports, empty, omitted_lines = [], [], set()
        for line_index, name, path, rotation, translation in entries:
            cloud = _read_pcd(path)
            world = cloud.xyz @ rotation.T + translation
            if not np.isfinite(world).all():
                raise ValueError('non-finite keyframe coordinates after map transform: ' + name)
            keep = (self_filter_keep_mask(cloud.xyz, mask_spec) if mask_spec is not None
                    else np.ones(cloud.count, dtype=bool))
            after_self_filter = int(np.count_nonzero(keep))
            for operation in applied:
                inside = np.all((world >= operation['min']) & (world <= operation['max']), axis=1)
                keep &= inside if operation['type'] == 'crop' else ~inside
            remaining = int(np.count_nonzero(keep))
            output_sha = None
            if remaining:
                output = temporary / 'patches' / name
                output.parent.mkdir(parents=True, exist_ok=True)
                cloud.write_selected(output, keep)
                output_sha = _sha(output)
            else:
                empty.append(name)
                omitted_lines.add(line_index)
            frame_reports.append(dict(name=name, input_points=cloud.count,
                output_points=remaining, removed_points=cloud.count - remaining,
                self_filtered_points=cloud.count - after_self_filter,
                roi_removed_points=after_self_filter - remaining,
                input_sha256=cloud.sha256, output_sha256=output_sha))
        if len(empty) == len(entries):
            raise ValueError('keyframe edits leave no nonempty keyframes; no descriptor source published')
        output_poses = temporary / poses.name
        output_poses.write_bytes(b''.join(raw for i, raw in enumerate(lines) if i not in omitted_lines))
        # Detect concurrently changed sources before exposing a READY directory.
        if _sha(poses) != pose_sha:
            raise ValueError('source poses changed during keyframe editing')
        for (_, _, path, _, _), result in zip(entries, frame_reports):
            if _sha(path) != result['input_sha256']:
                raise ValueError('source keyframe changed during editing: ' + result['name'])
        report = dict(schema_version=1, state='READY', source_keyframes=str(source),
            output_keyframes=str(destination), source_poses=str(poses),
            source_poses_sha256=pose_sha, output_poses=str(destination / poses.name),
            output_poses_sha256=_sha(output_poses), selection_frame='map', output_frame='body',
            pose_and_timestamp_tokens_preserved=True, point_records_preserved=True,
            applied_operations=applied, sampling_operations_not_applied=sampling,
            self_filter=mask_spec,
            self_filter_scope='body keyframes only; fused PCD remains the separately recorded map ROI result',
            voxel_policy='fused-map sampling only; descriptor preprocessing controls keyframe sampling',
            synchronization_scope='listed ROI edits and optional keyframe self mask; earlier temporal cleanup is not reconstructed',
            input_frames=len(entries), output_frames=len(entries) - len(empty), empty_frames=empty,
            input_points=sum(r['input_points'] for r in frame_reports),
            output_points=sum(r['output_points'] for r in frame_reports), frames=frame_reports)
        report['removed_points'] = report['input_points'] - report['output_points']
        report['self_filtered_points'] = sum(r['self_filtered_points'] for r in frame_reports)
        report['roi_removed_points'] = sum(r['roi_removed_points'] for r in frame_reports)
        (temporary / 'edit_provenance.json').write_text(
            json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        # Reserve the final directory exclusively. A plain rename could replace
        # an empty directory created by somebody else between the checks.
        destination.mkdir(exist_ok=False)
        published = True
        for child in (temporary / 'patches', output_poses, temporary / 'edit_provenance.json'):
            os.rename(child, destination / child.name)
        temporary.rmdir()
        return report
    except Exception:
        if published:
            shutil.rmtree(destination)
        raise
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
