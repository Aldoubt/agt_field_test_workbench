"""Bounded read-only XYZ samples from standard ASCII or binary PCD files."""
import math
from pathlib import Path
import struct


def sample_pcd(path, maximum=12000):
    path = Path(path)
    with path.open('rb') as stream:
        header_lines = []
        while True:
            line = stream.readline(16384)
            if not line or len(line) == 16384:
                raise ValueError('invalid PCD header')
            header_lines.append(line.decode('ascii', errors='replace').strip())
            if line.upper().startswith(b'DATA '):
                break
            if len(header_lines) > 100:
                raise ValueError('PCD header too long')
        info = {}
        for line in header_lines:
            key, _, value = line.partition(' ')
            info[key.upper()] = value.strip()
        fields = info['FIELDS'].split()
        sizes = list(map(int, info['SIZE'].split()))
        counts = list(map(int, info.get('COUNT', ' '.join(['1'] * len(fields))).split()))
        types = info['TYPE'].split()
        total = int(info.get('POINTS', '0'))
        if total < 1 or total > 200_000_000 or not all(x in fields for x in ('x', 'y', 'z')):
            raise ValueError('invalid PCD points or XYZ fields')
        offsets, step = {}, 0
        for field, size, count, typ in zip(fields, sizes, counts, types):
            offsets[field] = (step, size, typ)
            step += size * count
        stride = max(1, math.ceil(total / maximum))
        data_mode = info['DATA'].lower()
        points = []
        if data_mode == 'ascii':
            indices = {name: fields.index(name) for name in ('x', 'y', 'z')}
            for index, line in enumerate(stream):
                if index % stride:
                    continue
                parts = line.split()
                try:
                    xyz = [float(parts[indices[name]]) for name in ('x', 'y', 'z')]
                except (ValueError, IndexError):
                    continue
                if all(math.isfinite(x) for x in xyz):
                    points.append(xyz)
        elif data_mode == 'binary':
            data_start = stream.tell()
            for index in range(0, total, stride):
                stream.seek(data_start + index * step)
                record = stream.read(step)
                if len(record) != step:
                    break
                xyz = []
                for name in ('x', 'y', 'z'):
                    offset, size, typ = offsets[name]
                    if (typ, size) != ('F', 4):
                        raise ValueError('3D preview requires float32 XYZ PCD fields')
                    xyz.append(struct.unpack_from('<f', record, offset)[0])
                if all(math.isfinite(x) for x in xyz):
                    points.append(xyz)
        else:
            raise ValueError(f'3D preview does not support PCD DATA {data_mode}')
    return {'source': str(path), 'total_points': total, 'sampled_points': len(points), 'points': points}
