"""Strict, small PCD XYZ reader/writer: binary/ascii, including padded PointXYZ.

Fail closed on compressed/unsupported encodings; do not silently reinterpret
a fused map or modify input point records.
"""
from pathlib import Path
import numpy as np


def read_xyz(path):
    path = Path(path)
    header = {}
    with path.open('rb') as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('PCD DATA header missing: ' + str(path))
            text = line.decode('ascii', 'strict').strip()
            if not text or text.startswith('#'):
                continue
            name, *values = text.split()
            header[name.upper()] = values
            if name.upper() == 'DATA':
                offset = stream.tell()
                break
        fmt = header['DATA'][0].lower()
        if fmt not in ('binary', 'ascii'):
            raise ValueError('PCD encoding unsupported (no compressed support): ' + fmt)
        names = header.get('FIELDS', header.get('FIELD', []))
        sizes = [int(v) for v in header['SIZE']]
        kinds = header['TYPE']
        counts = [int(v) for v in header.get('COUNT', ['1'] * len(names))]
        if not (len(names) == len(sizes) == len(kinds) == len(counts)):
            raise ValueError('invalid PCD field descriptor')
        if not all(name in names for name in ('x', 'y', 'z')):
            raise ValueError('PCD must contain x y z')
        formats, offsets, position = [], [], 0
        for size, kind, count in zip(sizes, kinds, counts):
            code = {'F': {4: '<f4', 8: '<f8'},
                    'I': {1: 'i1', 2: '<i2', 4: '<i4'},
                    'U': {1: 'u1', 2: '<u2', 4: '<u4'}}.get(kind, {}).get(size)
            if code is None or count < 1:
                raise ValueError('unsupported PCD field type/count')
            formats.append((np.dtype(code), (count,)) if count > 1 else np.dtype(code))
            offsets.append(position)
            position += size * count
        dtype = np.dtype(dict(names=names, formats=formats, offsets=offsets, itemsize=position))
        n = int(header.get('POINTS', ['-1'])[0])
        if n < 0 or n != int(header.get('WIDTH', [str(n)])[0]) * int(header.get('HEIGHT', ['1'])[0]):
            raise ValueError('invalid PCD point count')
        if fmt == 'binary':
            if path.stat().st_size - offset != n * dtype.itemsize:
                raise ValueError('PCD data byte count mismatch')
            points = np.memmap(path, dtype=dtype, mode='r', offset=offset, shape=(n,))
        else:
            points = np.genfromtxt(stream, dtype=dtype, max_rows=n) if n else np.empty((0,), dtype=dtype)
        result = np.column_stack([points[k] for k in ('x', 'y', 'z')]).astype(np.float32, copy=False)
        if not np.isfinite(result).all():
            raise ValueError('non-finite PCD XYZ')
        return result


def write_xyz(path, points):
    path = Path(path)
    xyz = np.ascontiguousarray(points, dtype='<f4')
    if xyz.ndim != 2 or xyz.shape[1] != 3 or not np.isfinite(xyz).all():
        raise ValueError('invalid XYZ output')
    with path.open('wb') as stream:
        stream.write(('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                      f'WIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                      f'POINTS {len(xyz)}\nDATA binary\n').encode('ascii'))
        stream.write(xyz.tobytes())
