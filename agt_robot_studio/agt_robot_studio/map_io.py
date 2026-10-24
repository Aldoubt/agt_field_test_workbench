"""Deterministic Nav2 OccupancyGrid/PGM export. Unknown is NEVER free."""
from pathlib import Path
import numpy as np


def occupancy_to_gray(values, width, height):
    cells = np.asarray(values, dtype=np.int16).reshape((height, width))
    gray = np.full(cells.shape, 205, dtype=np.uint8)
    gray[(cells >= 0) & (cells <= 19)] = 254
    gray[cells >= 65] = 0
    # OccupancyGrid begins at bottom-left; PGM first row is top-left.
    # QImage requires a contiguous buffer rather than NumPy's negative-stride view.
    return np.ascontiguousarray(np.flipud(gray))


def grayscale_to_occupancy(gray):
    a = np.flipud(np.asarray(gray, dtype=np.uint8))
    out = np.full(a.shape, -1, dtype=np.int8)
    out[a >= 250] = 0
    out[a <= 10] = 100
    return out.ravel().tolist()


def save_nav2_map(directory, gray, resolution, origin, name='map'):
    """Create new map assets. Caller must supply an unused output directory."""
    d = Path(directory).expanduser()
    if d.exists():
        raise FileExistsError(f'Cannot overwrite existing map directory: {d}')
    d.mkdir(parents=True, exist_ok=False)
    data = np.asarray(gray, dtype=np.uint8)
    if data.ndim != 2 or not 0 < data.size < 20_000_000:
        raise ValueError('invalid map shape')
    if not 0 < resolution <= 1.0:
        raise ValueError('invalid resolution')
    if len(origin) != 3 or not all(np.isfinite(origin)):
        raise ValueError('invalid origin')
    h, w = data.shape
    image = d / f'{name}.pgm'
    with image.open('wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode('ascii'))
        f.write(data.tobytes())
    yaml = d / f'{name}.yaml'
    yaml.write_text(
        f'image: {name}.pgm\nmode: trinary\nresolution: {resolution:.8f}\n'
        f'origin: [{origin[0]:.8f}, {origin[1]:.8f}, {origin[2]:.8f}]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n', encoding='utf8')
    return str(yaml)
