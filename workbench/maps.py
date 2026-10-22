"""Immutable 2D occupancy map versions, independent of review status."""
import base64
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import struct
import uuid
import zlib

import yaml

from .runtime import ROOT, write_json

STATES = {'DRAFT', 'PROVISIONAL', 'ACCEPTED', 'REJECTED'}


def hash_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _new_id():
    return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8]


def read_pgm(path):
    data = path.read_bytes()
    offset = 0
    values = []
    while len(values) < 4:
        while offset < len(data) and data[offset] in b' \t\r\n':
            offset += 1
        if offset < len(data) and data[offset] == 35:
            offset = data.find(b'\n', offset) + 1
            if offset == 0:
                raise ValueError('unterminated PGM comment')
            continue
        match = re.match(rb'\S+', data[offset:])
        if not match:
            raise ValueError('invalid PGM header')
        values.append(match.group().decode('ascii'))
        offset += match.end()
    magic, width, height, maximum = values
    width, height, maximum = int(width), int(height), int(maximum)
    if magic not in ('P5', 'P2') or not 0 < width * height <= 16_000_000 or maximum != 255:
        raise ValueError('unsupported PGM dimensions or depth')
    if magic == 'P5':
        if offset >= len(data) or data[offset] not in b' \t\r\n':
            raise ValueError('PGM payload separator missing')
        pixels = data[offset + 1:offset + 1 + width * height]
    else:
        pixels = bytes(int(n) for n in re.findall(rb'\d+', data[offset:]))
    if len(pixels) != width * height:
        raise ValueError('PGM payload length mismatch')
    return width, height, pixels


def write_pgm(path, width, height, pixels):
    if len(pixels) != width * height or width * height > 16_000_000:
        raise ValueError('pixel count mismatch')
    path.write_bytes(f'P5\n{width} {height}\n255\n'.encode() + pixels)


def png(width, height, pixels):
    def chunk(tag, content):
        return struct.pack('>I', len(content)) + tag + content + struct.pack('>I', zlib.crc32(tag + content))
    scanlines = b''.join(b'\0' + pixels[y * width:(y + 1) * width] for y in range(height))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(scanlines)) + chunk(b'IEND', b''))


class Maps:
    def __init__(self, root=ROOT):
        self.root = Path(root)
        self.directory = self.root / 'maps/versions'
        self.directory.mkdir(parents=True, exist_ok=True)

    def list(self):
        return [json.loads(p.read_text()) for p in sorted(self.directory.glob('*/manifest.json'), reverse=True)]

    def get(self, map_id):
        if not re.fullmatch(r'[0-9TZ-]{16,24}-[0-9a-f]{8}', map_id):
            raise ValueError('invalid map id')
        path = self.directory / map_id / 'manifest.json'
        if not path.is_file():
            raise ValueError('map version not found')
        return json.loads(path.read_text())

    def import_yaml(self, yaml_path, calibration=None, source_bag=None):
        source = Path(yaml_path).expanduser().resolve()
        if not any(source.is_relative_to(self.root / p) for p in ('runs', 'maps')):
            raise ValueError('map import must come from this workspace runs or maps')
        if not source.is_file():
            raise ValueError('map YAML not found')
        meta = yaml.safe_load(source.read_text())
        image = (source.parent / meta['image']).resolve()
        if not image.is_relative_to(source.parent) or image.suffix.lower() != '.pgm':
            raise ValueError('map image must be a PGM beside its YAML')
        width, height, pixels = read_pgm(image)
        pcd = source.parent.parent / 'map_package' / 'map.pcd'
        return self._save(width, height, pixels, meta, {
            'source_yaml': str(source), 'source_pgm_sha256': hashlib.sha256(image.read_bytes()).hexdigest(),
            'source_pcd': str(pcd) if pcd.is_file() else None,
            'source_pcd_sha256': hash_file(pcd) if pcd.is_file() else None,
            'source_bag': source_bag, 'calibration': calibration, 'parent': None,
        })

    def revise(self, parent, pixels_b64, status='DRAFT'):
        original = self.get(parent)
        if status not in STATES:
            raise ValueError('invalid map status')
        width, height = original['width'], original['height']
        pixels = base64.b64decode(pixels_b64, validate=True)
        if len(pixels) != width * height:
            raise ValueError('pixel count mismatch')
        meta = yaml.safe_load((self.directory / parent / 'map.yaml').read_text())
        lineage = {k: original.get(k) for k in
                   ('source_yaml', 'source_pgm_sha256', 'source_pcd', 'source_pcd_sha256',
                    'source_bag', 'calibration')}
        lineage['parent'] = parent
        return self._save(width, height, pixels, meta, lineage, status)

    def set_status(self, map_id, status, reason):
        if status not in STATES or not reason.strip():
            raise ValueError('map status and reason required')
        manifest = self.get(map_id)
        manifest['status'] = status
        manifest['review_reason'] = reason[:500]
        manifest['reviewed_at'] = datetime.now(timezone.utc).isoformat()
        write_json(self.directory / map_id / 'manifest.json', manifest)
        return manifest

    def preview(self, map_id):
        manifest = self.get(map_id)
        _, _, pixels = read_pgm(self.directory / map_id / 'map.pgm')
        return png(manifest['width'], manifest['height'], pixels)

    def _save(self, width, height, pixels, meta, lineage, status='DRAFT'):
        if not isinstance(meta.get('resolution'), (int, float)) or meta['resolution'] <= 0:
            raise ValueError('positive map resolution required')
        if len(meta.get('origin', [])) != 3:
            raise ValueError('map origin [x,y,yaw] required')
        map_id = _new_id()
        directory = self.directory / map_id
        directory.mkdir(parents=True, exist_ok=False)
        write_pgm(directory / 'map.pgm', width, height, pixels)
        config = {key: meta.get(key) for key in
                  ('resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh')}
        config['image'] = 'map.pgm'
        (directory / 'map.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
        manifest = {'id': map_id, 'status': status, 'created_at': datetime.now(timezone.utc).isoformat(),
                    'width': width, 'height': height, 'resolution': meta['resolution'],
                    'origin': meta['origin'], 'sha256': hashlib.sha256((directory / 'map.pgm').read_bytes()).hexdigest(),
                    'yaml': str(directory / 'map.yaml'), 'pgm': str(directory / 'map.pgm'), **lineage}
        write_json(directory / 'manifest.json', manifest)
        return manifest
