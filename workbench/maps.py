"""Immutable 2D occupancy map versions, independent of review status."""
import base64
from datetime import datetime, timezone
import hashlib
import json
import math
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


def bake_keepout_pixels(pixels, width, height, polygons):
    """Bake cell-center polygons into occupied cells; geometry stays in PGM pixels."""
    raster=bytearray(pixels)
    if len(raster)!=width*height:raise ValueError('pixel count mismatch')
    def on_segment(x,y,a,b):
        cross=(x-a[0])*(b[1]-a[1])-(y-a[1])*(b[0]-a[0])
        return (abs(cross)<=1e-9 and min(a[0],b[0])-1e-9<=x<=max(a[0],b[0])+1e-9
                and min(a[1],b[1])-1e-9<=y<=max(a[1],b[1])+1e-9)
    def inside(x,y,polygon):
        contained=False;previous=polygon[-1]
        for current in polygon:
            if on_segment(x,y,previous,current):return True
            if (current[1]>y)!=(previous[1]>y):
                intersection=(previous[0]-current[0])*(y-current[1])/(previous[1]-current[1])+current[0]
                if x<intersection:contained=not contained
            previous=current
        return contained
    for polygon in polygons:
        if not polygon:continue
        xs=[float(p[0]) for p in polygon];ys=[float(p[1]) for p in polygon]
        x0=max(0,math.ceil(min(xs)));x1=min(width-1,math.floor(max(xs)))
        y0=max(0,math.ceil(min(ys)));y1=min(height-1,math.floor(max(ys)))
        for y in range(y0,y1+1):
            for x in range(x0,x1+1):
                if inside(float(x),float(y),polygon):raster[y*width+x]=0
    return bytes(raster)


def png(width, height, pixels):
    def chunk(tag, content):
        return struct.pack('>I', len(content)) + tag + content + struct.pack('>I', zlib.crc32(tag + content))
    scanlines = b''.join(b'\0' + pixels[y * width:(y + 1) * width] for y in range(height))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(scanlines)) + chunk(b'IEND', b''))


def relocalization_assets(record):
    """Return same-PCD optional assets; absence remains a supported legacy BBS fallback."""
    source=record.get('source_pcd')
    if not source:return ''
    directory=Path(source).expanduser().resolve().parent/'relocalization'
    return str(directory) if directory.is_dir() else ''


def asset_summary(record):
    directory=Path(relocalization_assets(record)) if relocalization_assets(record) else None
    if directory is None:return '未找到重定位资产：使用整图 3D-BBS 回退'
    polar=(directory/'polar_context.db').is_file()
    voxels=(directory/'voxelmaps_coords').is_dir()
    return ('AGT Polar Context（非 SC）：已生成' if polar else 'AGT Polar Context：未生成')+('；BBS 体素资产：已生成' if voxels else '；BBS 体素资产：未生成')


class Maps:
    def __init__(self, root=ROOT):
        self.root = Path(root)
        self.directory = self.root / 'maps/versions'
        self.directory.mkdir(parents=True, exist_ok=True)

    def list(self):
        records=[json.loads(p.read_text()) for p in sorted(self.directory.glob('*/manifest.json'), reverse=True)]
        return [dict(record,relocalization_assets=relocalization_assets(record),asset_summary=asset_summary(record)) for record in records]

    def get(self, map_id):
        if not re.fullmatch(r'[0-9TZ-]{16,24}-[0-9a-f]{8}', map_id):
            raise ValueError('invalid map id')
        path = self.directory / map_id / 'manifest.json'
        if not path.is_file():
            raise ValueError('map version not found')
        return json.loads(path.read_text())

    def import_yaml(self, yaml_path, calibration=None, source_bag=None, name=''):
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
            'name': name,
        })

    def revise(self, parent, pixels_b64, status='DRAFT', keepout_polygons_px=None):
        original = self.get(parent)
        if status not in STATES:
            raise ValueError('invalid map status')
        width, height = original['width'], original['height']
        pixels = base64.b64decode(pixels_b64, validate=True)
        if len(pixels) != width * height:
            raise ValueError('pixel count mismatch')
        polygons = keepout_polygons_px if keepout_polygons_px is not None else original.get('keepout_polygons_px', [])
        if not isinstance(polygons, list) or len(polygons) > 100:
            raise ValueError('too many keepout polygons')
        for polygon in polygons:
            if not isinstance(polygon, list) or not 3 <= len(polygon) <= 200:
                raise ValueError('invalid keepout polygon')
            for point in polygon:
                if (not isinstance(point, list) or len(point) != 2 or
                        any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in point) or
                        not 0 <= point[0] < width or not 0 <= point[1] < height):
                    raise ValueError('keepout polygon outside map')
        meta = yaml.safe_load((self.directory / parent / 'map.yaml').read_text())
        lineage = {k: original.get(k) for k in
                   ('source_yaml', 'source_pgm_sha256', 'source_pcd', 'source_pcd_sha256',
                    'source_bag', 'source_bag_sha256', 'calibration', 'cloud_id', 'projection', 'name')}
        lineage['parent'] = parent
        lineage['keepout_polygons_px'] = polygons
        pixels=bake_keepout_pixels(pixels,width,height,polygons)
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
        lineage=dict(lineage)
        lineage['name']=str(lineage.get('name') or '').strip()[:120]
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
        polygons_px = lineage.get('keepout_polygons_px', [])
        origin_x, origin_y, yaw = meta['origin']
        resolution = meta['resolution']
        def world(point):
            x = (point[0] + .5) * resolution
            y = (height - point[1] - .5) * resolution
            return [origin_x + math.cos(yaw) * x - math.sin(yaw) * y,
                    origin_y + math.sin(yaw) * x + math.cos(yaw) * y]
        world_polygons=[[world(p) for p in polygon] for polygon in polygons_px]
        # Keep the original flat list for existing Workbench readers and expose
        # the V3 bake_keepout_zones schema as well. Runtime uses the baked PGM.
        keepout = {'frame_id': 'map', 'polygons': world_polygons,
                   'zones':[{'id':index+1,'type':'keepout','polygon_m':polygon}
                            for index,polygon in enumerate(world_polygons)]}
        (directory / 'keepout_zones.yaml').write_text(yaml.safe_dump(keepout, sort_keys=False))
        bake={'schema_version':1,'zone_count':len(polygons_px),'inflate_cells':0,
              'baked_into':'map.pgm','coordinates':'cell-center pixels; origin yaw preserved in polygon_m'}
        (directory / 'keepout_bake.yaml').write_text(yaml.safe_dump(bake,sort_keys=False))
        manifest = {'id': map_id, 'status': status, 'created_at': datetime.now(timezone.utc).isoformat(),
                    'width': width, 'height': height, 'resolution': meta['resolution'],
                    'origin': meta['origin'], 'sha256': hashlib.sha256((directory / 'map.pgm').read_bytes()).hexdigest(),
                    'yaml': str(directory / 'map.yaml'), 'pgm': str(directory / 'map.pgm'),
                    'keepout_zones': str(directory / 'keepout_zones.yaml'),
                    'keepout_bake': str(directory / 'keepout_bake.yaml'), **lineage}
        write_json(directory / 'manifest.json', manifest)
        return manifest
