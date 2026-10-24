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


def png_rgb(width, height, rgb):
    """Encode a tightly packed RGB image without optional imaging libraries."""
    if width < 1 or height < 1 or len(rgb) != width * height * 3:
        raise ValueError('invalid RGB image dimensions')
    def chunk(tag, content):
        return struct.pack('>I', len(content)) + tag + content + struct.pack('>I', zlib.crc32(tag + content))
    stride = width * 3
    scanlines = b''.join(b'\\0' + rgb[y*stride:(y+1)*stride] for y in range(height))
    return (b'\\x89PNG\\r\\n\\x1a\\n' +
            chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)) +
            chunk(b'IDAT', zlib.compress(scanlines)) + chunk(b'IEND', b''))


def _asset_health(record):
    source=record.get('source_pcd')
    if not source:return None, {}, False, False
    directory=Path(source).expanduser().resolve().parent/'relocalization'
    if not directory.is_dir():return None, {}, False, False
    status={}
    status_path=directory/'asset_status.json'
    if status_path.is_file():
        try:status=json.loads(status_path.read_text())
        except (OSError,ValueError):pass
    mandatory=(directory/'voxelmaps_coords').is_dir() and (directory/'global_map_downsampled.pcd').is_file()
    fresh=(status.get('state')=='READY' and
           status.get('source_sha256')==record.get('source_pcd_sha256'))
    if status_path.exists():
        return directory,status,mandatory and fresh,False
    # Compatibility with V3 PGO map packages built before asset_status.json:
    # require metadata to name this exact PCD, not merely an adjacent directory.
    meta_path=directory/'relocalization_assets.yaml'
    if mandatory and meta_path.is_file():
        try:
            meta=yaml.safe_load(meta_path.read_text())
            if (isinstance(meta,dict) and meta.get('source_map') and
                Path(meta['source_map']).expanduser().resolve()==Path(source).expanduser().resolve()):
                return directory,status,True,True
        except (OSError,ValueError,TypeError):pass
    return directory,status,False,False


def relocalization_assets(record):
    """Never expose incomplete or stale BBS assets to V3 localization."""
    directory,status,ready,legacy=_asset_health(record)
    return str(directory) if ready else ''


def asset_summary(record):
    directory,status,ready,legacy=_asset_health(record)
    if directory is None:return '未找到重定位资产：使用整图 3D-BBS 回退'
    polar=(directory/'polar_context.db').is_file() and ready
    bbs=('已生成（旧版按源 PCD 路径校验，未记录 SHA）' if legacy else
         '已生成（同源 SHA）' if ready else
         '正在构建' if status.get('bbs')=='BUILDING' else
         '构建失败：'+str(status.get('error','请检查依赖'))[:120] if status.get('bbs')=='FAILED' else
         '未生成／版本校验失败')
    return ('AGT Polar Context（非 SC）：已生成' if polar else
            'AGT Polar Context：未生成（没有可用的关键帧描述子库）')+'；BBS 体素资产：'+bbs

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

    def compare_and_propose(self, base_id, candidate_id):
        """Compare an accepted map with a new same-source-PCD draft and create an immutable update draft.

        The new map grid remains the candidate's grid and PCD identity. The old accepted
        map is never modified, and the new version is never auto-accepted. Grid alignment
        must be exact to within a quarter cell; this method does not estimate SLAM drift.
        """
        import numpy as np
        base=self.get(base_id);candidate=self.get(candidate_id)
        if base_id==candidate_id:raise ValueError('base and candidate map versions must differ')
        if base.get('status')!='ACCEPTED':raise ValueError('base map must be ACCEPTED')
        if candidate.get('status')!='DRAFT':raise ValueError('candidate map must be DRAFT; review status is immutable workflow state')
        for label,record in (('base',base),('candidate',candidate)):
            if not record.get('source_pcd') or not record.get('source_pcd_sha256'):
                raise ValueError(label+' map must be linked to a source PCD and SHA-256')
            source=Path(record['source_pcd']).expanduser().resolve()
            if not source.is_file() or hash_file(source)!=record['source_pcd_sha256']:
                raise ValueError(label+' source PCD missing or SHA-256 changed')
            pgm=Path(record['pgm']).resolve()
            if not pgm.is_file() or hash_file(pgm)!=record['sha256']:
                raise ValueError(label+' PGM missing or SHA-256 changed')
            yaml_path=Path(record['yaml']).resolve()
            if not yaml_path.is_file():raise ValueError(label+' map YAML missing')
        base_meta=yaml.safe_load(Path(base['yaml']).read_text())
        candidate_meta=yaml.safe_load(Path(candidate['yaml']).read_text())
        resolution=float(base_meta['resolution']);candidate_resolution=float(candidate_meta['resolution'])
        if abs(resolution-candidate_resolution)>1e-7:
            raise ValueError('map resolutions differ; automatic diff requires identical resolution')
        bx,by,byaw=map(float,base_meta['origin'])
        cx,cy,cyaw=map(float,candidate_meta['origin'])
        yaw_delta=(cyaw-byaw+math.pi)%(2*math.pi)-math.pi
        if abs(yaw_delta)>1e-4:
            raise ValueError('map origin yaw differs; automatic diff will not guess a rotation')
        cell_x=(cx-bx)/resolution;cell_y=(cy-by)/resolution
        offset_x=int(round(cell_x));offset_y=int(round(cell_y))
        if abs(cell_x-offset_x)>.25 or abs(cell_y-offset_y)>.25:
            raise ValueError('map origins are not cell-aligned within 0.25 cell; align maps before comparing')
        bw,bh,bpixels=read_pgm(Path(base['pgm']))
        cw,ch,cpixels=read_pgm(Path(candidate['pgm']))
        def states(width,height,pixels):
            raw=np.frombuffer(pixels,dtype=np.uint8).reshape((height,width))
            result=np.full((height,width),-1,dtype=np.int8)
            result[raw<=64]=100
            result[raw>=220]=0
            return result
        old=states(bw,bh,bpixels);new=states(cw,ch,cpixels)
        min_x=min(0,offset_x);max_x=max(bw-1,offset_x+cw-1)
        min_y=min(0,offset_y);max_y=max(bh-1,offset_y+ch-1)
        width=max_x-min_x+1;height=max_y-min_y+1
        if width*height>16_000_000:raise ValueError('aligned map union exceeds safe 16 MP diff limit')
        old_grid=np.full((height,width),-1,dtype=np.int8)
        new_grid=np.full((height,width),-1,dtype=np.int8)
        old_row=max_y-(bh-1);old_col=-min_x
        new_row=max_y-(offset_y+ch-1);new_col=offset_x-min_x
        old_grid[old_row:old_row+bh,old_col:old_col+bw]=old
        new_grid[new_row:new_row+ch,new_col:new_col+cw]=new
        old_unknown=old_grid<0;new_unknown=new_grid<0
        new_obstacle=(new_grid==100)&(old_grid!=100)
        cleared=(old_grid==100)&(new_grid==0)
        newly_observed=old_unknown&(~new_unknown)&(~new_obstacle)
        coverage_lost=(~old_unknown)&new_unknown
        stable_occupied=(old_grid==100)&(new_grid==100)
        stable_free=(old_grid==0)&(new_grid==0)
        changed=(old_grid!=new_grid)&((~old_unknown)|(~new_unknown))
        rgb=np.zeros((height,width,3),dtype=np.uint8)
        rgb[:]=[100,108,116] # unknown in both / outside coverage
        rgb[stable_free]=[235,239,240]
        rgb[stable_occupied]=[26,36,43]
        rgb[newly_observed]=[72,177,118]
        rgb[coverage_lost]=[229,177,68]
        rgb[cleared]=[55,156,225]
        rgb[new_obstacle]=[230,75,70]
        report_id=_new_id()
        diff_root=self.root/'maps'/'diffs'/report_id
        diff_root.mkdir(parents=True,exist_ok=False)
        overlay=diff_root/'overlay.png'
        overlay.write_bytes(png_rgb(width,height,rgb.tobytes()))
        # This is a new immutable map version backed by the candidate's exact
        # PCD and grid. It is not a cell-wise fusion with an unrelated PCD.
        lineage={k:candidate.get(k) for k in (
            'source_yaml','source_pgm_sha256','source_pcd','source_pcd_sha256',
            'source_bag','source_bag_sha256','calibration','cloud_id','projection','name')}
        lineage.update(parent=base_id,keepout_polygons_px=candidate.get('keepout_polygons_px',[]),
            update_type='AUTOMATIC_DIFF_DRAFT',base_map_id=base_id,
            base_map_sha256=base['sha256'],candidate_map_id=candidate_id,
            candidate_map_sha256=candidate['sha256'],diff_report_id=report_id,
            diff_overlay=str(overlay),review_required=True,automatic_activation=False)
        update=self._save(cw,ch,cpixels,candidate_meta,lineage,status='DRAFT')
        report={
            'id':report_id,'created_at':datetime.now(timezone.utc).isoformat(),
            'state':'REVIEW_REQUIRED','base_map_id':base_id,'base_map_sha256':base['sha256'],
            'candidate_map_id':candidate_id,'candidate_map_sha256':candidate['sha256'],
            'update_map_id':update['id'],'update_map_status':'DRAFT',
            'base_pcd_sha256':base['source_pcd_sha256'],
            'candidate_pcd_sha256':candidate['source_pcd_sha256'],
            'resolution_m':resolution,'origin_yaw_delta_rad':yaw_delta,
            'candidate_offset_cells_xy':[offset_x,offset_y],
            'aligned_width':width,'aligned_height':height,
            'counts':{
                'new_obstacle_cells':int(np.count_nonzero(new_obstacle)),
                'cleared_obstacle_cells':int(np.count_nonzero(cleared)),
                'newly_observed_cells':int(np.count_nonzero(newly_observed)),
                'coverage_lost_cells':int(np.count_nonzero(coverage_lost)),
                'stable_occupied_cells':int(np.count_nonzero(stable_occupied)),
                'stable_free_cells':int(np.count_nonzero(stable_free)),
                'changed_cells':int(np.count_nonzero(changed)),
                'unknown_both_cells':int(np.count_nonzero(old_unknown&new_unknown)),
            },
            'overlay_png':str(overlay),'report_path':str(diff_root/'report.json'),
            'update_map_path':update['manifest_path'] if 'manifest_path' in update else str(self.directory/update['id']/'manifest.json'),
            'automatic_activation':False,'navigation_allowed':False,
            'acceptance_rule':'human_review_required; same candidate PCD remains localization source; rebuild PGM/BBS from this PCD if edited geometry is accepted',
        }
        write_json(diff_root/'report.json',report)
        update['diff_report']=str(diff_root/'report.json')
        return {'report':report,'update_map':update}

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
        return dict(manifest, asset_summary=asset_summary(manifest),
                    relocalization_assets=relocalization_assets(manifest))
