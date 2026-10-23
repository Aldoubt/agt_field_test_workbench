"""Immutable full-resolution clouds and reproducible PCD-to-grid operations."""
from pathlib import Path
import json
import math
import shutil
import yaml
from .runtime import ROOT, ros_call, write_json
from .maps import Maps, _new_id, hash_file, read_pgm


def finite(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


def operations(items):
    if not isinstance(items,list) or len(items)>100: raise ValueError('invalid operations')
    for op in items:
        if op.get('type') in ('crop','delete'):
            if any(not isinstance(op.get(k),list) or len(op[k])!=3 or not all(map(finite,op[k])) for k in ('min','max')):
                raise ValueError('ROI needs finite XYZ bounds')
            if any(a>=b for a,b in zip(op['min'],op['max'])): raise ValueError('ROI min must be smaller than max')
        elif op.get('type')=='voxel':
            if not finite(op.get('size')) or op['size']<=0: raise ValueError('positive voxel size required')
        else: raise ValueError('unknown cloud operation')
    return items


def projection(p):
    defaults=dict(resolution=.05,origin=[0.,0.,0.],range=[20.,20.],z_min=.15,z_max=2.,ground_min=-.3,
                  min_points=2,occupied_thresh=.65,free_thresh=.2)
    if not isinstance(p,dict) or set(p)-set(defaults): raise ValueError('unknown projection parameter')
    defaults.update(p);p=defaults
    if any(not finite(p[k]) for k in ('resolution','z_min','z_max','ground_min','occupied_thresh','free_thresh')):
        raise ValueError('nonfinite projection parameter')
    if p['resolution']<=0 or p['z_min']>=p['z_max'] or p['ground_min']>p['z_min']: raise ValueError('invalid height or resolution')
    for k,n in (('origin',3),('range',2)):
        if not isinstance(p[k],list) or len(p[k])!=n or not all(map(finite,p[k])): raise ValueError('invalid '+k)
    if min(p['range'])<=0 or math.ceil(p['range'][0]/p['resolution'])*math.ceil(p['range'][1]/p['resolution'])>16_000_000:
        raise ValueError('output grid too large')
    if not isinstance(p['min_points'],int) or p['min_points']<1 or not 0<=p['free_thresh']<p['occupied_thresh']<=1:
        raise ValueError('invalid occupancy threshold')
    return p


class Clouds:
    def __init__(self,root=ROOT,runner=ros_call):
        self.root=Path(root); self.directory=self.root/'maps/clouds';self.runner=runner
        self.directory.mkdir(parents=True,exist_ok=True)
    def list(self):
        return [json.loads(p.read_text()) for p in sorted(self.directory.glob('*/manifest.json'),reverse=True)]
    def get(self,version):
        import re
        if not re.fullmatch(r'[0-9TZ-]{16,24}-[0-9a-f]{8}',version): raise ValueError('invalid cloud ID')
        record=json.loads((self.directory/version/'manifest.json').read_text())
        if hash_file(record['pcd'])!=record['sha256']: raise ValueError('PCD SHA changed')
        return record
    def _process(self,source,spec):
        ident=_new_id(); config=self.root/'runs'/ident/'operations.yaml';config.parent.mkdir(parents=True)
        config.write_text(yaml.safe_dump(spec,sort_keys=False));output=self.directory/ident
        self.runner(['ros2','run','agt_field_test_bringup','pcd_ops',str(source),str(config),str(output)],timeout=3600)
        return ident,output,config
    def import_pcd(self,path,calibration=None,bag=None):
        source=Path(path).expanduser().resolve()
        if not source.is_file() or source.suffix.lower()!='.pcd': raise ValueError('existing PCD required')
        sha=hash_file(source)
        ident,out,config=self._process(source,{'operations':[]})
        shutil.copyfile(source,out/'original.pcd')
        if hash_file(source)!=sha or hash_file(out/'original.pcd')!=sha: raise ValueError('source changed during import')
        bag_sha={str(p.relative_to(Path(bag))):hash_file(p) for p in Path(bag).rglob('*') if p.is_file()} if bag and Path(bag).is_dir() else None
        return self._save(ident,out,dict(source=str(source),source_sha256=sha,calibration=calibration,source_bag=bag,source_bag_sha256=bag_sha,parent=None,operations=[]))
    def _save(self,ident,out,lineage):
        record=dict(id=ident,pcd=str(out/'cloud.pcd'),sha256=hash_file(out/'cloud.pcd'),status='DRAFT',
                    frame_id='map',processing=yaml.safe_load((out/'result.yaml').read_text()),**lineage)
        write_json(out/'manifest.json',record);return record
    def clean(self,parent,items):
        original=self.get(parent);items=operations(items)
        ident,out,config=self._process(original['pcd'],{'operations':items})
        return self._save(ident,out,dict(parent=parent,parent_sha256=original['sha256'],operations=items,
            calibration=original.get('calibration'),source_bag=original.get('source_bag'),source_bag_sha256=original.get('source_bag_sha256'),source=original['pcd'],source_sha256=original['sha256']))
    def grid(self,parent,params):
        original=self.get(parent)
        if not params:
            bounds=original['processing'];lo=bounds['bounds_min'];hi=bounds['bounds_max'];res=.05
            params=dict(origin=[lo[0]-res,lo[1]-res,0.],range=[hi[0]-lo[0]+2*res,hi[1]-lo[1]+2*res])
        p=projection(params)
        ident,out,config=self._process(original['pcd'],{'operations':[],'projection':p})
        meta=yaml.safe_load((out/'map.yaml').read_text());w,h,pixels=read_pgm(out/'map.pgm')
        return Maps(self.root)._save(w,h,pixels,meta,dict(parent=None,cloud_id=parent,projection=p,
            source_pcd=original['pcd'],source_pcd_sha256=original['sha256'],
            source_bag=original.get('source_bag'),source_bag_sha256=original.get('source_bag_sha256'),calibration=original.get('calibration'),
            source_yaml=str(out/'map.yaml'),source_pgm_sha256=hash_file(out/'map.pgm')))
