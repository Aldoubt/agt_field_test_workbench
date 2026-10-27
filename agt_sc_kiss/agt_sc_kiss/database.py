"""Immutable research DB beside, never replacing, Polar Context assets."""
import json
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from .geometry import sha,preprocess,level_rotation,write_json
import hashlib


def metadata_sha(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()


def build(map_pcd,map_yaml,keyframes,output,spec):
    from . import _scancontext as sc
    output=Path(output);keyframes=Path(keyframes)
    if output.exists():raise ValueError('SC DB output already exists; preserve it and select a new version')
    nav=yaml.safe_load(Path(map_yaml).read_text());pgm=(Path(map_yaml).parent/nav['image']).resolve()
    identities=dict(map_sha256=sha(map_pcd),navigation_yaml_sha256=sha(map_yaml),navigation_pgm_sha256=sha(pgm))
    poses=keyframes/'poses_timed.txt'
    if not poses.is_file():poses=keyframes/'poses.txt'
    entries=[];descriptions=[];index=sc.Index();seen=set();skipped=[]
    for line in poses.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith('#'):continue
        fields=line.split()
        if len(fields) not in (8,9):raise ValueError('keyframe pose must be name [stamp] x y z qw qx qy qz')
        name=fields[0];path=(keyframes/'patches'/name).resolve()
        if name in seen or not path.is_relative_to((keyframes/'patches').resolve()):raise ValueError('duplicate/invalid keyframe identity')
        seen.add(name);v=np.array(fields[-7:],dtype=float)
        if not np.isfinite(v).all() or not .99<np.linalg.norm(v[3:])<1.01:raise ValueError('invalid keyframe pose')
        q=[v[4],v[5],v[6],v[3]];level,yaw=level_rotation(q)
        raw=sc.load_pcd(str(path))
        body=preprocess(raw,spec,allow_empty=True)
        if len(body)<100:
            skipped.append(dict(name=name,reason='TOO_FEW_POINTS_AFTER_PREPROCESSING',
                input_points=len(raw),processed_points=len(body),minimum_points=100,pcd_sha256=sha(path)))
            continue
        descriptions.append(index.descriptor(body@level.T))
        entries.append(dict(name=name,translation=v[:3].tolist(),quaternion_xyzw=q,yaw=yaw,pcd_sha256=sha(path)))
    if not entries:raise ValueError(f'no usable keyframes; {len(skipped)} skipped below 100 processed points; fused PCD alone is not a keyframe database')
    if any(identities[k]!=sha(p) for k,p in [('map_sha256',map_pcd),('navigation_yaml_sha256',map_yaml),('navigation_pgm_sha256',pgm)]):
        raise ValueError('map source changed while building SC DB')
    output.mkdir(parents=True)
    np.savez_compressed(output/'descriptors.npz',descriptors=np.stack(descriptions))
    manifest=dict(schema_version=2,algorithm='scancontext',upstream_commit=sc.upstream_commit,
        map_path=str(Path(map_pcd).resolve()),navigation_yaml=str(Path(map_yaml).resolve()),
        navigation_geometry=dict(resolution=float(nav['resolution']),
                                  origin=[float(v) for v in nav['origin']]),
        preprocessing=spec,entries=entries,descriptors_sha256=sha(output/'descriptors.npz'),
        skipped_keyframes=skipped,keyframe_counts=dict(read=len(seen),indexed=len(entries),skipped=len(skipped)),
        pose_file_sha256=sha(poses),REAL_BAG_VERIFIED='NOT_RUN',**identities)
    manifest['metadata_sha256']=metadata_sha(manifest)
    write_json(output/'manifest.json',manifest)
    return manifest


class Database:
    def __init__(self,directory,map_pcd,spec):
        from . import _scancontext as sc
        self.directory=Path(directory);self.meta=json.loads((self.directory/'manifest.json').read_text())
        recorded=self.meta.get('metadata_sha256')
        if recorded!=metadata_sha({k:v for k,v in self.meta.items() if k!='metadata_sha256'}):raise ValueError('SC metadata SHA mismatch')
        self.verify_navigation()
        from .geometry import assert_contract
        assert_contract(spec,self.meta['preprocessing'])
        if self.meta['upstream_commit']!=sc.upstream_commit or self.meta['map_sha256']!=sha(map_pcd):raise ValueError('SC source map/version mismatch')
        path=self.directory/'descriptors.npz'
        if sha(path)!=self.meta['descriptors_sha256']:raise ValueError('SC descriptor SHA mismatch')
        with np.load(path,allow_pickle=False) as archive:descriptions=archive['descriptors']
        if len(descriptions)!=len(self.meta['entries']):raise ValueError('SC pose/descriptor count mismatch')
        self.index=sc.Index();self.index.load(list(descriptions))

    def verify_navigation(self):
        path=Path(self.meta['navigation_yaml']);nav=yaml.safe_load(path.read_text())
        if sha(path)!=self.meta['navigation_yaml_sha256'] or sha(path.parent/nav['image'])!=self.meta['navigation_pgm_sha256']:
            raise ValueError('SC navigation map/version mismatch')

    def query(self,body_points,body_quaternion,top_k=3):
        body=preprocess(body_points,self.meta['preprocessing']);level,_=level_rotation(body_quaternion)
        description=self.index.descriptor(body@level.T)
        candidates=[]
        for ident,distance,shift in self.index.query(description,int(top_k)):
            if distance>.13:continue  # unchanged upstream SC acceptance threshold
            entry=self.meta['entries'][ident]
            candidates.append(dict(entry,index=ident,distance=float(distance),
                yaw_seed=float(np.arctan2(np.sin(entry['yaw']-shift),np.cos(entry['yaw']-shift)))))
        return body,level,candidates
