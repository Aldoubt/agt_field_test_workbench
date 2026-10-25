"""Asymmetric, explicitly synthetic fixed-map research fixtures; never field truth."""
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def scene(seed):
    rng=np.random.default_rng(seed);parts=[]
    for index in range(9):
        angle=rng.uniform(-np.pi,np.pi);distance=rng.uniform(5,17)
        along=rng.uniform(-5,5,1200);z=rng.uniform(-1,rng.uniform(2,6),1200)
        parts.append(np.column_stack([distance*np.cos(angle)-along*np.sin(angle),
                                      distance*np.sin(angle)+along*np.cos(angle),z]))
    parts.append(np.column_stack([rng.uniform(-16,16,2500),rng.uniform(-16,16,2500),np.full(2500,-1.)]))
    return np.vstack(parts)


def fixture(root):
    from agt_sc_kiss.geometry import write_pcd,contract_from_lio,write_json
    from agt_sc_kiss.database import build
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    keyframes=root/'keyframes';(keyframes/'patches').mkdir(parents=True)
    centers=[np.array([0.,0.,0.]),np.array([85.,10.,.3]),np.array([-80.,-20.,.1])]
    clouds=[];poses=[]
    for i,center in enumerate(centers):
        local=scene(i+10);world=local+center;clouds.append(world)
        q=Rotation.from_euler('xyz',[.08,-.12,i*.2]).as_quat()
        body=local@Rotation.from_quat(q).as_matrix()
        name=f'{i:06d}.pcd';write_pcd(keyframes/'patches'/name,body)
        poses.append(' '.join(map(str,[name,*center,q[3],*q[:3]])))
    (keyframes/'poses.txt').write_text('\n'.join(poses)+'\n')
    map_path=root/'map.pcd';write_pcd(map_path,np.vstack(clouds))
    pgm=root/'map.pgm';pgm.write_bytes(b'P5\n20 20\n255\n'+bytes([254])*400)
    nav=root/'map.yaml';nav.write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.1,origin=[0.,0.,0.],negate=0,occupied_thresh=.65,free_thresh=.196)))
    body_lidar=Rotation.from_euler('xyz',[.02,-.12,.04])
    config=root/'lio.yaml';config.write_text(yaml.safe_dump(dict(r_il=body_lidar.as_matrix().ravel().tolist(),t_il=[.11,-.07,.03])))
    spec=contract_from_lio(config);database=root/'scan_context';build(map_path,nav,keyframes,database,spec)
    rotation=Rotation.from_euler('xyz',[-.05,.08,np.radians(36)]);translation=centers[1]
    query=(clouds[1]-translation)@rotation.as_matrix();query_path=root/'query.pcd';write_pcd(query_path,query)
    write_json(root/'contract.json',spec)
    return dict(root=root,map=map_path,map_yaml=nav,keyframes=keyframes,config=config,database=database,
        query=query_path,points=query,map_points=clouds[1],rotation=rotation,translation=translation,spec=spec)
