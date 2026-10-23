"""Map-bound immutable routes and relative-distance target geometry."""
import json
import math
from pathlib import Path
from .maps import Maps,_new_id,hash_file
from .runtime import ROOT,write_json


def pose(value):
    if not isinstance(value,dict) or set(value)-{'x','y','yaw','photo'}:raise ValueError('pose requires x/y/yaw')
    for k in ('x','y','yaw'):
        if isinstance(value.get(k),bool) or not isinstance(value.get(k),(int,float)) or not math.isfinite(value[k]):raise ValueError('finite pose required')
    return dict(x=value['x'],y=value['y'],yaw=value['yaw'],photo=bool(value.get('photo',False)))

def relative(current,distance):
    current=pose(current)
    if isinstance(distance,bool) or not isinstance(distance,(int,float)) or not math.isfinite(distance) or not 0<abs(distance)<=20:
        raise ValueError('relative distance must be nonzero and <=20 m')
    return dict(x=current['x']+distance*math.cos(current['yaw']),y=current['y']+distance*math.sin(current['yaw']),yaw=current['yaw'])

class Routes:
    def __init__(self,root=ROOT):self.root=Path(root);self.directory=self.root/'routes';self.directory.mkdir(parents=True,exist_ok=True)
    def list(self):return [json.loads(p.read_text()) for p in sorted(self.directory.glob('*/route.json'),reverse=True)]
    def save(self,map_id,points,name='',parent=None):
        m=Maps(self.root).get(map_id)
        if not isinstance(points,list) or not 1<=len(points)<=500:raise ValueError('1..500 waypoints required')
        points=[pose(p) for p in points]
        route=dict(id=_new_id(),name=str(name)[:120],map_id=map_id,map_sha256=m['sha256'],
                   map_yaml_sha256=hash_file(m['yaml']),frame_id='map',points=points,parent=parent,status='DRAFT',
                   photo_task_executor='camera waypoint executor required' if any(p['photo'] for p in points) else None)
        write_json(self.directory/route['id']/'route.json',route);return route
    def get(self,ident):
        import re
        if not re.fullmatch(r'[0-9TZ-]{16,24}-[0-9a-f]{8}',ident):raise ValueError('invalid route ID')
        return json.loads((self.directory/ident/'route.json').read_text())
    def validate(self,ident,map_id):
        r=self.get(ident);m=Maps(self.root).get(map_id)
        if r['map_id']!=map_id or r['map_sha256']!=m['sha256'] or r['map_yaml_sha256']!=hash_file(m['yaml']):
            raise ValueError('route/map identity changed; save a new route version')
        if r['frame_id']!='map':raise ValueError('route frame must be map')
        return r
