"""Map-bound immutable routes and relative-distance target geometry."""
import json
import math
from pathlib import Path
import yaml
from .maps import Maps,_new_id,hash_file
from .goal_safety import validate_endpoints
from .runtime import ROOT,write_json


def pose(value):
    allowed={'x','y','yaw','photo','heading','roll','pitch','photo_required'}
    if not isinstance(value,dict) or set(value)-allowed:raise ValueError('pose requires x/y/yaw and optional capture settings')
    for k in ('x','y','yaw'):
        if isinstance(value.get(k),bool) or not isinstance(value.get(k),(int,float)) or not math.isfinite(value[k]):raise ValueError('finite pose required')
    result=dict(x=value['x'],y=value['y'],yaw=value['yaw'],photo=bool(value.get('photo',False)))
    for k in ('heading','roll','pitch'):
        angle=value.get(k,0.)
        if isinstance(angle,bool) or not isinstance(angle,(int,float)) or not math.isfinite(angle) or abs(angle)>180:
            raise ValueError(k+' must be a finite angle in degrees')
        result[k]=float(angle)
    result['photo_required']=bool(value.get('photo_required',True))
    return result

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
        # Reject targets outside observed free cells when the route is created;
        # enforce the active robot's full footprint again when dispatched.
        validate_endpoints(m,points)
        route=dict(id=_new_id(),name=str(name)[:120],map_id=map_id,map_sha256=m['sha256'],
                   map_yaml_sha256=hash_file(m['yaml']),frame_id='map',points=points,parent=parent,status='DRAFT',
                   photo_task_executor='agt_navigation_runtime/mission_runtime' if any(p['photo'] for p in points) else None)
        directory=self.directory/route['id']
        mission=dict(version=1,mission_id=route['id'],map_id=map_id,points=[])
        for index,p in enumerate(points):
            point_id=f'P{index+1:03d}'
            views=[]
            if p['photo']:
                views=[dict(tag=point_id+'_view',heading=p['heading'],roll=p['roll'],pitch=p['pitch'],
                            required=p['photo_required'],save_image=True)]
            mission['points'].append(dict(id=point_id,pose=dict(x=p['x'],y=p['y'],yaw=p['yaw'],frame_id='map'),
                                          settle_time=1.0,views=views))
        mission_path=directory/'mission.yaml'
        mission_path.parent.mkdir(parents=True,exist_ok=True)
        mission_path.write_text(yaml.safe_dump(mission,sort_keys=False,allow_unicode=True),encoding='utf8')
        route['mission_file']=str(mission_path)
        write_json(directory/'route.json',route);return route
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
