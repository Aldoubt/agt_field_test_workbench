"""Version robot profiles without overwriting existing operator configuration."""
import json
from pathlib import Path
import yaml
from agt_field_test_bringup import profiles
from .runtime import ROOT,REPO,write_json
from .maps import _new_id,hash_file

def list_profiles(root=ROOT):
    result=[]
    for robot in ('bunker','fwmini'):
        try:
            path=profiles.selected_path(root,robot);value=profiles.load(path,robot);value['runtime_valid']=True
        except (ValueError,KeyError,TypeError,OSError) as exc:
            path=Path(root)/'config/robots'/(robot+'.yaml');original=yaml.safe_load(path.read_text()) if path.is_file() else {}
            candidate=yaml.safe_load((REPO/'templates/config/robots'/(robot+'.yaml')).read_text())
            for key in ('forward_mps','reverse_mps','angular_radps','linear_accel_mps2','linear_decel_mps2','control_hz','can_interface','footprint_m'):
                if key in original:candidate[key]=original[key]
            candidate['migration_source']=dict(path=str(path),sha256=hash_file(path) if path.is_file() else None)
            value=dict(robot=robot,runtime_valid=False,error=str(exc),profile_path=str(path),original=original,migration_candidate=candidate)
        result.append(value)
    return result

def save(root,robot,value):
    if robot not in ('bunker','fwmini') or value.get('robot')!=robot:raise ValueError('profile robot mismatch')
    value=dict(value)
    for key in ('profile_path','profile_sha256','topics','runtime_valid','error','migration_candidate','original'):value.pop(key,None)
    directory=Path(root)/'config/robots';directory.mkdir(parents=True,exist_ok=True);path=directory/(robot+'_'+_new_id()+'.yaml')
    with path.open('x') as stream:yaml.safe_dump(value,stream,sort_keys=False)
    try:loaded=profiles.load(path,robot)
    except Exception:path.unlink();raise
    write_json(directory/('active_'+robot+'.json'),dict(path=str(path),sha256=hash_file(path)))
    return loaded
