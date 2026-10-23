"""Candidate bundle versions; physical verification never inferred from software."""
import copy,hashlib,json
from pathlib import Path
import yaml
from .runtime import ROOT,write_json
from .maps import _new_id
from .fwmini import sha256
from agt_field_test_bringup import calibration

class Bundles:
    def __init__(self,root=ROOT):self.root=Path(root)
    def list(self,robot):
        if robot not in ('bunker','fwmini'):raise ValueError('unknown robot')
        items=[]
        for p in (self.root/'calibration'/robot).glob('*.yaml'):
            data=yaml.safe_load(p.read_text())
            if isinstance(data,dict) and data.get('kind')=='robot_calibration_bundle':
                items.append(dict(calibration.load(p,robot),path=str(p),calibration_sha256=calibration.digest(data)))
        return items
    def save(self,robot,data):
        if robot not in ('bunker','fwmini'):raise ValueError('unknown robot')
        value=copy.deepcopy(data);value.pop('path',None);value.pop('calibration_sha256',None);value['robot']=robot;value['kind']='robot_calibration_bundle';value['status']='CANDIDATE';value.pop('verification',None)
        value['version']=_new_id();directory=self.root/'calibration'/robot;directory.mkdir(parents=True,exist_ok=True)
        temporary=directory/(value['version']+'.yaml')
        with temporary.open('x') as stream:yaml.safe_dump(value,stream,sort_keys=False)
        try:calibration.load(temporary,robot)
        except Exception:temporary.unlink();raise
        return dict(value,path=str(temporary),calibration_sha256=calibration.digest(value))
    def verify(self,source,report):
        source=Path(source).resolve();report=Path(report).resolve()
        if not source.is_relative_to((self.root/'calibration').resolve()) or not any(report.is_relative_to((self.root/p).resolve()) for p in ('reports','runs')):raise ValueError('workspace calibration/report required')
        value=calibration.load(source);value['status']='VERIFIED';value['verification']={'report':str(report),'report_sha256':sha256(report)}
        target=source.parent/('verified_'+_new_id()+'.yaml')
        with target.open('x') as stream:yaml.safe_dump(value,stream,sort_keys=False)
        try:calibration.load(target)
        except Exception:target.unlink();raise
        return dict(value,path=str(target),calibration_sha256=calibration.digest(value))
