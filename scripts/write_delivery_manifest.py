#!/usr/bin/env python3
"""Write a new report bound to a clean, committed integration candidate."""
import hashlib,json,subprocess,uuid,sys
from datetime import datetime,timezone
from pathlib import Path
REPO=Path(__file__).resolve().parents[1];ROOT=REPO.parent.parent

def git(*args):return subprocess.check_output(['git','-c','safe.directory='+str(REPO),'-C',str(REPO),*args],text=True).strip()
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
if git('status','--porcelain'):raise SystemExit('commit/review integration tree first; existing edits preserved')
head=git('rev-parse','HEAD');evidence=REPO/'docs/evidence/field_loop_20261023'
required={'unit_final.log':('Ran 22 tests','OK'),'colcon_test_final.log':('135 tests, 0 errors, 0 failures',),
          'ros_field_loop_final.log':('Ran 7 tests','OK'),'fwmini_control_final.log':('Ran 9 tests','OK'),
          'fwmini_tf_final.log':('Ran 4 tests','OK'),'final_runtime_recheck.log':('Ran 1 test','OK'),
          'session_lifecycle_final.log':('"state": "PASS"',)}
for name,needles in required.items():
    text=(evidence/name).read_text()
    if not all(n in text for n in needles) or 'FAILED (' in text:raise SystemExit('software evidence not passing: '+name)
audit=json.loads(subprocess.check_output(['python3',str(REPO/'scripts/audit.py')],text=True))
if audit['dirty'] or audit['mismatched_sha'] or audit['duplicate_packages']:raise SystemExit('dependency audit blocked')
sys.path.insert(0,str(REPO))
from workbench.profiles import list_profiles
selected_profiles={p['robot']:p for p in list_profiles(ROOT)}
versions={}
for robot in ('bunker','fwmini'):
    versions[robot]={'profile':selected_profiles[robot],'calibrations':[{'file':str(p.relative_to(ROOT)),'sha256':sha(p)} for p in sorted((ROOT/'calibration'/robot).glob('*.yaml'))], 'field_map':'NOT_RUN','field_route':'NOT_RUN'}
now=datetime.now(timezone.utc);target=ROOT/'reports'/('integration_'+head+'_'+now.strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8]+'.json')
report={'integration_candidate_sha':head,'branch':git('branch','--show-current'),'generated_at':now.isoformat(),
        'build_environment':(evidence/'environment.txt').read_text(),'dependencies':audit,
        'robot_versions':versions,'evidence_sha256':{str(p.relative_to(REPO)):sha(p) for p in sorted(evidence.iterdir()) if p.is_file()},
        'software':'PASS','real_bag_benchmark':'NOT_RUN','bunker_motion_diagnostic':'NOT_RUN','GPS_hardware_constraints':'NOT_RUN',
        'field_accuracy_5_7_10m':'NOT_RUN','camera_waypoints':'NOT_RUN','recognition_map50':'NOT_RUN','formal_document_comparison':'NOT_RUN',
        'autonomous_navigation_enabled':False,'fwmini_lateral_autonomous':'BLOCKED','GPS_graph_optimization':'NOT_RUN',
        'manual':'docs/workbench_operator_manual.md','delivery':'docs/field_loop_delivery.md'}
target.parent.mkdir(parents=True,exist_ok=True)
with target.open('x') as f:json.dump(report,f,indent=2,ensure_ascii=False)
print(target)
