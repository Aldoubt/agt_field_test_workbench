"""Physical evidence is separate from operator setup and software observations."""
import hashlib,json
from pathlib import Path
CHECKS=('physical_estop','remote_takeover','command_timeout','feedback_freshness','geometry','localization','tf_ownership','motion_safety')
SETUP=('wiring','physical_estop','remote_takeover','clear_area','supervised')

def setup(value):
    if not isinstance(value,dict) or any(value.get(k) is not True for k in SETUP):
        raise ValueError('confirm wiring, physical E-stop, remote takeover, clear area and supervision')
    return value

def report(path,robot,profile_sha,map_sha,calibration_sha):
    path=Path(path).expanduser().resolve();r=json.loads(path.read_text())
    if (r.get('scope')!='PHYSICAL_NAVIGATION' or r.get('robot')!=robot or r.get('profile_sha256')!=profile_sha
        or r.get('map_sha256')!=map_sha or r.get('calibration_sha256')!=calibration_sha
        or not r.get('operator') or not r.get('observed_at') or not r.get('observations')
        or any(r.get('checks',{}).get(k)!='PASS' for k in CHECKS)):
        raise ValueError('navigation requires matching PHYSICAL_NAVIGATION report and all safety checks PASS')
    return dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),data=r)
