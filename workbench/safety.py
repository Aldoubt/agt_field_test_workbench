"""Physical evidence is separate from operator setup and software observations."""
import hashlib,json
from pathlib import Path
CHECKS=('physical_estop','remote_takeover','command_timeout','feedback_freshness','geometry','localization','tf_ownership','motion_safety')
SETUP=('wiring','physical_estop','remote_takeover','clear_area','supervised')

def setup(value):
    if not isinstance(value,dict) or any(value.get(k) is not True for k in SETUP):
        raise ValueError('confirm wiring, physical E-stop, remote takeover, clear area and supervision')
    return value

def report(path,robot,profile_sha,map_sha,calibration_sha,compatible_map_sha256s=()):
    path=Path(path).expanduser().resolve();r=json.loads(path.read_text())
    compatible={value for value in (compatible_map_sha256s or ()) if value}
    accepted_map_shas={map_sha}|compatible
    if (r.get('scope')!='PHYSICAL_NAVIGATION' or r.get('robot')!=robot or
        r.get('profile_sha256')!=profile_sha or r.get('map_sha256') not in accepted_map_shas or
        r.get('calibration_sha256')!=calibration_sha or not r.get('operator') or
        not r.get('observed_at') or not r.get('observations') or
        any(r.get('checks',{}).get(k)!='PASS' for k in CHECKS)):
        raise ValueError('navigation requires matching PHYSICAL_NAVIGATION evidence and all safety checks PASS')
    return dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),data=r)
