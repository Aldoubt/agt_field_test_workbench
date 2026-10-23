#!/usr/bin/env python3
"""Read-only host/PTP evidence; it never configures clocks or declares calibration."""
import argparse,json,re,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--interface',default='eth0');p.add_argument('--output',required=True);a=p.parse_args()
if not re.fullmatch(r'[A-Za-z0-9_.:-]{1,40}',a.interface):raise ValueError('invalid interface')
commands={'clock':['timedatectl','show'],'ptp':['pmc','-u','-b','0','GET','TIME_STATUS_NP'],
          'phc2sys':['pgrep','-a','phc2sys'],'hardware_timestamping':['ethtool','-T',a.interface]}
report={'state':'NOT_RUN','physical_time_sync':'NOT_RUN','observations':{}}
for name,cmd in commands.items():
    try:
        r=subprocess.run(cmd,capture_output=True,text=True,timeout=4)
        report['observations'][name]={'state':'OBSERVED' if r.returncode==0 else 'WARN','output':(r.stdout+r.stderr)[-16000:]}
    except (OSError,subprocess.TimeoutExpired) as e:report['observations'][name]={'state':'NOT_RUN','reason':str(e)}
report['required']='compare raw/aligned/algorithm headers during stationary, spin, straight and figure-eight replay; inspect PTP offset and drift'
target=Path(a.output);target.parent.mkdir(parents=True,exist_ok=True)
with target.open('x') as f:json.dump(report,f,indent=2)
