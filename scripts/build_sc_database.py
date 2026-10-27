#!/usr/bin/env python3
"""Build from existing keyframes or exact FAST-LIO spool; never infer scans from fused PCD."""
import argparse
from pathlib import Path
import json
import sys
REPO=Path(__file__).resolve().parents[1];sys.path.insert(0,str(REPO))


def main():
    from agt_sc_kiss.database import build
    from agt_sc_kiss.geometry import contract_from_lio
    p=argparse.ArgumentParser();p.add_argument('--map',required=True);p.add_argument('--map-yaml',required=True)
    p.add_argument('--lio-config',required=True);p.add_argument('--output',required=True)
    p.add_argument('--query-self-filter-config',default='',help='same explicit base_link mask JSON used by localization queries')
    group=p.add_mutually_exclusive_group(required=True);group.add_argument('--keyframes');group.add_argument('--artifact')
    a=p.parse_args();keyframes=a.keyframes
    if a.artifact:
        from workbench.map_cleanup import prepare_keyframes
        output=Path(a.output);keyframes=output.with_name(output.name+'_keyframes')
        prepare_keyframes(a.artifact,keyframes)
    value=build(a.map,a.map_yaml,keyframes,a.output,contract_from_lio(a.lio_config,a.query_self_filter_config))
    print(json.dumps(dict(database=str(Path(a.output).resolve()),keyframes=len(value['entries']),map_sha256=value['map_sha256'])))

if __name__=='__main__':main()
