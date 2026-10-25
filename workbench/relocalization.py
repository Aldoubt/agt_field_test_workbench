"""Scan Context preflight independent of occupancy-grid pixel hashes."""
import hashlib
import json
import math
from pathlib import Path

import yaml

from .maps import hash_file


def _metadata_sha(value):
    return hashlib.sha256(json.dumps(
        {k:v for k,v in value.items() if k!='metadata_sha256'},
        sort_keys=True,allow_nan=False).encode()).hexdigest()


def _same_geometry(left, right):
    try:
        if not math.isclose(float(left['resolution']),float(right['resolution']),rel_tol=0,abs_tol=1e-9):
            return False
        a,b=left.get('origin',[]),right.get('origin',[])
        return len(a)==3 and len(b)==3 and all(
            math.isclose(float(x),float(y),rel_tol=0,abs_tol=1e-8) for x,y in zip(a,b))
    except (KeyError,TypeError,ValueError):
        return False


def validate_sc_database(value,map_record):
    """Validate SC against source PCD, preprocessing provenance and map-frame geometry."""
    if not value or not map_record:
        raise ValueError('sc_kiss requires an explicit database and selected map')
    directory=Path(value).expanduser().resolve()
    manifest=json.loads((directory/'manifest.json').read_text())
    if manifest.get('metadata_sha256')!=_metadata_sha(manifest):
        raise ValueError('Scan Context metadata SHA mismatch')
    if (manifest.get('algorithm')!='scancontext' or
            manifest.get('map_sha256')!=map_record.get('source_pcd_sha256')):
        raise ValueError('Scan Context database does not match selected localization PCD identity')
    descriptor_path=directory/'descriptors.npz'
    if not descriptor_path.is_file() or hash_file(descriptor_path)!=manifest.get('descriptors_sha256'):
        raise ValueError('Scan Context descriptors SHA mismatch')
    built_yaml=Path(manifest.get('navigation_yaml','')).expanduser()
    if not built_yaml.is_file() or not manifest.get('navigation_yaml_sha256'):
        raise ValueError('Scan Context build-time navigation YAML is missing or unhashed')
    if hash_file(built_yaml)!=manifest['navigation_yaml_sha256']:
        raise ValueError('Scan Context build-time navigation YAML SHA mismatch')
    built_meta=yaml.safe_load(built_yaml.read_text())
    built_pgm=(built_yaml.parent/built_meta['image']).resolve()
    if not built_pgm.is_file() or not manifest.get('navigation_pgm_sha256'):
        raise ValueError('Scan Context build-time navigation PGM is missing or unhashed')
    if hash_file(built_pgm)!=manifest['navigation_pgm_sha256']:
        raise ValueError('Scan Context build-time navigation PGM SHA mismatch')
    current_meta=yaml.safe_load(Path(map_record['yaml']).read_text())
    if not _same_geometry(built_meta,current_meta):
        raise ValueError('Scan Context database map-frame geometry differs; rebuild after changing grid origin/resolution')
    return str(directory)
