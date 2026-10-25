"""Research database preflight; no change to the existing BBS asset selection."""
import json
from pathlib import Path
from .maps import hash_file


def validate_sc_database(value,map_record):
    if not value or not map_record:raise ValueError('sc_kiss requires an explicit database and selected map')
    directory=Path(value).expanduser().resolve();manifest=json.loads((directory/'manifest.json').read_text())
    expected={'map_sha256':map_record.get('source_pcd_sha256'),
              'navigation_yaml_sha256':hash_file(Path(map_record['yaml'])),
              'navigation_pgm_sha256':map_record['sha256']}
    if manifest.get('algorithm')!='scancontext' or any(manifest.get(k)!=v for k,v in expected.items()):
        raise ValueError('Scan Context database does not match selected PCD/navigation map identity')
    if hash_file(directory/'descriptors.npz')!=manifest.get('descriptors_sha256'):
        raise ValueError('Scan Context descriptors SHA mismatch')
    return str(directory)
