"""Explicit, immutable self-filter configuration for localization queries only."""
import json
from pathlib import Path

from .maps import hash_file


def describe_query_filter(value, root=None):
    value = str(value or '').strip()
    empty = dict(query_self_filter_config='', query_self_filter_source='',
                 query_self_filter_sha256='', query_self_filter=None)
    if not value:
        return empty
    source = Path(value).expanduser()
    if not source.is_absolute() and root is not None:
        source = Path(root) / source
    source = source.resolve()
    from agt_field_test_bringup.relocalization_self_filter import load_self_filter
    before = hash_file(source)
    spec = load_self_filter(source)
    if hash_file(source) != before:
        raise ValueError('query self-filter changed while reading')
    return dict(query_self_filter_config=str(source), query_self_filter_source=str(source),
                query_self_filter_sha256=before, query_self_filter=spec)


def freeze_query_filter(value, directory, root=None, *, expected_sha256=None):
    record = describe_query_filter(value, root)
    if expected_sha256 is not None and record['query_self_filter_sha256'] != expected_sha256:
        raise ValueError('query self-filter changed after the operation was requested')
    if not record['query_self_filter_config']:
        return record
    source = Path(record['query_self_filter_config'])
    target = Path(directory) / 'query_self_filter.json'
    data = source.read_bytes()
    # A new cloud/session owns its snapshot; never replace another version.
    with target.open('xb') as stream:
        stream.write(data)
    if (hash_file(target) != record['query_self_filter_sha256']
            or hash_file(source) != record['query_self_filter_sha256']):
        raise ValueError('query self-filter changed while freezing configuration')
    record['query_self_filter_config'] = str(target.resolve())
    return record


def query_filter_matches(current, requested):
    if (current or {}).get('query_self_filter_sha256', '') != requested['query_self_filter_sha256']:
        return False
    requested_path=requested.get('query_self_filter_config')
    if requested_path:
        try:
            if hash_file(requested_path)!=requested['query_self_filter_sha256']:return False
        except OSError:return False
    frozen = (current or {}).get('query_self_filter_config')
    if frozen:
        try:
            return hash_file(frozen) == current.get('query_self_filter_sha256')
        except OSError:
            return False
    return not requested['query_self_filter_sha256']


def validate_sc_query_filter(database, record):
    """Reject both filtered-query/unfiltered-DB and the inverse mismatch."""
    from agt_field_test_bringup.relocalization_self_filter import validate_self_filter_spec
    manifest = json.loads((Path(database) / 'manifest.json').read_text())
    actual = manifest.get('preprocessing', {}).get('self_filter')
    actual = validate_self_filter_spec(actual) if actual is not None else None
    expected = record.get('query_self_filter')
    if actual != expected:
        raise ValueError('Scan Context query self-filter differs; rebuild the database with the same configuration')
    return str(database)


def validate_bbs_query_filter(directory, record):
    """Bind Polar/BBS assets and the live query to the same body-mask contract."""
    status_path = Path(directory) / 'asset_status.json'
    status = json.loads(status_path.read_text()) if status_path.is_file() else {}
    actual = status.get('query_self_filter')
    if actual is not None:
        from agt_field_test_bringup.relocalization_self_filter import validate_self_filter_spec
        actual = validate_self_filter_spec(actual)
    elif status.get('query_self_filter_sha256'):
        raise ValueError('BBS assets have an incomplete query self-filter contract; rebuild assets')
    if actual != record.get('query_self_filter'):
        raise ValueError('Polar Context/BBS query self-filter differs; select the map configuration or rebuild assets')
    return str(directory)
