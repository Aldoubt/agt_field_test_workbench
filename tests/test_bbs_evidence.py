"""Exercise the shipped attempt archive without ROS or a localization backend."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess

import pytest


REPO = Path(__file__).resolve().parents[1]
MODULE_PATH = ('navigation/localization/agt_global_relocalization/'
               'agt_global_relocalization/attempt_evidence.py')


def _module_source():
    override = os.environ.get('BBS_EVIDENCE_SOURCE')
    if override:
        return Path(override).read_text(encoding='utf-8')
    # Tests exercise the exact added module that deployments apply to the
    # pinned dependency, without requiring that dependency checkout locally.
    for patch in sorted((REPO / 'patches').glob('bbs*.patch')):
        active = False
        lines = []
        for line in patch.read_text(encoding='utf-8').splitlines(keepends=True):
            if line.startswith('diff --git '):
                if active:
                    break
                active = line.rstrip().endswith(' b/' + MODULE_PATH)
                continue
            if active and line.startswith('+') and not line.startswith('+++'):
                lines.append(line[1:])
        if lines:
            return ''.join(lines)
    raise AssertionError('The BBS patch must include the new attempt_evidence.py module')


@pytest.fixture
def evidence(tmp_path):
    source = tmp_path / 'attempt_evidence.py'
    source.write_text(_module_source(), encoding='utf-8')
    spec = importlib.util.spec_from_file_location('bbs_attempt_evidence', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _record(attempt):
    return json.loads((attempt.path / 'attempt.json').read_text(encoding='utf-8'))


def test_exact_query_and_success_metadata_are_saved_atomically(evidence, tmp_path):
    query = b'VERSION .7\nFIELDS x y z\nDATA ascii\n1 2 3\n'
    with evidence.AttemptEvidence(tmp_path / 'runs') as attempt:
        (attempt.path / 'query_scan.pcd').write_bytes(query)
        attempt.update(map_id='orchard', query_frame='body', points=1)
        assert _record(attempt)['completed'] is False
        attempt.record_backend(subprocess.CompletedProcess(['matcher'], 0, '{"success":true}', ''))
    record = _record(attempt)
    assert record['schema'] == evidence.SCHEMA
    assert record['completed'] is True
    assert record['outcome'] == 'success'
    assert record['map_id'] == 'orchard'
    assert record['query_sha256'] == hashlib.sha256(query).hexdigest()
    assert record['elapsed_sec'] >= 0
    assert record['started_at'].endswith('+00:00')
    assert record['completed_at'].endswith('+00:00')
    assert not list(attempt.path.glob('.*.tmp'))


def test_backend_return_four_preserves_original_failure_and_output(evidence, tmp_path):
    failure = RuntimeError('backend returned 4: GICP did not converge')
    with pytest.raises(RuntimeError) as raised:
        with evidence.AttemptEvidence(tmp_path) as attempt:
            (attempt.path / 'query_scan.pcd').write_bytes(b'exact rejected query')
            attempt.record_backend(stdout='{"success":false}', stderr='candidate 1 rejected', returncode=4)
            raise failure
    assert raised.value is failure
    record = _record(attempt)
    assert record['completed'] is True
    assert record['outcome'] == 'rejected'
    assert record['backend_returncode'] == 4
    assert record['exception_type'] == 'RuntimeError'
    assert (attempt.path / 'backend.stdout').read_text() == '{"success":false}'
    assert (attempt.path / 'backend.stderr').read_text() == 'candidate 1 rejected'
    assert record['query_sha256'] == hashlib.sha256(b'exact rejected query').hexdigest()


def test_exception_without_process_result_is_retained(evidence, tmp_path):
    with pytest.raises(ValueError, match='invalid transform'):
        with evidence.AttemptEvidence(tmp_path) as attempt:
            raise ValueError('invalid transform')
    record = _record(attempt)
    assert record['completed'] is True
    assert record['outcome'] == 'error'
    assert record['exception_message'] == 'invalid transform'


def test_timeout_preserves_partial_output_and_query(evidence, tmp_path):
    timeout = subprocess.TimeoutExpired(['matcher'], 18, output=b'candidate trace', stderr=b'time limit')
    with pytest.raises(subprocess.TimeoutExpired) as raised:
        with evidence.AttemptEvidence(tmp_path) as attempt:
            (attempt.path / 'query_scan.pcd').write_bytes(b'timed out query')
            raise timeout
    assert raised.value is timeout
    record = _record(attempt)
    assert record['outcome'] == 'error'
    assert record['exception_type'] == 'TimeoutExpired'
    assert record['backend_returncode'] is None
    assert (attempt.path / 'backend.stdout').read_text() == 'candidate trace'
    assert (attempt.path / 'backend.stderr').read_text() == 'time limit'
    assert record['query_sha256'] == hashlib.sha256(b'timed out query').hexdigest()


def test_retention_only_removes_old_completed_owned_directories(evidence, tmp_path):
    root = tmp_path / 'runs'
    root.mkdir()

    def directory(name, record, mtime):
        path = root / name
        path.mkdir()
        (path / 'attempt.json').write_text(json.dumps(record))
        os.utime(path, (mtime, mtime))
        return path

    complete = dict(schema=evidence.SCHEMA, completed=True)
    oldest = directory('attempt_oldest', complete, 1)
    recent = directory('attempt_recent', complete, 2)
    unfinished = directory('attempt_running', dict(schema=evidence.SCHEMA, completed=False), 1)
    foreign = directory('attempt_foreign', dict(schema='foreign', completed=True), 1)
    malformed = directory('attempt_bad', complete, 1)
    (malformed / 'attempt.json').write_text('{broken')
    other = directory('other_results', complete, 1)
    list_record = directory('attempt_list', [], 1)
    outside = tmp_path / 'outside'
    outside.mkdir()
    (outside / 'attempt.json').write_text(json.dumps(complete))
    symlink = root / 'attempt_link'
    symlink.symlink_to(outside, target_is_directory=True)
    metadata_link = root / 'attempt_metadata_link'
    metadata_link.mkdir()
    (metadata_link / 'attempt.json').symlink_to(outside / 'attempt.json')

    with evidence.AttemptEvidence(root, keep=2) as attempt:
        pass

    assert not oldest.exists()
    assert recent.is_dir()
    assert attempt.path.is_dir()
    for preserved in (unfinished, foreign, malformed, other, list_record, outside, metadata_link):
        assert preserved.is_dir()
    assert symlink.is_symlink()
    assert (outside / 'attempt.json').is_file()


@pytest.mark.parametrize('fail_matching', [False, True])
def test_cleanup_error_never_replaces_matching_outcome(evidence, tmp_path, monkeypatch, caplog, fail_matching):
    def denied(_self):
        raise PermissionError('archive busy')

    monkeypatch.setattr(evidence.AttemptEvidence, '_prune', denied)
    failure = RuntimeError('matching failed')
    if fail_matching:
        with pytest.raises(RuntimeError) as raised:
            with evidence.AttemptEvidence(tmp_path) as attempt:
                raise failure
        assert raised.value is failure
    else:
        with evidence.AttemptEvidence(tmp_path) as attempt:
            pass
    assert _record(attempt)['outcome'] == ('error' if fail_matching else 'success')
    assert 'Could not prune localization evidence: archive busy' in caplog.text


def test_invalid_metadata_does_not_poison_exception_record(evidence, tmp_path):
    with pytest.raises(ValueError):
        with evidence.AttemptEvidence(tmp_path) as attempt:
            attempt.update(fitness=float('nan'))
    record = _record(attempt)
    assert record['completed'] is True
    assert record['outcome'] == 'error'
    assert 'fitness' not in record


@pytest.mark.parametrize('failure_stage', ['hash', 'save'])
def test_final_archive_failure_does_not_reject_an_already_published_pose(
        evidence, tmp_path, monkeypatch, caplog, failure_stage):
    published = []
    with evidence.AttemptEvidence(tmp_path) as attempt:
        (attempt.path / 'query_scan.pcd').write_bytes(b'accepted query')
        attempt.record_backend(returncode=0, stdout='{"success":true}')
        published.append('accepted pose')

        def denied(*_args, **_kwargs):
            raise OSError('archive unavailable')

        if failure_stage == 'hash':
            monkeypatch.setattr(evidence.hashlib, 'sha256', denied)
        else:
            monkeypatch.setattr(attempt, '_save', denied)
    assert published == ['accepted pose']
    assert 'Could not finalize localization evidence: archive unavailable' in caplog.text
    assert _record(attempt)['completed'] is False


def test_distinct_attempts_and_lifecycle_fields_are_protected(evidence, tmp_path):
    with evidence.AttemptEvidence(tmp_path) as first:
        with pytest.raises(ValueError, match='lifecycle'):
            first.update(completed=True)
    with evidence.AttemptEvidence(tmp_path) as second:
        assert first.path != second.path
    with pytest.raises(RuntimeError, match='active context'):
        second.update(points=3)
