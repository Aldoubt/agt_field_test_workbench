"""Exercise reviewed patch succession with local Git fixtures and no dependencies."""
import contextlib
import difflib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch


REPO = Path(__file__).resolve().parents[1]
MANIFESTS = ('control_manifest.json', 'sc_kiss_manifest.json',
             'relocalization_self_filter_manifest.json')


def digest(data):
    return hashlib.sha256(data.encode()).hexdigest()


@unittest.skipUnless(shutil.which('git'), 'Git is required for the patch fixture')
class PatchSuccessorTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.base = Path(temporary.name)
        self.workspace = self.base / 'workspace'
        self.fixture = self.base / 'reviewed'
        (self.fixture / 'patches').mkdir(parents=True)
        spec = importlib.util.spec_from_file_location(
            'apply_control_fixture', REPO / 'scripts/apply_control_integration.py')
        self.applicator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.applicator)
        self.applicator.REPO = self.fixture
        self.revisions = [
            {'launch.py': "backend = 'base'\n", 'control.py': 'guard = False\n',
             'query.py': "descriptor = 'none'\n"},
            {'launch.py': "backend = 'control'\n", 'control.py': 'guard = True\n',
             'query.py': "descriptor = 'none'\n"},
            {'launch.py': "backend = 'sc'\n", 'control.py': 'guard = True\n',
             'query.py': "descriptor = 'sc'\n"},
            {'launch.py': "backend = 'sc_with_self_mask'\n", 'control.py': 'guard = True\n',
             'query.py': "descriptor = 'sc_with_self_mask'\n"},
        ]
        self.dependencies = {}
        self.heads = {}
        manifests = [{} for _ in MANIFESTS]
        for relative in ('first_dependency', 'later_dependency'):
            root = self.workspace / 'src' / relative
            root.mkdir(parents=True)
            self.dependencies[relative] = root
            self.git(root, 'init', '-q')
            for name, contents in self.revisions[0].items():
                (root / name).write_text(contents)
            (root / 'operator-notes.txt').write_text('original notes\n')
            self.git(root, 'add', '.')
            self.git(root, '-c', 'user.name=Patch Fixture', '-c',
                     'user.email=patch-fixture@example.invalid', '-c', 'commit.gpgsign=false',
                     'commit', '-qm', 'pinned fixture')
            self.heads[relative] = self.git(root, 'rev-parse', 'HEAD').strip()
            # These unrelated edits must survive every patch and validation run.
            (root / 'operator-notes.txt').write_text('operator notes edited locally\n')
            (root / 'untracked-notes.txt').write_text('untracked local work\n')
            for stage, manifest in enumerate(manifests):
                before, after = self.revisions[stage:stage + 2]
                changed = [name for name in before if before[name] != after[name]]
                patch_name = f'patches/{stage}-{relative}.patch'
                lines = []
                for name in changed:
                    lines.append(f'diff --git a/{name} b/{name}\n')
                    lines.extend(difflib.unified_diff(
                        before[name].splitlines(keepends=True),
                        after[name].splitlines(keepends=True),
                        fromfile='a/' + name, tofile='b/' + name))
                (self.fixture / patch_name).write_text(''.join(lines))
                manifest[relative] = dict(commit=self.heads[relative], patch=patch_name,
                    files={name: dict(before=digest(before[name]), after=digest(after[name]))
                           for name in changed})
        for name, manifest in zip(MANIFESTS, manifests):
            (self.fixture / 'patches' / name).write_text(json.dumps(manifest))

    @staticmethod
    def git(root, *arguments):
        return subprocess.check_output(
            ['git', '-C', str(root), *arguments], text=True, stderr=subprocess.PIPE)

    def apply(self, stage, check=False):
        arguments = ['apply_control_integration.py', '--workspace', str(self.workspace)]
        # The first command also exercises the real default manifest lookup.
        # apply_sc_kiss_integration.py delegates to this same entry with --manifest.
        if stage:
            arguments.extend(['--manifest', str(self.fixture / 'patches' / MANIFESTS[stage])])
        if check:
            arguments.append('--check')
        with patch.object(sys, 'argv', arguments), contextlib.redirect_stdout(io.StringIO()):
            self.applicator.main()

    def snapshot(self):
        return {(relative, str(path.relative_to(root))): path.read_bytes()
                for relative, root in self.dependencies.items()
                for path in root.rglob('*')
                if path.is_file() and '.git' not in path.relative_to(root).parts}

    def assert_heads_preserved(self):
        for relative, root in self.dependencies.items():
            self.assertEqual(self.git(root, 'rev-parse', 'HEAD').strip(), self.heads[relative])

    def install_chain(self):
        for stage in range(len(MANIFESTS)):
            self.apply(stage)

    def test_control_sc_self_mask_sequence_and_repeated_apply_check_preserve_local_work(self):
        original = self.snapshot()
        for stage in range(len(MANIFESTS)):
            before = self.snapshot()
            self.apply(stage, check=True)
            self.assertEqual(self.snapshot(), before)
            self.apply(stage)
            for root in self.dependencies.values():
                for name, expected in self.revisions[stage + 1].items():
                    self.assertEqual((root / name).read_text(), expected)
            self.assert_heads_preserved()
        final = self.snapshot()
        for key, content in original.items():
            if key[1] not in self.revisions[0]:
                self.assertEqual(final[key], content)
        for stage in range(len(MANIFESTS)):
            for check in (False, True):
                with self.subTest(stage=stage, check=check):
                    self.apply(stage, check=check)
                    self.assertEqual(self.snapshot(), final)
                    self.assert_heads_preserved()

    def test_unknown_edit_in_later_dependency_rejects_before_any_patch_is_written(self):
        path = self.dependencies['later_dependency'] / 'launch.py'
        path.write_text(path.read_text() + '# operator edit before installation\n')
        before = self.snapshot()
        for check in (False, True):
            with self.subTest(check=check):
                with self.assertRaisesRegex(SystemExit, 'differs.*preserved'):
                    self.apply(0, check=check)
                self.assertEqual(self.snapshot(), before)
                self.assert_heads_preserved()
        self.assertEqual((self.dependencies['first_dependency'] / 'control.py').read_text(),
                         self.revisions[0]['control.py'])

    def test_unknown_edit_after_successor_is_rejected_by_every_manifest(self):
        self.install_chain()
        path = self.dependencies['later_dependency'] / 'launch.py'
        path.write_text(path.read_text() + '# operator edit after installation\n')
        before = self.snapshot()
        for stage in range(len(MANIFESTS)):
            for check in (False, True):
                with self.subTest(stage=stage, check=check):
                    with self.assertRaisesRegex(SystemExit, 'differs.*preserved'):
                        self.apply(stage, check=check)
                    self.assertEqual(self.snapshot(), before)
                    self.assert_heads_preserved()

    def test_reviewed_but_unreachable_successor_hash_is_rejected(self):
        self.install_chain()
        sc_manifest = self.fixture / 'patches' / MANIFESTS[1]
        spec = json.loads(sc_manifest.read_text())
        spec['later_dependency']['files']['launch.py']['before'] = digest('unrelated source\n')
        sc_manifest.write_text(json.dumps(spec))
        before = self.snapshot()
        with self.assertRaisesRegex(SystemExit, 'differs.*preserved'):
            self.apply(0)
        self.assertEqual(self.snapshot(), before)
        self.assert_heads_preserved()

    def test_successor_from_another_pinned_commit_is_rejected(self):
        self.apply(0)
        self.apply(1)
        sc_manifest = self.fixture / 'patches' / MANIFESTS[1]
        spec = json.loads(sc_manifest.read_text())
        spec['later_dependency']['commit'] = '0' * 40
        sc_manifest.write_text(json.dumps(spec))
        before = self.snapshot()
        with self.assertRaisesRegex(SystemExit, 'differs.*preserved'):
            self.apply(0)
        self.assertEqual(self.snapshot(), before)
        self.assert_heads_preserved()

    def test_real_self_mask_manifest_starts_at_the_reviewed_sc_successor(self):
        sc = json.loads((REPO / 'patches' / MANIFESTS[1]).read_text())
        mask = json.loads((REPO / 'patches' / MANIFESTS[2]).read_text())
        self.assertTrue(mask)
        for relative, successor in mask.items():
            self.assertEqual(successor['commit'], sc[relative]['commit'])
            self.assertTrue(successor['files'])
            for name, hashes in successor['files'].items():
                with self.subTest(dependency=relative, file=name):
                    self.assertEqual(hashes['before'], sc[relative]['files'][name]['after'])


if __name__ == '__main__':
    unittest.main()
