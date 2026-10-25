"""User-scoped installation and launcher tests; fake ROS never starts devices."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[1]
LAUNCH = REPO / 'scripts' / 'launch_studio.sh'
INSTALL = REPO / 'scripts' / 'install_studio_desktop.sh'


class StudioDesktopTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='agt studio ')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.workspace = self.root / "field robot's workspace"
        (self.workspace / 'install').mkdir(parents=True)
        (self.workspace / 'install' / 'setup.bash').write_text('export AGT_TEST_OVERLAY=loaded\n')
        self.ros_setup = self.root / 'ros setup.bash'
        self.ros_setup.write_text('export AGT_TEST_ROS=loaded\n')
        self.bin = self.root / 'user bin'
        self.data = self.root / 'user data'
        self.env = dict(os.environ, XDG_DATA_HOME=str(self.data),
                        XDG_STATE_HOME=str(self.root / 'state'),
                        AGT_STUDIO_ROS_SETUP=str(self.ros_setup))
        self.fakebin = self.root / 'fake bin'
        self.fakebin.mkdir()
        fake_ros = self.fakebin / 'ros2'
        fake_ros.write_text('#!' + sys.executable + '\n' + '''import json, os, sys
from pathlib import Path
if sys.argv[1:] == ['pkg', 'prefix', 'agt_robot_studio']:
    print(os.environ['AGT_FIELD_TEST_WS'] + '/install/agt_robot_studio')
elif sys.argv[1:4] == ['run', 'agt_robot_studio', 'agt_robot_studio']:
    Path(os.environ['AGT_TEST_OUTPUT']).write_text(json.dumps({
        'args': sys.argv[1:], 'workspace': os.environ['AGT_FIELD_TEST_WS'],
        'ros': os.environ.get('AGT_TEST_ROS'), 'overlay': os.environ.get('AGT_TEST_OVERLAY'),
        'cwd': os.getcwd()}))
else:
    sys.exit(7)
''')
        fake_ros.chmod(0o755)
        # Only launcher dependency probes use this stub; installer uses real Python.
        fake_python = self.fakebin / 'python3'
        fake_python.write_text('#!/usr/bin/env bash\n[[ "$1" == -c ]] || exit 9\nexit "${AGT_TEST_IMPORT_EXIT:-0}"\n')
        fake_python.chmod(0o755)
        self.output = self.root / 'ros invocation.json'
        self.launch_env = dict(self.env, PATH=str(self.fakebin) + os.pathsep + self.env['PATH'],
                               AGT_TEST_OUTPUT=str(self.output))

    def run_script(self, path, *args, env=None):
        return subprocess.run(['bash', str(path), *map(str, args)], env=env or self.env,
                              text=True, capture_output=True, timeout=10)

    def install(self):
        return self.run_script(INSTALL, '--workspace', self.workspace, '--bin-dir', self.bin)

    def test_help_does_not_require_ros_or_workspace(self):
        for path in (LAUNCH, INSTALL):
            result = self.run_script(path, '--help', env=dict(self.env, AGT_FIELD_TEST_WS='/nonexistent'))
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('Usage:', result.stdout)
        self.assertFalse(self.output.exists())

    def test_install_only_creates_user_shortcuts_and_preserves_configuration(self):
        config = self.workspace / 'config' / 'robot.yaml'
        config.parent.mkdir()
        config.write_text('robot: original\n')
        result = self.install()
        self.assertEqual(result.returncode, 0, result.stderr)
        launcher = self.bin / 'agt-robot-studio'
        desktop = self.data / 'applications' / 'agt-robot-studio.desktop'
        self.assertTrue(os.access(launcher, os.X_OK))
        self.assertIn('Terminal=false', desktop.read_text())
        self.assertIn(' --desktop --demo', desktop.read_text())
        self.assertEqual(config.read_text(), 'robot: original\n')
        self.assertFalse(self.output.exists())
        # Re-installing our own shortcuts is allowed and leaves data unchanged.
        self.assertEqual(self.install().returncode, 0)

    def test_launcher_preserves_spaces_quotes_arguments_and_sources_workspace(self):
        self.assertEqual(self.install().returncode, 0)
        config = self.root / "camera config 'quoted'.yaml"
        config.write_text('robot: bunker\n')
        result = self.run_script(self.bin / 'agt-robot-studio', '--demo', '--config', config,
                                 env=self.launch_env)
        self.assertEqual(result.returncode, 0, result.stderr)
        record = json.loads(self.output.read_text())
        self.assertEqual(record['args'], ['run', 'agt_robot_studio', 'agt_robot_studio',
                                         '--demo', '--config', str(config)])
        self.assertEqual(record['workspace'], str(self.workspace))
        self.assertEqual(record['cwd'], str(self.workspace))
        self.assertEqual((record['ros'], record['overlay']), ('loaded', 'loaded'))

    def test_environment_check_never_runs_application(self):
        result = self.run_script(LAUNCH, '--workspace', self.workspace, '--check', env=self.launch_env)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('Environment check passed', result.stdout)
        self.assertFalse(self.output.exists())

    def test_missing_environment_has_actionable_failure_and_no_ros_run(self):
        result = self.run_script(LAUNCH, '--workspace', self.workspace,
                                 env=dict(self.launch_env, AGT_STUDIO_ROS_SETUP=str(self.root / 'missing')))
        self.assertEqual(result.returncode, 2)
        self.assertIn('ROS 2 Humble', result.stderr)
        result = self.run_script(LAUNCH, '--workspace', self.root / 'missing', env=self.launch_env)
        self.assertEqual(result.returncode, 2)
        self.assertIn('工作区不存在', result.stderr)
        (self.workspace / 'install' / 'setup.bash').unlink()
        result = self.run_script(LAUNCH, '--workspace', self.workspace, env=self.launch_env)
        self.assertEqual(result.returncode, 2)
        self.assertIn('colcon build', result.stderr)
        self.assertFalse(self.output.exists())

    def test_failed_import_and_bad_argument_do_not_start_application(self):
        result = self.run_script(LAUNCH, '--workspace', self.workspace,
                                 env=dict(self.launch_env, AGT_TEST_IMPORT_EXIT='1'))
        self.assertEqual(result.returncode, 2)
        self.assertIn('Python/Qt', result.stderr)
        self.assertEqual(self.run_script(LAUNCH, '--workspace').returncode, 2)
        self.assertEqual(self.run_script(LAUNCH, '--not-an-option').returncode, 2)
        self.assertFalse(self.output.exists())

    def test_install_refuses_unrelated_existing_launcher(self):
        self.bin.mkdir()
        launcher = self.bin / 'agt-robot-studio'
        launcher.write_text('unrelated application\n')
        result = self.install()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('unrelated file', result.stderr)
        self.assertEqual(launcher.read_text(), 'unrelated application\n')

    def test_desktop_exec_escapes_percent_field_code_in_path(self):
        self.bin = self.root / 'user %f bin'
        result = self.install()
        self.assertEqual(result.returncode, 0, result.stderr)
        desktop = (self.data / 'applications' / 'agt-robot-studio.desktop').read_text()
        self.assertIn('user %%f bin', desktop)


if __name__ == '__main__':
    unittest.main()
