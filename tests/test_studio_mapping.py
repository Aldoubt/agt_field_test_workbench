from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class StudioMappingContractTests(unittest.TestCase):
    def test_fastlio_is_default_and_glim_is_explicitly_experimental(self):
        source = (ROOT / 'agt_robot_studio/agt_robot_studio/ui.py').read_text()
        self.assertIn("'mapping_backend':'fastlio'", source)
        self.assertIn('FAST-LIO2 无 PGO（Bunker 当前验收基线）', source)
        self.assertIn('GLIM（实验：依赖／配置／真实回放未验收）', source)
        self.assertIn("'/api/mapping/start'", source)
        self.assertIn("f'/api/jobs/{self.mapping_job_id}/finish'", source)

    def test_fastlio_uses_workbench_not_a_second_qt_process(self):
        source = (ROOT / 'agt_robot_studio/agt_robot_studio/ui.py').read_text()
        fast_path = source.split('def _start_mapping_processes(self):', 1)[1].split(
            'def toggle_glim(self):', 1)[0]
        self.assertIn("'backend':'fastlio'", fast_path)
        self.assertNotIn("self._proc('ros2'", fast_path)
        self.assertNotIn('livox_ros_driver2', fast_path)
        self.assertNotIn('robot_state_publisher', fast_path)
        self.assertNotIn("['bag','record'", fast_path)

    def test_mapping_name_reaches_pipeline_and_cloud_manifest(self):
        runtime = (ROOT / 'workbench/runtime.py').read_text()
        pipeline = (ROOT / 'scripts/mapping_pipeline.py').read_text()
        self.assertIn("'--map-name'", runtime)
        self.assertIn("name=args.map_name", pipeline)


if __name__ == '__main__':
    unittest.main()
