import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from workbench.session import Sessions, localization_matches_navigation_map


class SessionStartupTests(unittest.TestCase):
    def test_bridge_start_waits_for_ping_instead_of_requiring_second_click(self):
        class Jobs:
            def __init__(self): self.created = []
            def list(self): return []
            def _new(self, *args): self.created.append(args); return {'id':'bridge'}
        jobs = Jobs()
        sessions = Sessions(jobs, Path(tempfile.mkdtemp()))
        with patch('workbench.session.ros_ready', return_value=True), \
             patch('workbench.session.rpc', side_effect=[OSError('starting'), {'ready':True}]) as rpc:
            self.assertEqual(sessions.bridge(), {'ready':True})
            self.assertEqual(len(jobs.created), 1)
            self.assertEqual(rpc.call_count, 2)

    def test_stop_all_stops_owned_jobs_and_clears_session_pointer(self):
        class Jobs:
            def __init__(self): self.stopped = False
            def list(self): return []
            def stop_all(self): self.stopped = True
        jobs = Jobs()
        root = Path(tempfile.mkdtemp())
        (root/'work').mkdir()
        (root/'work/current_session.json').write_text('{}')
        sessions = Sessions(jobs, root)
        with patch('workbench.session.rpc', side_effect=OSError('already stopped')):
            self.assertEqual(sessions.stop_all(), [])
        self.assertTrue(jobs.stopped)
        self.assertFalse((root/'work/current_session.json').exists())

    def test_live_status_never_restarts_stopped_bridge(self):
        from unittest.mock import Mock
        from workbench.server import Handler
        handler = object.__new__(Handler)
        handler.path = '/api/live'
        handler.response = Mock()
        with patch('workbench.server.SESSIONS.bridge') as bridge, \
             patch('workbench.server.rpc', return_value={'ready':True}):
            handler.do_GET()
        bridge.assert_not_called()
        handler.response.assert_called_once_with(200, {'ready':True})


if __name__ == '__main__':
    unittest.main()


def test_navigation_reuses_localization_across_grid_only_revision():
    current={'task':'localization','robot':'bunker','localization_pcd_sha256':'pcd-sha',
             'global_backend':'sc_kiss','sc_database':'/assets/sc_db','lio_backend':'fastlio2'}
    revised_grid={'id':'new-grid-version','sha256':'new-pgm-sha','source_pcd_sha256':'pcd-sha'}
    assert localization_matches_navigation_map(
        current,'bunker',revised_grid,'sc_kiss','/assets/sc_db','fastlio2')
    assert not localization_matches_navigation_map(
        current,'bunker',dict(revised_grid,source_pcd_sha256='different-pcd'),
        'sc_kiss','/assets/sc_db','fastlio2')
    assert not localization_matches_navigation_map(
        current,'bunker',revised_grid,'bbs','/assets/sc_db','fastlio2')
    assert not localization_matches_navigation_map(
        current,'bunker',revised_grid,'sc_kiss','/assets/other_db','fastlio2')
