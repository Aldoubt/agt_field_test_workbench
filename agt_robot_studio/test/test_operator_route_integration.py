"""Operator route races exercised through the real window without robot I/O."""
from copy import deepcopy
import os
import time
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
try:
    from PyQt5.QtGui import QImage
    from PyQt5.QtWidgets import QApplication, QMessageBox
    from agt_robot_studio.ui import DEFAULT_CONFIG, Studio
except ImportError:
    QApplication = None


class FakeApi:
    """Keep queued payload references, as ApiWorker does, and defer local reads."""
    def __init__(self):
        self.calls = []
        self.local_calls = []

    def call(self, *args):
        self.calls.append(args)

    def local(self, name, callback):
        self.local_calls.append((name, callback))

    def requests(self, path):
        return [call for call in self.calls if call[2] == path]


@unittest.skipIf(QApplication is None, 'PyQt5 is not installed in this test environment')
class OperatorRouteIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = Studio(DEFAULT_CONFIG, demo=True)
        self.window.timer.stop()
        self.window.display_timer.stop()
        self.addCleanup(self.window.close)
        self.api = FakeApi()
        self.window.api = self.api
        self.window.demo = False
        self.maps = {
            ident: {'id': ident, 'name': ident, 'status': 'ACCEPTED',
                    'pgm': '/not-read-by-fake-api.pgm', 'width': 40,
                    'height': 40, 'resolution': .1, 'origin': [0., 0., 0.],
                    'asset_summary': 'Test map'}
            for ident in ('map-1', 'map-2', 'map-3')
        }
        self.window.maps_by_id = deepcopy(self.maps)
        self.window.nav_map_box.clear()
        for ident in self.maps:
            self.window.nav_map_box.addItem(ident, ident)
        self.window.current_map_id = 'map-1'
        self.window.displayed_map_id = 'map-1'
        self.window.live_status = {
            'session': {'id': 'session-1', 'robot': 'bunker',
                        'task': 'navigation', 'map_id': 'map-1'},
            'localized': True,
            'nav2_states': {'controller_server': 3, 'bt_navigator': 3},
            'feedback_fresh': [True, ''],
            'action_status': {'bridge_id':'bridge-1','recent':[]},
        }
        self.window.live_seen = time.monotonic()
        self.window.target_mode_box.setCurrentIndex(
            self.window.target_mode_box.findData('route'))
        warning_patch = patch('agt_robot_studio.ui.QMessageBox.warning',
                              return_value=QMessageBox.Ok)
        self.warning = warning_patch.start()
        self.addCleanup(warning_patch.stop)

    def add_point(self, x=1., y=2.):
        self.window.send_goal({'x': x, 'y': y, 'yaw': 0.})

    def save_response(self, ident='saved-route'):
        request = self.api.requests('/api/routes')[-1][3]
        route = {'id': ident, 'map_id': request['map_id'],
                 'points': deepcopy(request['points']), 'status': 'DRAFT'}
        self.window._on_api('route_save', True, route)
        return route

    def select_map(self, ident):
        self.window.nav_map_box.setCurrentIndex(self.window.nav_map_box.findData(ident))
        self.window.select_navigation_map()

    def map_response(self, ident):
        image = QImage(40, 40, QImage.Format_Grayscale8)
        image.fill(255)
        self.window._on_api('map_loaded', True, (self.maps[ident], image))

    def test_queued_save_is_immutable_while_operator_adds_and_edits_points(self):
        self.add_point()
        self.window.save_route()
        queued = self.api.requests('/api/routes')[0][3]
        before = deepcopy(queued)
        self.add_point(3., 3.)
        self.window.waypoint_editor.select_point(0)
        self.window.waypoint_editor.x_spin.setValue(1.75)
        self.assertEqual(queued, before)
        self.assertEqual(len(self.window.route_points), 2)
        self.assertEqual(self.window.route_points[0]['x'], 1.75)
        self.assertEqual(self.api.requests('/api/control/route'), [])

    def test_late_save_after_field_edit_does_not_make_visible_draft_executable(self):
        self.add_point()
        self.window.save_route()
        self.window.waypoint_editor.x_spin.setValue(2.5)
        self.save_response()
        self.window._update_operator_gate()
        self.window.run_route()
        self.assertIsNone(self.window.current_route_id)
        self.assertFalse(self.window.run_route_btn.isEnabled())
        self.assertEqual(self.window.route_points[0]['x'], 2.5)
        self.assertEqual(self.api.requests('/api/control/route'), [])
        self.assertTrue(self.window.save_route_btn.isEnabled())

    def test_late_save_after_clear_does_not_restore_deleted_route(self):
        self.add_point()
        self.window.save_route()
        self.window.clear_route()
        self.save_response()
        self.window.run_route()
        self.assertEqual(self.window.route_points, [])
        self.assertEqual(self.window.nav_view.waypoints, [])
        self.assertIsNone(self.window.current_route_id)
        self.assertEqual(self.api.requests('/api/control/route'), [])

    def test_late_save_does_not_replace_a_different_loaded_route(self):
        self.add_point()
        self.window.save_route()
        other = {'id': 'other-route', 'map_id': 'map-1',
                 'points': [{'x': 3., 'y': 3., 'yaw': .2, 'photo': False}]}
        self.window.routes_by_id = {other['id']: other}
        self.window.route_box.addItem('Other route', other['id'])
        self.window.load_route()
        self.save_response()
        self.assertEqual(self.window.current_route_id, 'other-route')
        self.assertEqual(self.window.route_points, other['points'])
        self.window.run_route()
        self.assertEqual(self.api.requests('/api/control/route')[-1][3]['route_id'],
                         'other-route')

    def test_saved_route_can_run_but_editor_change_requires_another_save(self):
        self.add_point()
        self.window.save_route()
        route = self.save_response()
        self.window._on_api('refresh_routes', True, [route])
        self.window._update_operator_gate()
        self.assertTrue(self.window.run_route_btn.isEnabled())
        self.window.run_route_btn.click()
        self.assertEqual(len(self.api.requests('/api/control/route')), 1)
        self.window._on_api('route_run', True, {'state': 'REQUESTED'})
        completed = deepcopy(self.window.live_status)
        completed['navigation_activity'] = {'pending': False, 'active': False}
        completed['mission'] = {'state': 6, 'point_count': 1, 'point_index': 1}
        request_id=self.api.requests('/api/control/route')[-1][3]['request_id']
        completed['action_status']={'bridge_id':'bridge-1','recent':[
            {'request_id':request_id,'action_id':'route-1','bridge_id':'bridge-1',
             'state':'SUCCEEDED','terminal':True,'seq':3}]}
        self.window._on_api('refresh_live', True, completed)
        self.window.waypoint_editor.heading_spin.setValue(-30.)
        self.window._update_operator_gate()
        self.window.run_route()
        self.assertIsNone(self.window.current_route_id)
        self.assertFalse(self.window.run_route_btn.isEnabled())
        self.assertEqual(len(self.api.requests('/api/control/route')), 1)

    def test_running_mission_keeps_route_visible_and_blocks_draft_mutation(self):
        self.add_point()
        self.window.save_route()
        self.save_response()
        before = deepcopy(self.window.route_points)
        active = deepcopy(self.window.live_status)
        active['navigation_activity'] = {'pending': False, 'active': True}
        active['mission'] = {'state': 2, 'point_count': 1, 'point_index': 1}
        self.window._on_api('refresh_live', True, active)
        self.assertFalse(self.window.run_route_btn.isEnabled())
        self.assertFalse(self.window.waypoint_editor.x_spin.isEnabled())
        self.window.clear_route()
        self.add_point(3., 3.)
        self.window.waypoint_editor.delete_selected()
        self.window.run_route()
        self.assertEqual(self.window.route_points, before)
        self.assertEqual(self.window.nav_view.waypoints, before)
        self.assertEqual(self.api.requests('/api/control/route'), [])

    def test_switching_maps_does_not_silently_rebind_existing_route_points(self):
        self.add_point()
        before = deepcopy(self.window.route_points)
        self.select_map('map-2')
        self.map_response('map-2')
        self.window.save_route()
        self.add_point(3., 3.)
        self.assertEqual(self.window.route_map_id, 'map-1')
        self.assertEqual(self.window.route_points, before)
        self.assertEqual(self.api.requests('/api/routes'), [])
        self.assertTrue(self.warning.called)

    def test_late_map_response_cannot_enable_point_selection_on_old_map(self):
        self.select_map('map-2')
        self.select_map('map-3')
        self.assertFalse(self.window.nav_view.navigation_mode)
        self.map_response('map-2')
        self.assertFalse(self.window.nav_view.navigation_mode)
        self.assertEqual(self.window.displayed_map_id, 'map-1')
        self.add_point()
        self.assertEqual(self.window.route_points, [])
        self.map_response('map-3')
        self.assertTrue(self.window.nav_view.navigation_mode)
        self.assertEqual(self.window.displayed_map_id, 'map-3')
        self.add_point()
        self.assertEqual(len(self.window.route_points), 1)
        self.assertEqual(self.window.route_map_id, 'map-3')

    def test_failed_map_load_keeps_old_image_inert(self):
        self.select_map('map-2')
        self.window._on_api('map_loaded', False, 'cannot read map')
        self.assertFalse(self.window.nav_view.navigation_mode)
        self.add_point()
        self.assertEqual(self.window.route_points, [])
        self.window.target_mode_box.setCurrentIndex(
            self.window.target_mode_box.findData('goal'))
        self.add_point()
        self.window.submit_goal()
        self.assertIsNone(self.window.draft_goal)
        self.assertEqual(self.api.requests('/api/control/goal'), [])

    def test_old_map_route_requires_explicit_migration_and_new_save(self):
        route = {'id': 'old-map-route', 'map_id': 'map-2',
                 'points': [{'x': 1., 'y': 1., 'yaw': 0., 'photo': False}]}
        self.window.routes_by_id = {route['id']: route}
        self.window.route_box.addItem('Old map route', route['id'])
        with patch('agt_robot_studio.ui.QMessageBox.question', return_value=QMessageBox.No):
            self.window.load_route()
        self.assertEqual(self.window.route_points, [])
        with patch('agt_robot_studio.ui.QMessageBox.question', return_value=QMessageBox.Yes):
            self.window.load_route()
        self.assertEqual(self.window.route_points, route['points'])
        self.assertEqual(self.window.route_map_id, 'map-1')
        self.assertIsNone(self.window.current_route_id)
        self.window.run_route()
        self.assertEqual(self.api.requests('/api/control/route'), [])
        self.window.save_route()
        queued = self.api.requests('/api/routes')[-1][3]
        self.assertEqual(queued['map_id'], 'map-1')
        self.assertEqual(queued['parent'], 'old-map-route')


if __name__ == '__main__':
    unittest.main()
