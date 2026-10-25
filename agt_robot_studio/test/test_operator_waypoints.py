import math
import os
import unittest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
try:
    from PyQt5.QtWidgets import QApplication
    from agt_robot_studio.operator_waypoints import WaypointEditor
except ImportError:
    QApplication = None


@unittest.skipIf(QApplication is None, 'PyQt5 is not installed in this test environment')
class WaypointEditorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def editor(self):
        widget = WaypointEditor()
        self.addCleanup(widget.close)
        return widget

    @staticmethod
    def point(x=1., yaw=math.pi/2):
        return dict(x=x, y=2., yaw=yaw, photo=True, heading=15., pitch=-10.,
                    roll=3., photo_required=True)

    def test_loading_and_selection_do_not_mark_route_dirty(self):
        widget = self.editor()
        changes = []
        widget.points_changed.connect(changes.append)
        source = [self.point(), self.point(3.)]
        widget.set_points(source)
        widget.select_point(1)
        self.assertEqual(changes, [])
        self.assertEqual(widget.selected_index, 1)
        self.assertAlmostEqual(widget.yaw_spin.value(), 90.)
        source[1]['x'] = 90.
        self.assertEqual(widget.points()[1]['x'], 3.)
        snapshot = widget.points()
        snapshot[0]['x'] = 99.
        self.assertEqual(widget.points()[0]['x'], 1.)

    def test_only_edited_field_changes_and_yaw_converts_degrees(self):
        widget = self.editor()
        original = self.point(yaw=1.23456789)
        original['extension'] = {'kept': True}
        widget.set_points([original])
        changes = []
        widget.points_changed.connect(changes.append)
        widget.heading_spin.setValue(32.5)
        self.assertEqual(len(changes), 1)
        self.assertEqual(changes[-1][0]['yaw'], original['yaw'])
        self.assertEqual(changes[-1][0]['roll'], 3.)
        self.assertEqual(changes[-1][0]['extension'], {'kept': True})
        widget.yaw_spin.setValue(-45.)
        self.assertAlmostEqual(changes[-1][0]['yaw'], -math.pi/4)
        self.assertEqual(changes[-1][0]['heading'], 32.5)
        self.assertNotIn('id', changes[-1][0])

    def test_reorder_and_delete_keep_the_selected_waypoint(self):
        widget = self.editor()
        widget.set_points([self.point(1.), self.point(2.), self.point(3.)], selected_index=1)
        changes = []
        widget.points_changed.connect(changes.append)
        self.assertTrue(widget.move_selected(-1))
        self.assertEqual(widget.selected_index, 0)
        self.assertEqual([p['x'] for p in changes[-1]], [2., 1., 3.])
        self.assertFalse(widget.move_selected(-1))
        self.assertEqual(len(changes), 1)
        self.assertTrue(widget.delete_selected())
        self.assertEqual([p['x'] for p in changes[-1]], [1., 3.])
        self.assertEqual(widget.selected_index, 0)

    def test_copy_measured_angles_is_atomic_and_does_not_change_pose(self):
        widget = self.editor()
        widget.set_points([self.point()])
        changes = []
        widget.points_changed.connect(changes.append)
        self.assertTrue(widget.apply_gimbal_angles({'heading': 30., 'pitch': -15.}))
        self.assertEqual(len(changes), 1)
        self.assertEqual(changes[0][0]['roll'], 3.)
        self.assertEqual(changes[0][0]['x'], 1.)
        before = widget.points()
        with self.assertRaises(ValueError):
            widget.apply_gimbal_angles({'heading': 25., 'pitch': 100.})
        self.assertEqual(widget.points(), before)
        self.assertEqual(len(changes), 1)
        with self.assertRaises(ValueError):
            widget.apply_gimbal_angles({'heading': float('nan')})
        self.assertEqual(widget.points(), before)

    def test_empty_selection_and_running_lock_prevent_edits(self):
        widget = self.editor()
        self.assertEqual(widget.selected_index, -1)
        self.assertFalse(widget.fields_widget.isEnabled())
        self.assertFalse(widget.delete_selected())
        self.assertFalse(widget.apply_gimbal_angles({'heading': 30.}))
        widget.set_points([self.point(), self.point(3.)])
        changes = []
        widget.points_changed.connect(changes.append)
        widget.set_editable(False, '任务执行中')
        widget.x_spin.setValue(9.)
        self.assertFalse(widget.move_selected(1))
        self.assertFalse(widget.delete_selected())
        self.assertFalse(widget.apply_gimbal_angles({'heading': 30.}))
        widget.select_point(1)
        self.assertEqual(changes, [])
        self.assertEqual(widget.points()[0]['x'], 1.)
        self.assertIn('任务执行中', widget.hint_label.text())
        widget.set_editable(True)
        self.assertTrue(widget.fields_widget.isEnabled())

    def test_invalid_load_preserves_current_route(self):
        widget = self.editor()
        widget.set_points([self.point()])
        before = widget.points()
        changes = []
        widget.points_changed.connect(changes.append)
        with self.assertRaises(ValueError):
            widget.set_points([dict(x=1., y=2., yaw=float('inf'))])
        self.assertEqual(widget.points(), before)
        self.assertEqual(changes, [])

    def test_last_delete_returns_clear_empty_state(self):
        widget = self.editor()
        widget.set_points([self.point()])
        changes = []
        widget.points_changed.connect(changes.append)
        widget.delete_selected()
        self.assertEqual(changes, [[]])
        self.assertEqual(widget.selected_index, -1)
        self.assertFalse(widget.fields_widget.isEnabled())
        self.assertIn('尚无航点', widget.hint_label.text())


if __name__ == '__main__':
    unittest.main()
