import os
import unittest
import numpy as np
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
try:
    from PyQt5.QtWidgets import QApplication
    from agt_robot_studio.ui import Studio, DEFAULT_CONFIG
    from agt_robot_studio.map_io import save_nav2_map
except ImportError:
    QApplication = None


@unittest.skipIf(QApplication is None, 'PyQt5 is not installed in this test environment')
class StudioSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def window(self):
        window = Studio(DEFAULT_CONFIG, demo=True)
        self.addCleanup(window.close)
        return window

    def test_six_page_demo_starts_without_ros_or_workbench(self):
        window = self.window()
        self.assertEqual(list(window.pages), ['建图', '设备与传感器', '地图', '定位与导航', '参数', '日志'])
        self.assertEqual(window.stack.count(), 6)
        self.assertIsNone(window.api)
        self.assertIsNone(window.worker)
        self.assertFalse(window.map_view.image.isNull())
        self.assertFalse(window.nav_view.image.isNull())
        self.assertEqual(window.sensors_btn.text(), '连接传感器')
        self.assertEqual(window.chassis_btn.text(), '连接底盘')
        self.assertEqual(window.record_btn.text(), '开始录制')
        self.assertEqual(window.stop_ros_btn.text(), '停止全部 ROS')

    def test_device_page_camera_and_motion_checks_are_independent(self):
        window=self.window()
        self.assertIn('设备与传感器',window.pages)
        self.assertFalse(any(c.isChecked() for c in window.motion_checks.values()))
        self.assertFalse(any(window._session_payload('manual')['checks'].values()))
        called=[]
        class FakeApi:
            def call(self,*args):called.append(args)
        window.demo=False;window.api=FakeApi()
        window.start_camera()
        window.stop_camera()
        self.assertEqual([x[2] for x in called],
                         ['/api/devices/camera/start','/api/devices/camera/stop'])
        window.api=None

    def test_navigation_waypoint_position_then_heading_clicks(self):
        import math
        window=self.window()
        goals=[]
        window.nav_view.goal_clicked.connect(goals.append)
        window.nav_view.select_heading_point(1.0,2.0)
        self.assertEqual(goals,[])
        self.assertEqual(window.nav_view.pending_goal,(1.0,2.0))
        window.nav_view.select_heading_point(1.01,2.0)
        self.assertEqual(goals,[])
        window.nav_view.select_heading_point(1.0,3.0)
        self.assertEqual(len(goals),1)
        self.assertAlmostEqual(goals[0]['yaw'],math.pi/2)
        self.assertEqual((goals[0]['x'],goals[0]['y']),(1.,2.))
        self.assertIsNone(window.nav_view.pending_goal)
        window.nav_view.select_heading_point(4.,5.)
        window.nav_view.reset_heading()
        self.assertIsNone(window.nav_view.pending_goal)
        self.assertEqual(len(goals),1)

    def test_mapping_save_is_awaited_and_final_state_matches_bbs_result(self):
        window=self.window()
        window.mapping_job_id='job-1'
        window.mapping_saving=True
        window._on_api('refresh_jobs',True,[{
            'id':'job-1','kind':'mapping_live','robot':'bunker',
            'state':'RUNNING','save_requested_at':'2026-10-24T00:00:00Z',
            'save_status':{'stage':'BBS_ASSETS'}}])
        self.assertIn('正在保存',window.mapping_state.text())
        self.assertIn('BBS',window.mapping_save_status.text())
        self.assertFalse(window.glim_btn.isEnabled())
        window._on_api('refresh_jobs',True,[{
            'id':'job-1','kind':'mapping_live','robot':'bunker',
            'state':'PASS','output':'/tmp/mapping1',
            'result':{'map_version':'map-version-1',
                'relocalization_assets':{'state':'READY'}}}])
        self.assertIsNone(window.mapping_job_id)
        self.assertFalse(window.mapping_saving)
        self.assertIn('地图已经保存',window.mapping_save_status.text())
        self.assertIn('BBS 索引已生成',window.mapping_save_status.text())
        window.mapping_job_id='job-2'
        window.mapping_saving=True
        window._on_api('refresh_jobs',True,[{
            'id':'job-2','kind':'mapping_live','robot':'bunker',
            'state':'FAILED','log':'/tmp/failed.log','output':'/tmp/mapping2'}])
        self.assertIn('失败',window.mapping_save_status.text())

    def test_global_status_bar_follows_sessions_and_topic_rates_on_every_page(self):
        window=self.window()
        self.assertIn('离线演示',window.mode_state_label.text())
        window.demo=False
        window.live_status={'session':{'id':'session-1','robot':'bunker','task':'navigation'},
            'localized':True,'chassis':{'enabled':True,'online':True,'state':0}}
        window._on_state({'connected':True,'rates':{'lidar':10.2,'imu':100.,'odom':50.}})
        window._update_live()
        self.assertEqual(window.robot_state_label.text(),'设备：Bunker')
        self.assertIn('自主导航 · 已定位',window.mode_state_label.text())
        self.assertIn('通讯在线',window.chassis_state_label.text())
        self.assertIn('在线 10.2 Hz',window.lidar_state_label.text())
        window.pages['参数'].click()
        self.assertIn('在线 100.0 Hz',window.imu_state_label.text())
        window.live_status['chassis']['online']=False
        window._on_state({'connected':True,'rates':{'lidar':None,'imu':None,'odom':None}})
        window._update_live()
        self.assertIn('无反馈',window.chassis_state_label.text())
        self.assertIn('离线',window.odom_state_label.text())

    def test_generated_map_loads_into_editor_and_navigation_view(self):
        import tempfile
        from pathlib import Path
        window = self.window()
        with tempfile.TemporaryDirectory() as directory:
            yaml_path = save_nav2_map(Path(directory)/'version', np.full((4, 5), 254, dtype=np.uint8), .05, (-1., -2., .1))
            manifest = {'id':'20261024T000000Z-deadbeef','status':'DRAFT','pgm':str(Path(yaml_path).with_suffix('.pgm')),
                        'yaml':yaml_path,'resolution':.05,'origin':[-1.,-2.,.1],'width':5,'height':4}
            window._on_api('cloud_project', True, manifest)
            self.assertEqual(window.current_map_id, manifest['id'])
            self.assertEqual(window.map_view.image.width(), 5)
            self.assertEqual(window.nav_view.origin, (-1.,-2.,.1))

    def test_navigation_buttons_use_existing_workbench_contract(self):
        window = self.window()
        calls = []
        class FakeApi:
            def call(self, *args): calls.append(args)
        window.demo = False
        window.api = FakeApi()
        window.current_map_id = 'edited-map'
        window.nav_map_box.addItem('Navigation map', 'map-1')
        window.live_status['pose'] = {'x':1.,'y':2.,'yaw':.3}
        window.send_goal({'x':3.,'y':4.})
        window.move_distance(1)
        window.relocalize()
        self.assertEqual(calls[0][2], '/api/control/goal')
        self.assertEqual(calls[0][3]['map_id'], 'map-1')
        self.assertEqual(calls[0][3]['pose'], {'x':3.,'y':4.,'yaw':.3})
        self.assertEqual(calls[1][2], '/api/control/distance')
        self.assertEqual(calls[1][3]['map_id'], 'map-1')
        self.assertEqual(calls[1][3]['distance_m'], .5)
        self.assertEqual(calls[2][2], '/api/control/relocalize')
        window.global_backend_box.setCurrentIndex(window.global_backend_box.findData('kiss'))
        payload = window._session_payload('navigation', 'map-1')
        self.assertEqual(payload['global_backend'], 'kiss')
        self.assertEqual(payload['lio_backend'], 'fastlio2')
        self.assertFalse(payload['enable_camera_gimbal'])
        self.assertNotIn('global_backend', window._session_payload('sensors'))
        window.api = None

    def test_backend_change_is_checked_by_backend_without_gui_stopping_components(self):
        window = self.window()
        calls = []
        class FakeApi:
            def call(self, *args): calls.append(args)
        window.demo = False
        window.api = FakeApi()
        window._ensure_ros_worker = lambda: None
        window.live_status['session'] = {
            'task':'localization', 'map_id':'map-1', 'robot':'bunker',
            'global_backend':'bbs'}
        window.global_backend_box.setCurrentIndex(window.global_backend_box.findData('kiss'))
        window._start_session('localization', 'map-1')
        self.assertEqual(calls[0][2], '/api/session/start')
        self.assertEqual(calls[0][3]['global_backend'], 'kiss')
        self.assertIsNone(window.pending_session)
        window.api = None

    def test_photo_waypoints_are_saved_then_run_through_mission_route_api(self):
        window=self.window();calls=[]
        class FakeApi:
            def call(self,*args):calls.append(args)
        window.demo=False;window.api=FakeApi();window._ensure_ros_worker=lambda:None
        window.nav_map_box.addItem('map','map-1')
        window.target_mode_box.setCurrentIndex(window.target_mode_box.findData('route'))
        window.live_status['pose']={'x':0.,'y':0.,'yaw':.2}
        window.gimbal_heading.setValue(-30.)
        window.send_goal({'x':2.,'y':3.})
        self.assertEqual(len(window.route_points),1)
        self.assertTrue(window.route_points[0]['photo'])
        self.assertEqual(window.route_points[0]['heading'],-30.)
        window.save_route()
        self.assertEqual(calls[0][2],'/api/routes')
        self.assertEqual(calls[0][3]['map_id'],'map-1')
        window._on_api('route_save',True,{'id':'route-1'})
        window.run_route()
        self.assertEqual(calls[-1][2],'/api/control/route')
        self.assertEqual(calls[-1][3]['route_id'],'route-1')
        window.api=None

    def test_map_names_backend_labels_and_projection_payload(self):
        from PyQt5.QtWidgets import QLabel
        window=self.window();calls=[]
        class FakeApi:
            def call(self,*args):calls.append(args)
        named='20261024T000000Z-deadbeef';legacy='20261024T000001Z-abcdef01'
        window._on_api('refresh_maps',True,[{'id':named,'name':'一号温室','status':'ACCEPTED'},
            {'id':legacy,'status':'DRAFT'}])
        self.assertIn('一号温室',window.nav_map_box.itemText(0))
        self.assertIn('deadbeef',window.nav_map_box.itemText(0))
        self.assertIn(legacy,window.nav_map_box.itemText(1))
        labels={label.text() for label in window.findChildren(QLabel)}
        self.assertIn('里程计后端',labels);self.assertIn('重定位后端',labels)
        window.demo=False;window.api=FakeApi()
        window.cloud_box.addItem('一号温室','cloud-1')
        window.clouds_by_id={'cloud-1':{'id':'cloud-1','name':'一号温室'}}
        window.projection_edits['resolution'].setValue(.1)
        window.generate_grid()
        project=next(call for call in calls if call[0]=='cloud_project')
        self.assertEqual(project[3]['name'],'一号温室')
        self.assertEqual(project[3]['projection']['resolution'],.1)
        # Fake API never sends an HTTP completion callback; release its
        # simulated pending write before addCleanup(window.close).
        window.grid_building=False
        window.api=None

    def test_keepout_polygon_is_baked_undoable_and_sent_with_new_map_version(self):
        window=self.window();calls=[]
        class FakeApi:
            def call(self,*args):calls.append(args)
        index=window.map_brush.findData('keepout');window.map_brush.setCurrentIndex(index)
        window.map_view.polygon=[(10,10),(20,10),(20,20),(10,20)]
        self.assertTrue(window.map_view.finish_keepout())
        self.assertEqual(window.map_view.pixels()[10,10],0)
        self.assertEqual(len(window.map_view.keepout),1)
        window.map_view.undo_last()
        self.assertEqual(window.map_view.pixels()[10,10],254)
        self.assertEqual(window.map_view.keepout,[])
        window.map_view.polygon=[(10,10),(20,10),(20,20),(10,20)]
        window.map_view.finish_keepout()
        window.demo=False;window.api=FakeApi();window.current_map_id='map-1'
        window.export_map('DRAFT')
        self.assertEqual(calls[0][2],'/api/maps/map-1/revise')
        self.assertEqual(calls[0][3]['keepout_polygons_px'],[[[10,10],[20,10],[20,20],[10,20]]])
        window.map_saving=False
        window.api=None

    def test_queued_map_write_blocks_gui_close_before_network_request(self):
        from unittest.mock import patch
        from PyQt5.QtGui import QCloseEvent
        from PyQt5.QtWidgets import QMessageBox
        from agt_robot_studio.workbench_client import ApiWorker,WorkbenchClient
        window=self.window()
        worker=ApiWorker(WorkbenchClient())
        worker.call('map_revise','POST','/api/maps/abc/revise',{'pixels':'test'})
        self.assertTrue(worker.has_pending({'map_revise'}))
        window.api=worker
        window.demo=False
        event=QCloseEvent()
        with patch('agt_robot_studio.ui.QMessageBox.warning',return_value=QMessageBox.Ok):
            window.closeEvent(event)
        self.assertFalse(event.isAccepted())
        window.api=None

    def test_pcd_roi_cleanup_uses_full_resolution_backend_and_version(self):
        from unittest.mock import patch
        from PyQt5.QtWidgets import QMessageBox
        window=self.window()
        calls=[]
        class FakeApi:
            def call(self,*args):calls.append(args)
        window.cloud_box.addItem('原始地图','cloud-original')
        for axis in ('x','y','z'):
            window.cloud_roi_edits[(axis,'min')].setValue(-.2)
            window.cloud_roi_edits[(axis,'max')].setValue(.3)
        window.demo=False
        window.api=FakeApi()
        with patch('agt_robot_studio.ui.QMessageBox.question',return_value=QMessageBox.Yes):
            window.clean_cloud_roi()
        self.assertEqual(calls[-1][2],'/api/clouds/cloud-original/clean')
        self.assertEqual(calls[-1][3]['operations'][0]['type'],'delete')
        self.assertEqual(calls[-1][3]['operations'][0]['min'],[-.2,-.2,-.2])
        self.assertFalse(window.cloud_clean_btn.isEnabled())
        window._on_api('cloud_clean',True,{'id':'cloud-cleaned'})
        self.assertEqual(window.pending_cloud_id,'cloud-cleaned')
        self.assertTrue(window.cloud_clean_btn.isEnabled())
        window.api=None

    def test_pcd_import_keeps_map_name(self):
        from unittest.mock import patch
        window=self.window();calls=[]
        class FakeApi:
            def call(self,*args):calls.append(args)
        window.demo=False;window.api=FakeApi();window.map_name_edit.setText('一号温室_东侧')
        with patch('agt_robot_studio.ui.QFileDialog.getOpenFileName',return_value=('/tmp/map.pcd','')):
            window.import_pcd()
        self.assertEqual(calls[0][3]['name'],'一号温室_东侧')
        window.api=None

    def test_parameter_page_persists_topics_projection_and_devices(self):
        import tempfile
        from pathlib import Path
        from unittest.mock import patch
        import yaml
        window=self.window()
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)/'studio.yaml'
            window.record_topics_edit.setPlainText('/livox/lidar, /camera/image_raw')
            window.projection_edits['resolution'].setValue(.08)
            window.camera_device_edit.setText('/dev/video2')
            with patch('agt_robot_studio.ui.USER_CONFIG',target):window.save_parameters()
            saved=yaml.safe_load(target.read_text())
            self.assertEqual(saved['record_topics'],['/livox/lidar','/camera/image_raw'])
            self.assertEqual(saved['projection']['resolution'],.08)
            self.assertEqual(saved['camera_device_path'],'/dev/video2')
            self.assertEqual(window.cfg['projection']['resolution'],.08)

    def test_initial_window_fits_available_screen(self):
        window = self.window()
        available = window.screen().availableGeometry()
        self.assertLessEqual(window.width(), available.width())
        self.assertLessEqual(window.height(), available.height())
        self.assertLessEqual(window.minimumWidth(), available.width())
        self.assertLessEqual(window.minimumHeight(), available.height())
        self.assertEqual(window.global_backend_box.currentData(), 'bbs')

    def test_device_recording_and_stop_all_buttons_use_managed_backend(self):
        window = self.window()
        calls = []
        class FakeApi:
            def call(self, *args): calls.append(args)
        window.demo = False
        window.api = FakeApi()
        for checkbox in window.motion_checks.values():checkbox.setChecked(True)
        window.connect_sensors()
        window.connect_chassis()
        window.record_topics_edit.setPlainText('/livox/lidar\n/camera/image_raw')
        window.toggle_recording()
        window.recorder_id = 'recorder-1'
        window.toggle_recording()
        window.close_ros_nodes()
        self.assertEqual(calls[0][2], '/api/session/start')
        self.assertEqual(calls[0][0], 'device_sensors')
        self.assertNotIn('checks', calls[0][3])
        self.assertEqual(calls[1][0], 'device_manual')
        self.assertEqual(calls[1][3]['checks']['physical_estop'], True)
        self.assertEqual(calls[2][2], '/api/recording/start')
        self.assertEqual(calls[2][3]['template'], 'full_field_test')
        self.assertEqual(calls[2][3]['topics'], ['/livox/lidar','/camera/image_raw'])
        self.assertEqual(calls[3][2], '/api/jobs/recorder-1/stop')
        self.assertEqual(calls[4][2], '/api/session/stop_all')
        window.api = None


if __name__ == '__main__':
    unittest.main()
