"""Exercise map/cloud identity and delayed preview replies without ROS or devices."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
try:
    from PyQt5.QtGui import QImage
    from PyQt5.QtWidgets import QApplication, QMessageBox
except ImportError:
    QApplication = None
else:
    from agt_robot_studio.ui import Studio, DEFAULT_CONFIG


@unittest.skipIf(QApplication is None, 'PyQt5 is not installed in this test environment')
class CloudSelectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def window(self):
        window = Studio(DEFAULT_CONFIG, demo=True)
        window.timer.stop();window.display_timer.stop()
        calls = []
        class Api:
            def call(self, *args):calls.append(args)
        window.demo = False;window.api = Api()
        window._ensure_ros_worker = lambda: None
        def close():
            window.api = None;window.grid_building = False;window.map_saving = False
            window.close()
        self.addCleanup(close)
        return window, calls

    @staticmethod
    def cloud(letter, cleanup='APPLIED'):
        return {'id':'cloud-'+letter,'name':'地图 '+letter,'status':'DRAFT',
                'pcd':'/maps/cloud-'+letter+'/cloud.pcd','sha256':letter*64,
                'processing':{'input_points':1500,'output_points':1234},
                'keyframe_cleanup':{'state':cleanup}}

    @staticmethod
    def map_record(cloud):
        return {'id':'map-'+cloud['id'],'name':cloud['name'],'cloud_id':cloud['id'],
                'source_pcd':cloud['pcd'],'source_pcd_sha256':cloud['sha256'],
                'resolution':.05,'origin':[0.,0.,0.],'width':4,'height':3,
                'status':'DRAFT','asset_summary':'测试资产未构建'}

    @staticmethod
    def image():
        image = QImage(4,3,QImage.Format_Grayscale8);image.fill(254)
        return image

    @staticmethod
    def preview(cloud, x):
        return {'cloud_id':cloud['id'],'cloud_sha256':cloud['sha256'],
                'points':[[x,2.,3.]],'sampled_points':1,'total_points':1234}

    @staticmethod
    def tag(cloud):return 'cloud_preview:'+cloud['id']+':'+cloud['sha256']

    def test_loading_map_selects_its_source_cloud_not_previous_cloud(self):
        window,calls = self.window();a=self.cloud('a');b=self.cloud('b')
        window._on_api('refresh_clouds',True,[a,b])
        self.assertEqual(window.cloud_box.currentData(),a['id'])
        window._load_map(self.map_record(b),self.image())
        self.assertEqual(window.cloud_box.currentData(),b['id'])
        self.assertEqual(window.cloud_preview_request,(b['id'],b['sha256']))
        self.assertEqual(calls[-1][2],'/api/clouds/'+b['id']+'/preview')
        self.assertEqual(window.clean_view.xyz.size,0)
        window._on_api(self.tag(b),True,self.preview(b,20.))
        self.assertEqual(float(window.clean_view.xyz[0,0]),20.)
        self.assertIn(b['name'],window.cloud_info.text())

    def test_missing_source_clears_selection_until_matching_inventory_arrives(self):
        window,_ = self.window();a=self.cloud('a');b=self.cloud('b')
        window._on_api('refresh_clouds',True,[a])
        window._on_api(self.tag(a),True,self.preview(a,10.))
        window._load_map(self.map_record(b),self.image())
        self.assertIsNone(window.cloud_box.currentData())
        self.assertEqual(window.clean_view.xyz.size,0)
        window._on_api('refresh_clouds',True,[a])
        self.assertIsNone(window.cloud_box.currentData())
        window._on_api('refresh_clouds',True,[a,b])
        self.assertEqual(window.cloud_box.currentData(),b['id'])
        self.assertEqual(window.cloud_preview_request,(b['id'],b['sha256']))

    def test_import_and_cleanup_refresh_selected_cloud_even_with_blocked_signals(self):
        for event in ('cloud_import','cloud_clean'):
            with self.subTest(event=event):
                window,calls=self.window();a=self.cloud('a');b=self.cloud('b')
                window._on_api('refresh_clouds',True,[a])
                window._on_api(self.tag(a),True,self.preview(a,10.))
                window._on_api(event,True,b)
                self.assertEqual(window.pending_cloud_id,b['id'])
                with patch('agt_robot_studio.ui.QMessageBox.warning') as warning, \
                        patch('agt_robot_studio.ui.QMessageBox.question',return_value=QMessageBox.No) as question:
                    window.clean_cloud_roi()
                    window.generate_grid()
                    self.assertEqual(warning.call_count,2);question.assert_not_called()
                # An inventory request sent before cleanup can arrive after it.
                window._on_api('refresh_clouds',True,[a])
                self.assertEqual(window.pending_cloud_id,b['id'])
                self.assertIsNone(window.cloud_box.currentData())
                window._on_api('refresh_clouds',True,[b,a])
                self.assertEqual(window.cloud_box.currentData(),b['id'])
                self.assertEqual(calls[-1][0],self.tag(b))
                self.assertEqual(window.clean_view.xyz.size,0)
                window._on_api(self.tag(b),True,self.preview(b,20.))
                self.assertEqual(float(window.clean_view.xyz[0,0]),20.)

    def test_late_preview_success_failure_and_wrong_sha_cannot_replace_new_cloud(self):
        window,_=self.window();a=self.cloud('a');b=self.cloud('b')
        window._on_api('refresh_clouds',True,[a,b])
        window.cloud_box.setCurrentIndex(window.cloud_box.findData(b['id']))
        window._on_api(self.tag(a),True,self.preview(a,10.))
        self.assertEqual(window.clean_view.xyz.size,0)
        window._on_api(self.tag(b),True,self.preview(b,20.))
        previous_text=window.cloud_info.text()
        with patch('agt_robot_studio.ui.QMessageBox.warning') as warning:
            window._on_api(self.tag(a),False,'old request failed')
            warning.assert_not_called()
        bad=self.preview(b,99.);bad['cloud_sha256']='c'*64
        window._on_api(self.tag(b),True,bad)
        self.assertEqual(float(window.clean_view.xyz[0,0]),20.)
        self.assertEqual(window.cloud_info.text(),previous_text)

    def test_confirmation_names_actual_cloud_and_carries_optional_filter(self):
        window,calls=self.window();cloud=self.cloud('a','UNAVAILABLE')
        window._on_api('refresh_clouds',True,[cloud])
        for axis in ('x','y','z'):
            window.cloud_roi_edits[(axis,'min')].setValue(-.2)
            window.cloud_roi_edits[(axis,'max')].setValue(.3)
        window.query_self_filter_edit.setText('/maps/reviewed_query_filter.json')
        with patch('agt_robot_studio.ui.QMessageBox.question',return_value=QMessageBox.Yes) as question:
            window.clean_cloud_roi()
        text=question.call_args.args[2]
        for expected in (cloud['name'],cloud['id'],cloud['sha256'],'1,234','UNAVAILABLE'):
            self.assertIn(expected,text)
        self.assertIn('仅完成几何地图清理',text)
        self.assertEqual(calls[-1][2],'/api/clouds/'+cloud['id']+'/clean')
        self.assertEqual(calls[-1][3]['query_self_filter_config'],'/maps/reviewed_query_filter.json')
        self.assertEqual(calls[-1][3]['operations'][0]['min'],[-.2,-.2,-.2])

    def test_selected_map_supplies_its_frozen_filter_without_manual_reentry(self):
        window,_=self.window();a=self.cloud('a');b=self.cloud('b')
        b.update(query_self_filter_config='/maps/cloud-b/query_self_filter.json',
                 query_self_filter_sha256='e'*64)
        window._on_api('refresh_clouds',True,[a,b])
        window._load_map(self.map_record(b),self.image())
        self.assertEqual(window.query_self_filter_edit.text(),b['query_self_filter_config'])
        self.assertEqual(window._session_payload('localization')['query_self_filter_config'],
                         b['query_self_filter_config'])
        self.assertIn('e'*16,window.query_filter_source_label.text())
        self.assertFalse(window.query_filter_override)

    def test_manual_filter_override_survives_same_source_reload_but_new_source_rebinds(self):
        window,_=self.window();a=self.cloud('a');b=self.cloud('b')
        a.update(query_self_filter_config='/maps/cloud-a/query_self_filter.json',
                 query_self_filter_sha256='d'*64)
        window._on_api('refresh_clouds',True,[a,b])
        override='/maps/reviewed_query_filter.json'
        window.query_self_filter_edit.setText(override)
        window.query_self_filter_edit.textEdited.emit(override)
        window._load_map(self.map_record(a),self.image())
        window._on_api('refresh_clouds',True,[a,b])
        self.assertEqual(window.query_self_filter_edit.text(),override)
        self.assertTrue(window.query_filter_override)
        window._load_map(self.map_record(b),self.image())
        self.assertEqual(window.query_self_filter_edit.text(),'')
        self.assertFalse(window.query_filter_override)

    def test_cleared_filter_is_explicit_in_cleanup_request_and_unsupported_reason_is_visible(self):
        window,calls=self.window();cloud=self.cloud('a','UNSUPPORTED')
        cloud.update(query_self_filter_config='/maps/cloud-a/query_self_filter.json',
                     query_self_filter_sha256='d'*64)
        cloud['keyframe_cleanup']['reason']='binary_compressed keyframe patches require conversion'
        window._on_api('refresh_clouds',True,[cloud])
        window.query_self_filter_edit.clear();window.query_self_filter_edit.textEdited.emit('')
        for axis in ('x','y','z'):
            window.cloud_roi_edits[(axis,'min')].setValue(-.2)
            window.cloud_roi_edits[(axis,'max')].setValue(.3)
        with patch('agt_robot_studio.ui.QMessageBox.question',return_value=QMessageBox.Yes) as question:
            window.clean_cloud_roi()
        self.assertEqual(calls[-1][3]['query_self_filter_config'],'')
        self.assertIn('未启用',question.call_args.args[2])
        self.assertIn('格式不支持同步',question.call_args.args[2])
        self.assertIn('binary_compressed',question.call_args.args[2])

    def test_filter_config_is_saved_and_shared_by_import_and_localization_only(self):
        window,calls=self.window()
        self.assertEqual(window._session_payload('localization')['query_self_filter_config'],'')
        self.assertNotIn('query_self_filter_config',window._session_payload('sensors'))
        window.query_self_filter_edit.setText('/maps/reviewed_query_filter.json')
        self.assertEqual(window._session_payload('localization')['query_self_filter_config'],
                         '/maps/reviewed_query_filter.json')
        with patch('agt_robot_studio.ui.QFileDialog.getOpenFileName',return_value=('/maps/new.pcd','')):
            window.import_pcd()
        self.assertEqual(calls[-1][3]['calibration']['query_self_filter_config'],
                         '/maps/reviewed_query_filter.json')
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)/'studio.yaml'
            with patch('agt_robot_studio.ui.USER_CONFIG',target):window.save_parameters()
            import yaml
            saved=yaml.safe_load(target.read_text())
            self.assertEqual(saved['query_self_filter_config'],'/maps/reviewed_query_filter.json')

    def test_nonempty_or_running_filter_is_not_silently_reused_by_ui(self):
        window,calls=self.window()
        window.global_backend_box.setCurrentIndex(window.global_backend_box.findData('bbs'))
        window.live_status['session']={'task':'localization','map_id':'map-a','robot':'bunker',
            'global_backend':'bbs','lio_backend':'fastlio2'}
        window._start_session('localization','map-a')
        self.assertFalse(any(call[2]=='/api/session/start' for call in calls))
        window.query_self_filter_edit.setText('/maps/reviewed_query_filter.json')
        window._start_session('localization','map-a')
        self.assertEqual(calls[-1][2],'/api/session/start')
        calls.clear();window.query_self_filter_edit.clear()
        window.live_status['session']['query_self_filter_sha256']='d'*64
        window._start_session('localization','map-a')
        self.assertEqual(calls[-1][2],'/api/session/start')
        self.assertEqual(calls[-1][3]['query_self_filter_config'],'')


if __name__ == '__main__':unittest.main()
