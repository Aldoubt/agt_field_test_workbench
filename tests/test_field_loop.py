import sys
import tempfile
import unittest
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'agt_field_test_bringup'))
from agt_field_test_bringup import profiles,calibration
from workbench.routes import relative
from workbench.gps import prior
from workbench.acceptance import navigation

class FieldLoop(unittest.TestCase):
    def test_profile_is_actual_nav_guard_and_smoother_limit(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'robot.yaml';p.write_text(yaml.safe_dump(dict(robot='bunker',forward_mps=.15,reverse_mps=.08,angular_radps=.1,linear_accel_mps2=.1,linear_decel_mps2=.2,control_hz=50,footprint_m=[[-.5,-.4],[.5,-.4],[.5,.4]],body_frame='base_link')))
            spec=profiles.load(p);out=Path(d)/'nav.yaml'
            profiles.navigation(spec,ROOT.parent/'agt_navigation_v3/config',out)
            nav=yaml.safe_load(out.read_text())
            self.assertEqual(nav['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'],.15)
            self.assertEqual(nav['velocity_smoother']['ros__parameters']['max_velocity'],[.15,0.,.1])
            self.assertTrue(nav['agt_cmd_vel_guard']['ros__parameters']['require_chassis_feedback'])
            self.assertEqual(profiles.guard(spec)['max_linear_y'],0.)
    def test_bunker_cannot_be_made_holonomic(self):
        p=ROOT/'agt_field_test_bringup/config/bunker_calibration_candidate.yaml'
        b=calibration.load(p,'bunker');self.assertEqual(b['status'],'CANDIDATE')
        self.assertFalse(b['rtk_arm']['enabled'])
        xml=calibration.description(b,'fastlio2')
        self.assertNotIn('rtk_antenna_link',xml)
        self.assertIn('livox_frame',xml)
    def test_mechanical_pitch_reaches_generated_robot_description(self):
        import math
        import xml.etree.ElementTree as ET
        bundle=calibration.load(ROOT/'agt_field_test_bringup/config/bunker_calibration_candidate.yaml','bunker')
        xml=ET.fromstring(calibration.description(bundle,'fastlio2'))
        origin=xml.find("joint[@name='base_link_to_lidar_link']/origin")
        self.assertAlmostEqual(float(origin.attrib['rpy'].split()[1]),math.radians(15.))
        self.assertEqual(bundle['mount']['pitch_confirmation']['pitch_deg'],15.)
        self.assertEqual(bundle['internal']['fastlio2']['translation'],[-.011,-.02329,.04412])

    def test_relative_goal_uses_robot_heading(self):
        import math
        target=relative(dict(x=2.,y=3.,yaw=math.pi/2),-5.)
        self.assertAlmostEqual(target['x'],2.);self.assertAlmostEqual(target['y'],-2.)
    def test_gps_cannot_be_truth_without_fix_and_alignment(self):
        result=prior({}, {}, {},100.)
        self.assertFalse(result['usable']);self.assertEqual(result['weight'],0.)
    def test_missing_physical_trials_remain_not_run(self):
        report=navigation([])
        self.assertTrue(all(r['formal_state']=='NOT_RUN' for r in report['results']))
        self.assertEqual([r['engineering_limit_m'] for r in report['results']],[.12,.15,.20])

    def test_healthy_gps_prior_and_jump_degradation(self):
        status=dict(fix_stamp=100.,status_stamp=100.,imu_stamp=100.,rtk_fixed=True,position_status_valid=True,position_std_valid=True,east_std=.02,north_std=.02,up_std=.04)
        align=dict(status='VERIFIED',lever_arm_status='VERIFIED',origin_lla=[30.,120.,50.],enu_to_map_xy=[1.,2.],enu_to_map_yaw=0.,lever_arm_base_xyz=[.5,.1,0.])
        fix=dict(latitude=30.,longitude=120.,altitude=50.,status=2)
        value=prior(fix,status,align,100.1,dict(x=.5,y=1.9,yaw=0.))
        self.assertTrue(value['usable']);self.assertAlmostEqual(value['map_prior']['x'],.5)
        value=prior(fix,status,align,100.1,dict(x=10.,y=10.,yaw=0.))
        self.assertFalse(value['usable']);self.assertIn('gps_jump_rejected',value['reasons'])
        self.assertFalse(prior(fix,status,align,101.,dict(x=.5,y=1.9,yaw=0.))['usable'])
    def test_software_report_cannot_open_physical_navigation(self):
        from workbench.safety import report
        import json
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'report.json';p.write_text(json.dumps(dict(scope='SOFTWARE_TEST',checks={})))
            with self.assertRaises(ValueError):report(p,'bunker','a','b','c')

    def test_navigation_calibration_keeps_navigation_frames(self):
        bundle=calibration.load(ROOT/'agt_field_test_bringup/config/bunker_calibration_candidate.yaml','bunker')
        share=ROOT.parent/'agt_navigation_v3/navigation/nav2/agt_navigation_runtime'
        with tempfile.TemporaryDirectory() as d:
            fast,batch=calibration.configs(bundle,share,ROOT.parent/'agt-lio-pgo-mapping/bringup/agt_mapping_bringup/config/fastlio2_mid360.yaml',d)
            f=yaml.safe_load(Path(fast).read_text());b=yaml.safe_load(Path(batch).read_text())
            self.assertEqual((f['world_frame'],f['body_frame']),('odom','body'))
            self.assertFalse(f['esti_il'])
            self.assertEqual(f['t_il'],bundle['internal']['fastlio2']['translation'])
            self.assertEqual(b['/**']['ros__parameters']['mapping']['extrinsic_T'],bundle['internal']['batch_lio']['translation'])

    def test_profile_upgrade_versions_preserve_original_limits(self):
        from workbench.profiles import list_profiles,save
        import shutil
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);folder=root/'config/robots';folder.mkdir(parents=True)
            for robot in ('bunker','fwmini'):shutil.copyfile(ROOT/'templates/config/robots'/(robot+'.yaml'),folder/(robot+'.yaml'))
            target=folder/'fwmini.yaml';old=yaml.safe_load(target.read_text());old['can_bitrate']=None;old['forward_mps']=.04;target.write_text(yaml.safe_dump(old));before=target.read_bytes()
            records=list_profiles(root);fw=next(p for p in records if p['robot']=='fwmini');self.assertFalse(fw['runtime_valid'])
            value=save(root,'fwmini',fw['migration_candidate'])
            self.assertEqual(value['forward_mps'],.04);self.assertEqual(value['can_bitrate'],500000)
            self.assertEqual(target.read_bytes(),before)
            self.assertEqual(profiles.load(profiles.selected_path(root,'fwmini'))['forward_mps'],.04)

if __name__=='__main__':unittest.main()
