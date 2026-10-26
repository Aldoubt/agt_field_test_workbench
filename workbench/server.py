"""Local browser entry. Fixed operations only; no shell or velocity endpoint."""
import argparse
import hashlib
from io import BytesIO
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import mimetypes
from pathlib import Path
import re
import signal
import zipfile
import subprocess
import tempfile
import shutil
from yaml import YAMLError

from .maps import Maps
from .cloud import sample_pcd
from .artifacts import Clouds
from .routes import Routes
from .session import Sessions,rpc
from .calibrations import Bundles
from .acceptance import navigation as acceptance_navigation,ap50
from .runtime import Jobs, ROOT, REPO, TEMPLATES, status, write_json
from .fwmini import profiles, resolve_profile, verify_profile

STATIC = REPO / 'workbench/static'
JOBS = Jobs()
MAPS = Maps()
CLOUDS = Clouds()
ROUTES = Routes()
SESSIONS = Sessions(JOBS)
BUNDLES = Bundles()


def calibrations(robot):
    if robot not in ('bunker', 'fwmini'):
        raise ValueError('unknown robot')
    return [json.loads(p.read_text()) for p in sorted((ROOT / 'calibration' / robot).glob('*.json'))]


class Handler(BaseHTTPRequestHandler):
    def response(self, code, value):
        data = json.dumps(value, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def body(self):
        length = int(self.headers.get('Content-Length', '0'))
        if length > 24_000_000:
            raise ValueError('request body too large')
        return json.loads(self.rfile.read(length)) if length else {}

    def do_GET(self):
        try:
            if self.path == '/api/profiles':
                from .profiles import list_profiles
                return self.response(200,list_profiles(ROOT))
            if self.path == '/api/clouds':return self.response(200,CLOUDS.list())
            if self.path == '/api/routes':return self.response(200,ROUTES.list())
            if self.path == '/api/live':
                # Status polling is read-only; only an explicit session start may
                # launch the bridge. Otherwise Stop All would immediately undo itself.
                try:return self.response(200,rpc(ROOT,'status'))
                except (OSError,ValueError) as exc:return self.response(200,{'state':'ROS_BRIDGE_UNAVAILABLE','reason':str(exc),'session':SESSIONS.current()})
            match=re.fullmatch(r'/api/bundles/(bunker|fwmini)',self.path)
            if match:return self.response(200,BUNDLES.list(match.group(1)))
            match=re.fullmatch(r'/api/clouds/([^/]+)/preview',self.path)
            if match:return self.response(200,sample_pcd(CLOUDS.get(match.group(1))['pcd']))
            if self.path == '/api/status':
                return self.response(200, status())
            if self.path == '/api/jobs':
                return self.response(200, JOBS.list())
            if self.path == '/api/components':return self.response(200,SESSIONS.components())
            if self.path.split('?',1)[0] == '/api/devices/camera':
                from .devices import camera_status
                return self.response(200, camera_status(JOBS, probe='probe=1' in self.path))
            if self.path == '/api/devices/captures':
                from .devices import list_captures
                return self.response(200, list_captures(JOBS))
            if self.path == '/api/devices/captures/export':
                from .devices import list_captures
                listing=list_captures(JOBS)
                if listing.get('state')!='READY':raise ValueError('no capture session to export')
                root=Path(listing['capture_root']).resolve()
                with tempfile.TemporaryFile() as buffer:
                    with zipfile.ZipFile(buffer,'w',compression=zipfile.ZIP_DEFLATED) as archive:
                        for path in root.rglob('*'):
                            if (path.is_file() and path.resolve().is_relative_to(root)
                                    and path.suffix.lower() in ('.png','.jpg','.jpeg','.json','.jsonl','.csv','.yaml')):
                                archive.write(path,path.relative_to(root).as_posix())
                    length=buffer.tell();buffer.seek(0)
                    self.send_response(200);self.send_header('Content-Type','application/zip')
                    self.send_header('Content-Disposition','attachment; filename="agt_captures.zip"')
                    self.send_header('Content-Length',str(length));self.end_headers()
                    shutil.copyfileobj(buffer,self.wfile,length=1024*1024)
                return
            if self.path == '/api/maps':
                return self.response(200, MAPS.list())
            match=re.fullmatch(r'/api/maps/([^/]+)/diff',self.path)
            if match:return self.response(200,MAPS.diff_preview(match.group(1)))
            if self.path == '/api/templates':
                return self.response(200, TEMPLATES)
            if self.path == '/api/fwmini/mounts':
                return self.response(200, profiles(ROOT))
            if self.path.startswith('/api/calibrations/'):
                return self.response(200, calibrations(self.path.rsplit('/', 1)[-1]))
            match = re.fullmatch(r'/api/maps/([^/]+)/preview', self.path)
            if match:
                data = MAPS.preview(match.group(1))
                self.send_response(200)
                self.send_header('Content-Type', 'image/png')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            match = re.fullmatch(r'/api/maps/([^/]+)/cloud', self.path)
            if match:
                manifest = MAPS.get(match.group(1))
                source = manifest.get('source_pcd')
                if not source or not Path(source).is_file():
                    raise ValueError('this 2D map has no linked PCD')
                return self.response(200, sample_pcd(source))
            match = re.fullmatch(r'/api/maps/([^/]+)/export', self.path)
            if match:
                manifest = MAPS.get(match.group(1))
                directory = ROOT / 'maps/versions' / manifest['id']
                buffer = BytesIO()
                with zipfile.ZipFile(buffer, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
                    for name in ('map.pgm', 'map.yaml', 'manifest.json', 'keepout_zones.yaml'):
                        archive.write(directory / name, name)
                data = buffer.getvalue()
                self.send_response(200)
                self.send_header('Content-Type', 'application/zip')
                self.send_header('Content-Disposition', f'attachment; filename="agt_map_{manifest["id"]}.zip"')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            match = re.fullmatch(r'/api/jobs/([^/]+)/log', self.path)
            if match:
                file = ROOT / 'runs' / match.group(1) / 'process.log'
                if not re.fullmatch(r'[0-9TZ-]{16,24}-[0-9a-f]{8}', match.group(1)) or not file.is_file():
                    raise ValueError('log not found')
                return self.response(200, {'text': file.read_text(errors='replace')[-30000:]})
            file = STATIC / ('index.html' if self.path == '/' else self.path.lstrip('/'))
            if not file.resolve().is_relative_to(STATIC) or not file.is_file():
                return self.response(404, {'error': 'not found'})
            data = file.read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', mimetypes.guess_type(file)[0] or 'application/octet-stream')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        except (ValueError, OSError, KeyError, TypeError, YAMLError) as exc:
            self.response(400, {'error': str(exc)})

    def do_POST(self):
        try:
            origin=self.headers.get('Origin')
            if origin and origin not in ('http://'+self.headers.get('Host',''), 'https://'+self.headers.get('Host','')):
                raise ValueError('cross-origin mutations are not allowed')
            if self.headers.get('Content-Type','').split(';')[0]!='application/json':raise ValueError('application/json required')
            payload = self.body()
            if self.path == '/api/devices/camera/start':
                from .devices import start_camera
                return self.response(200, start_camera(JOBS, payload))
            if self.path == '/api/devices/camera/stop':
                from .devices import stop_camera
                return self.response(200, stop_camera(JOBS))
            if self.path == '/api/devices/camera/move':
                from .devices import start_capture
                return self.response(200, start_capture(JOBS,payload,save_image=False))
            if self.path == '/api/devices/camera/capture':
                from .devices import start_capture
                return self.response(200, start_capture(JOBS,payload,save_image=True))
            match=re.fullmatch(r'/api/profiles/(bunker|fwmini)',self.path)
            if match:
                if any(j['kind']=='field_session' and j['state'] in ('RUNNING','ORPHANED') for j in JOBS.list()):raise ValueError('stop the active session before selecting a new profile')
                from .profiles import save
                return self.response(200,save(ROOT,match.group(1),payload['profile']))
            if self.path=='/api/reports/read':
                file=Path(payload['path']).expanduser();file=(ROOT/file).resolve() if not file.is_absolute() else file.resolve()
                if not any(file.is_relative_to((ROOT/p).resolve()) for p in ('runs','reports')) or file.suffix!='.json':raise ValueError('workspace JSON report required')
                return self.response(200,json.loads(file.read_text()))
            if self.path=='/api/diagnostics/time':
                from .maps import _new_id
                interface=payload.get('interface','eth0')
                if not re.fullmatch(r'[A-Za-z0-9_.:-]{1,40}',interface):raise ValueError('invalid interface')
                output=ROOT/'reports'/('time_'+_new_id()+'.json')
                return self.response(200,JOBS._new('time_diagnostics',['python3',str(REPO/'scripts/diagnose_time.py'),'--interface',interface,'--output',str(output)],{'output':str(output)}))
            if self.path=='/api/ins/start':
                from .runtime import ros_ready
                from .maps import _new_id
                import yaml
                if not ros_ready():raise ValueError('Humble workspace required')
                if any(j['kind']=='ins_observation' and j['state'] in ('RUNNING','ORPHANED') for j in JOBS.list()):raise ValueError('INS observation already running')
                port=Path(payload.get('port','/dev/ttyUSB0')).resolve();baud=payload.get('baudrate',230400)
                if not str(port).startswith('/dev/') or not port.is_char_device() or baud not in (115200,230400,460800,921600):raise ValueError('connected serial device and supported configured baud required')
                cfg=ROOT/'runs'/('ins_'+_new_id())/'driver.yaml';cfg.parent.mkdir(parents=True)
                cfg.write_text(yaml.safe_dump({'agt_asensing_driver':{'ros__parameters':dict(port=str(port),baudrate=baud,ins_frame_id='ins_link',gnss_frame_id='rtk_antenna_link',use_device_orientation_in_imu=False,rtk_fixed_types=[4])}}))
                return self.response(200,JOBS._new('ins_observation',['bash',str(REPO/'scripts/ros_exec.sh'),'ros2','run','agt_asensing_driver','asensing_node','--ros-args','-r','__node:=agt_asensing_driver','--params-file',str(cfg)],{'config':str(cfg),'scope':'HARDWARE_OBSERVATION_ONLY'}))
            if self.path=='/api/clouds/import':return self.response(200,CLOUDS.import_pcd(payload['path'],payload.get('calibration'),payload.get('bag'),payload.get('name','')))
            match=re.fullmatch(r'/api/clouds/([^/]+)/(clean|project)',self.path)
            if match:
                return self.response(200,CLOUDS.clean(match.group(1),payload['operations']) if match.group(2)=='clean' else CLOUDS.grid(match.group(1),payload.get('projection',{}),payload.get('name')))
            if self.path=='/api/routes':return self.response(200,ROUTES.save(payload['map_id'],payload['points'],payload.get('name',''),payload.get('parent')))
            if self.path=='/api/session/start':return self.response(200,SESSIONS.start(payload))
            if self.path=='/api/session/stop':return self.response(200,SESSIONS.stop())
            if self.path=='/api/session/stop_all':return self.response(200,SESSIONS.stop_all())
            match=re.fullmatch(r'/api/components/(sensors|chassis|guard|localization|navigation)/stop',self.path)
            if match:return self.response(200,SESSIONS.stop_component(match.group(1)))
            if self.path=='/api/components/guard/start':return self.response(200,SESSIONS.start_guard())
            match=re.fullmatch(r'/api/control/(distance|route|goal|jog|stop|mode|relocalize)',self.path)
            if match:return self.response(200,rpc(ROOT,match.group(1),payload))
            match=re.fullmatch(r'/api/bundles/(bunker|fwmini)',self.path)
            if match:return self.response(200,BUNDLES.save(match.group(1),payload['bundle']))
            if self.path=='/api/bundles/verify':return self.response(200,BUNDLES.verify(payload['source'],payload['report']))
            if self.path=='/api/reports/acceptance':
                from .maps import _new_id
                result={'navigation':acceptance_navigation(payload.get('trials',[])),
                    'recognition':ap50(payload.get('ground_truth',[]),payload.get('predictions',[]),payload.get('images',{}),payload.get('model',{})),
                    'physical_execution':'NOT_RUN' if not payload.get('trials') else 'USER_EVIDENCE', 'dependencies':__import__('workbench.runtime',fromlist=['source_versions']).source_versions(ROOT)}
                ident='acceptance_'+_new_id();target=ROOT/'reports'/(ident+'.json');inputs=ROOT/'reports'/(ident+'.inputs.json')
                write_json(inputs,payload);result['inputs']=dict(path=str(inputs),sha256=__import__('workbench.maps',fromlist=['hash_file']).hash_file(inputs))
                write_json(target,result)
                return self.response(200,dict(result,path=str(target)))
            if self.path in ('/api/experiments/benchmark','/api/experiments/tracker_ab'):
                from .maps import _new_id
                folder=ROOT/'reports'/('experiment_'+_new_id())
                if self.path.endswith('benchmark'):
                    spec=ROOT/'runs'/('experiment_'+_new_id())/'spec.json';write_json(spec,payload['spec'])
                    command=['python3',str(REPO/'scripts/localization_benchmark.py'),'--spec',str(spec),'--output',str(folder)]
                else:
                    command=['python3',str(REPO/'scripts/localization_ab.py'),'--bag',str(Path(payload['bag']).resolve()),'--map-id',payload['map_id'],'--robot',payload['robot'],'--calibration',str(Path(payload['calibration']).resolve()),'--output',str(folder)]
                return self.response(200,JOBS._new('offline_experiment',['bash',str(REPO/'scripts/ros_exec.sh'),*command],{'output':str(folder),'scope':'OFFLINE_ONLY'}))
            if self.path=='/api/diagnostics/analyze':
                bag=Path(payload['bag']).expanduser().resolve()
                if not (bag/'metadata.yaml').is_file():raise ValueError('existing bag required')
                from .maps import _new_id
                output=ROOT/'reports'/('motion_'+_new_id())
                return self.response(200,JOBS._new('motion_analysis',['bash',str(REPO/'scripts/ros_exec.sh'),'python3',str(REPO/'scripts/analyze_bag.py'),'--bag',str(bag),'--output',str(output)],{'output':str(output)}))
            match=re.fullmatch(r'/api/jobs/([^/]+)/finish',self.path)
            if match:return self.response(200,JOBS.finish_mapping(match.group(1)))
            if self.path == '/api/mapping/start':
                return self.response(200, JOBS.start_mapping(
                    payload.get('bag'), payload.get('robot', 'bunker'),
                    payload.get('calibration'), bool(payload.get('live')),
                    payload.get('livox_config'), payload.get('mount_profile'),payload.get('backend','fastlio'),payload.get('calibration_bundle'),payload.get('map_name','')))
            if self.path == '/api/recording/start':
                return self.response(200, JOBS.start_recorder(
                    payload.get('template'), payload.get('robot', 'bunker'),
                    payload.get('calibration'), payload.get('map_version'), payload.get('label', ''),
                    payload.get('mount_profile'), payload.get('topics')))
            if self.path == '/api/fwmini/mounts/verify':
                source = resolve_profile(ROOT, payload.get('profile'))
                report = Path(payload['report']).expanduser()
                report = (ROOT/report).resolve() if not report.is_absolute() else report.resolve()
                if not any(report.is_relative_to(ROOT/p) for p in ('runs', 'reports')):
                    raise ValueError('verification report must be in runs/ or reports/')
                stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
                target = ROOT/'calibration/fwmini'/f'fwmini_mid360_verified_{stamp}.yaml'
                verify_profile(source, report, payload['sensor_id'], target)
                return self.response(200, {'path': str(target), 'status': 'VERIFIED'})
            match = re.fullmatch(r'/api/jobs/([^/]+)/stop', self.path)
            if match:
                job=next((j for j in JOBS.list() if j['id']==match.group(1)),None)
                components={'sensor_stack':'sensors','chassis_component':'chassis','control_guard':'guard',
                            'field_session':'localization','navigation_component':'navigation'}
                if job and job['kind'] in components:
                    return self.response(200,SESSIONS.stop_component(components[job['kind']]))
                if job and job['kind']=='camera_gimbal':
                    from .devices import stop_camera
                    return self.response(200,stop_camera(JOBS))
                return self.response(200, JOBS.stop(match.group(1)))
            if self.path == '/api/maps/diff':
                return self.response(200, MAPS.compare_and_propose(payload['base_id'],payload['candidate_id'],payload.get('alignment')))
            if self.path == '/api/maps/import':
                return self.response(200, MAPS.import_yaml(payload['yaml_path'],
                                     payload.get('calibration'), payload.get('source_bag'), payload.get('name','')))
            match = re.fullmatch(r'/api/maps/([^/]+)/revise', self.path)
            if match:
                return self.response(200, MAPS.revise(match.group(1), payload['pixels'],
                                                      payload.get('status', 'DRAFT'),
                                                      payload.get('keepout_polygons_px')))
            match = re.fullmatch(r'/api/maps/([^/]+)/status', self.path)
            if match:
                return self.response(200, MAPS.set_status(match.group(1), payload['status'],
                                                           payload['reason']))
            if self.path == '/api/calibrations':
                robot = payload['robot']
                calibrations(robot)
                internal = payload['internal']
                if len(internal['rotation_row_major']) != 9 or len(internal['translation_m']) != 3:
                    raise ValueError('internal calibration requires 9 rotation and 3 translation values')
                values = internal['rotation_row_major'] + internal['translation_m']
                if any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
                    raise ValueError('calibration values must be finite numbers')
                version = payload.get('version')
                if not version or not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', version):
                    raise ValueError('invalid version')
                target = ROOT / 'calibration' / robot / (version + '.json')
                if target.exists():
                    raise ValueError('calibration version already exists')
                verified = bool(payload.get('verified', False))
                evidence = payload.get('evidence')
                if verified:
                    evidence_path = Path(evidence or '').expanduser().resolve()
                    if not evidence_path.is_file() or not any(evidence_path.is_relative_to(ROOT / p)
                                                                for p in ('runs', 'reports')):
                        raise ValueError('verified calibration requires a report in runs/ or reports/')
                    checks = json.loads(evidence_path.read_text()).get('checks', {})
                    if any(checks.get(name) != 'PASS' for name in ('repeatability', 'motion', 'geometry')):
                        raise ValueError('repeatability, motion and geometry checks must all PASS')
                    if not payload.get('sensor_id') or not payload.get('mount'):
                        raise ValueError('sensor identity and mount are required for verified calibration')
                record = {'robot': robot, 'version': version, 'internal': internal,
                          'sensor_id': payload.get('sensor_id'), 'mount': payload.get('mount'),
                          'evidence': evidence, 'verified': verified,
                          'created_at': datetime.now(timezone.utc).isoformat(),
                          'status': 'VERIFIED' if verified else 'PROVISIONAL'}
                record['sha256'] = hashlib.sha256(json.dumps(record, sort_keys=True).encode()).hexdigest()
                write_json(target, record)
                return self.response(200, record)
            return self.response(404, {'error': 'not found'})
        except (ValueError, KeyError, OSError, TypeError, YAMLError, subprocess.CalledProcessError,
                subprocess.TimeoutExpired) as exc:
            self.response(400, exc.as_response() if hasattr(exc, 'as_response') else {'error': str(exc)})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    if args.host not in ('127.0.0.1', 'localhost', '::1'):
        parser.error('only localhost binding is supported; use an authenticated tunnel for remote access')
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    def terminate(_signal, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    print(f'AGT Field Test Workbench: http://{args.host}:{args.port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        JOBS.stop_all()
        server.server_close()


if __name__ == '__main__':
    main()
