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

from .maps import Maps
from .cloud import sample_pcd
from .runtime import Jobs, ROOT, REPO, TEMPLATES, status, write_json

STATIC = REPO / 'workbench/static'
JOBS = Jobs()
MAPS = Maps()


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
            if self.path == '/api/status':
                return self.response(200, status())
            if self.path == '/api/jobs':
                return self.response(200, JOBS.list())
            if self.path == '/api/maps':
                return self.response(200, MAPS.list())
            if self.path == '/api/templates':
                return self.response(200, TEMPLATES)
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
        except (ValueError, OSError) as exc:
            self.response(400, {'error': str(exc)})

    def do_POST(self):
        try:
            payload = self.body()
            if self.path == '/api/mapping/start':
                return self.response(200, JOBS.start_mapping(
                    payload.get('bag'), payload.get('robot', 'bunker'),
                    payload.get('calibration'), bool(payload.get('live')),
                    payload.get('livox_config')))
            if self.path == '/api/recording/start':
                return self.response(200, JOBS.start_recorder(
                    payload.get('template'), payload.get('robot', 'bunker'),
                    payload.get('calibration'), payload.get('map_version'), payload.get('label', '')))
            match = re.fullmatch(r'/api/jobs/([^/]+)/stop', self.path)
            if match:
                return self.response(200, JOBS.stop(match.group(1)))
            if self.path == '/api/maps/import':
                return self.response(200, MAPS.import_yaml(payload['yaml_path'],
                                     payload.get('calibration'), payload.get('source_bag')))
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
        except (ValueError, KeyError, OSError, TypeError, subprocess.CalledProcessError,
                subprocess.TimeoutExpired) as exc:
            self.response(400, {'error': str(exc)})


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
