"""MID360S configuration validation; never substitutes the legacy MID360 profile."""
import hashlib
import json
from pathlib import Path


MODEL = 'MID360S'
SENSOR_IP = '192.168.1.164'
HOST_IP = '192.168.1.5'


def validate_mid360s_config(path, sensor_ip=SENSOR_IP, host_ip=HOST_IP):
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise ValueError('MID360S requires an explicit existing MID360s_config.json; legacy MID360 fallback is disabled')
    try:
        data = json.loads(path.read_text())
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f'invalid MID360S configuration {path}: {exc}') from exc
    if not isinstance(data, dict) or not isinstance(data.get('Mid360s'), dict):
        raise ValueError('MID360S configuration must contain a Mid360s block; legacy MID360 configuration is not accepted')
    hosts = data['Mid360s'].get('host_net_info')
    lidars = data.get('lidar_configs')
    if not isinstance(hosts, list) or not hosts or not isinstance(lidars, list) or not lidars:
        raise ValueError('MID360S host_net_info and lidar_configs must be non-empty lists')
    host_ips = [item.get('host_ip') for item in hosts if isinstance(item, dict)]
    sensor_ips = [item.get('ip') for item in lidars if isinstance(item, dict)]
    if host_ip not in host_ips:
        raise ValueError(f'MID360S host_ip must include {host_ip}; observed {host_ips}')
    if sensor_ip not in sensor_ips:
        raise ValueError(f'MID360S sensor ip must include {sensor_ip}; observed {sensor_ips}')
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    return {
        'model': MODEL,
        'config_path': str(path),
        'config_sha256': digest,
        'sensor_ip': sensor_ip,
        'host_ip': host_ip,
        'device_identity': f'{MODEL}@{sensor_ip}',
        'serial_number': 'NOT_QUERIED',
        'connection_state': 'CONFIG_VALIDATED_NOT_CONNECTED',
    }
