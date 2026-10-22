#!/usr/bin/env python3
"""Append tracker decisions and actual map->odom TF updates to JSONL."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


class LocalizationAudit(Node):
    def __init__(self):
        super().__init__('agt_field_test_localization_audit')
        self.declare_parameter('directory', str(Path.home() / 'agt_field_test_ws/runs'))
        self.declare_parameter('global_map', '')
        directory = Path(self.get_parameter('directory').value)
        directory.mkdir(parents=True, exist_ok=True)
        map_path = Path(self.get_parameter('global_map').value)
        self.map_sha256 = hashlib.sha256(map_path.read_bytes()).hexdigest() if map_path.is_file() else None
        name = datetime.now(timezone.utc).strftime('localization_%Y%m%dT%H%M%SZ_') + uuid.uuid4().hex[:8]
        self.path = directory / (name + '.jsonl')
        self.output = self.path.open('x', encoding='utf-8', buffering=1)
        self.create_subscription(String, '/agt/map_tracking/status', self.tracker, 50)
        self.create_subscription(String, '/agt/relocalization/status', self.manager, 50)
        self.create_subscription(TFMessage, '/tf', self.transforms, 100)

    def record(self, kind, data):
        self.output.write(json.dumps({'observed_at': datetime.now(timezone.utc).isoformat(),
                                      'kind': kind, 'global_map_sha256': self.map_sha256,
                                      **data}, sort_keys=True) + '\n')

    def tracker(self, msg):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self.record('tracker', payload)

    def manager(self, msg):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self.record('localization_manager', payload)

    def transforms(self, msg):
        for transform in msg.transforms:
            if (transform.header.frame_id.lstrip('/') != 'map' or
                    transform.child_frame_id.lstrip('/') != 'odom'):
                continue
            t, q = transform.transform.translation, transform.transform.rotation
            self.record('map_odom_tf', {'stamp_sec': transform.header.stamp.sec,
                        'stamp_nanosec': transform.header.stamp.nanosec,
                        'translation': [t.x, t.y, t.z], 'rotation_xyzw': [q.x, q.y, q.z, q.w]})

    def destroy_node(self):
        self.output.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = LocalizationAudit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
