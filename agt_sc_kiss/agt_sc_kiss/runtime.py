"""Thin helpers used only by the selected experiment in the existing ROS node."""
import json
from pathlib import Path
from scipy.spatial.transform import Rotation
from .geometry import contract,write_json


def configure_query(node):
    from rclpy.parameter import Parameter
    from agt_batch_lio_adapter.extrinsics import load_lio_body_to_lidar
    if str(node.get_parameter('relocalization_query_frame_mode').value)!='mapping_body':
        raise ValueError('sc_kiss requires mapping_body; no implicit frame conversion')
    expected=str(node.get_parameter('mount_lidar_frame').value)
    if any(cloud.header.frame_id!=expected for cloud in node.clouds):
        raise ValueError('sc_kiss expects raw sensor-frame queries: '+expected)
    selected=str(node.get_parameter('body_to_base_calibration_file').value)
    if not selected:raise ValueError('selected LIO calibration is required for sc_kiss')
    t,q=load_lio_body_to_lidar(selected)
    node.set_parameters([Parameter('mapping_body_livox_translation',value=list(t)),
                         Parameter('mapping_body_livox_quaternion_xyzw',value=list(q))])


def request_arguments(node,stamp,base_from_body,directory):
    from rclpy.duration import Duration
    from rclpy.time import Time
    value=lambda name:node.get_parameter(name).value
    spec=contract(value('mapping_body_livox_translation'),value('mapping_body_livox_quaternion_xyzw'),
                  value('query_min_range_m'),value('query_max_range_m'),value('query_voxel_leaf_m'))
    path=Path(directory)/'sc_contract.json';write_json(path,spec)
    tf=node.tf_buffer.lookup_transform('odom',str(value('mount_base_frame')),Time.from_msg(stamp),
        timeout=Duration(seconds=float(value('tf_timeout_sec')))).transform
    q=tf.rotation;t=tf.translation
    body=node.compose_pose(dict(x=t.x,y=t.y,z=t.z,qx=q.x,qy=q.y,qz=q.z,qw=q.w),base_from_body)
    angles=Rotation.from_quat([body[k] for k in ('qx','qy','qz','qw')]).as_euler('xyz')
    return dict(sc_contract=str(path),sc_database=str(value('sc_database')),
                sc_roll=float(angles[0]),sc_pitch=float(angles[1]))


def check_publication(node):
    if node._sc_cancelled:raise RuntimeError('experiment stopped; result discarded')
    ready,detail=node.stationary_gate()
    if not ready:raise RuntimeError('robot moved or stationary feedback expired during sc_kiss: '+detail)
