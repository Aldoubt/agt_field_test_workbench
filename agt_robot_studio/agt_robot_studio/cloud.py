"""Read only XYZ from ROS PointCloud2 with endian, offset and stride handled."""
import numpy as np

_DT = {7: 'f4', 8: 'f8'}  # PointField.FLOAT32/FLOAT64


def xyz_from_cloud(msg, limit=5500):
    count = int(msg.width) * int(msg.height)
    if count < 1 or msg.point_step < 12:
        return np.empty((0, 3), dtype=np.float32)
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields and fields[k].datatype in _DT for k in ('x', 'y', 'z')):
        return np.empty((0, 3), dtype=np.float32)
    names = ('x', 'y', 'z')
    dtype = np.dtype({'names': list(names), 'formats': [ ('>' if msg.is_bigendian else '<') + _DT[fields[k].datatype] for k in names],
                      'offsets': [int(fields[k].offset) for k in names], 'itemsize': int(msg.point_step)})
    if msg.row_step < msg.width * msg.point_step:
        return np.empty((0, 3), dtype=np.float32)
    if len(msg.data) < int(msg.row_step) * int(msg.height):
        return np.empty((0, 3), dtype=np.float32)
    a = np.ndarray((int(msg.height), int(msg.width)), dtype=dtype,
                   buffer=bytes(msg.data), strides=(int(msg.row_step), int(msg.point_step)))
    step = max(1, count // max(1, limit))
    flat = a.reshape(-1) if msg.row_step == msg.width * msg.point_step else a.copy().reshape(-1)
    sample = flat[::step][:limit]
    points = np.column_stack([sample[k] for k in names]).astype(np.float32, copy=False)
    return points[np.isfinite(points).all(axis=1)]
