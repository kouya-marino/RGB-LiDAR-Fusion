"""Conversions between ROS messages and numpy, plus small quaternion helpers.

Kept separate from the node so they can be unit-tested without starting DDS.
Messages are parsed with numpy directly rather than sensor_msgs_py/cv_bridge:
it is faster, and it handles livox_ros_driver2's packed point layout (mixed
float32 / uint8 / float64 fields, 26 bytes per point) without assumptions.
"""
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, PointField

from rgb_lidar_fusion.io_cloud import pack_rgb

_DTYPE_OF = {
    PointField.INT8: np.int8, PointField.UINT8: np.uint8,
    PointField.INT16: np.int16, PointField.UINT16: np.uint16,
    PointField.INT32: np.int32, PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64,
}


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract (N,3) float64 xyz from a PointCloud2, whatever else it carries.

    Lidar drivers add fields (intensity, tag, line, timestamp...) and may pack
    them unaligned, so build the structured dtype from the message's own fields
    rather than assuming a layout.
    """
    names = {f.name: f for f in msg.fields}
    for axis in ('x', 'y', 'z'):
        if axis not in names:
            raise ValueError('PointCloud2 has no {!r} field'.format(axis))
    dtype = np.dtype({
        'names': [f.name for f in msg.fields],
        'formats': [_DTYPE_OF[f.datatype] for f in msg.fields],
        'offsets': [f.offset for f in msg.fields],
        'itemsize': msg.point_step,
    })
    arr = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
    return np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float64)


def image_to_rgb(msg: Image) -> np.ndarray:
    """Decode an Image to an (H,W,3) uint8 RGB array.

    Handled inline rather than via cv_bridge: colour cameras publish rgb8 or
    bgr8, and a reshape is both faster and one less dependency.
    """
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('rgb8', 'bgr8'):
        img = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width, :]
        return img[:, :, ::-1] if enc == 'bgr8' else img
    if enc in ('mono8', '8uc1'):
        gray = buf.reshape(msg.height, msg.step)[:, :msg.width]
        return np.repeat(gray[:, :, None], 3, axis=2)
    raise ValueError('unsupported image encoding {!r}'.format(msg.encoding))


def slerp(q0: np.ndarray, q1: np.ndarray, u: float) -> np.ndarray:
    """Shortest-path quaternion slerp. Hand-rolled to avoid building a scipy
    Slerp object per scan, and to make the sign fix explicit."""
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:              # take the short way round
        q1, dot = -q1, -dot
    if dot > 0.9995:           # nearly identical -> lerp is numerically safer
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = np.arccos(np.clip(dot, -1.0, 1.0))
    s = np.sin(theta)
    return (np.sin((1.0 - u) * theta) / s) * q0 + (np.sin(u * theta) / s) * q1


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def angle_between(q0: np.ndarray, q1: np.ndarray) -> float:
    """Rotation angle in radians between two quaternions."""
    dot = abs(float(np.dot(q0 / np.linalg.norm(q0), q1 / np.linalg.norm(q1))))
    return 2.0 * np.arccos(np.clip(dot, -1.0, 1.0))


def transform_to_matrix(tf) -> np.ndarray:
    """geometry_msgs/Transform (translation + quaternion) -> 4x4 SE(3)."""
    t, q = tf.translation, tf.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(np.array([q.x, q.y, q.z, q.w], dtype=float))
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def xyzrgb_to_msg(xyz: np.ndarray, rgb: np.ndarray, frame_id: str,
                  stamp) -> PointCloud2:
    """Build a PointCloud2 with the PCL `x y z rgb` layout (16 bytes/point),
    which RViz shows with its RGB8 colour transformer."""
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    n = xyz.shape[0]
    record = np.empty((n, 4), dtype=np.float32)
    record[:, :3] = xyz
    record[:, 3] = pack_rgb(rgb)
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp
    msg.height, msg.width = 1, n
    msg.is_bigendian = False
    msg.point_step, msg.row_step = 16, 16 * n
    msg.is_dense = True
    msg.fields = [PointField(name=name, offset=4 * i,
                             datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z', 'rgb'))]
    msg.data = record.tobytes(order='C')
    return msg
