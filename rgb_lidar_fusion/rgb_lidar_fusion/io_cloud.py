"""Write coloured clouds out as binary PLY and binary PCD.

Both formats are emitted. PLY is the more portable of the two and is the one to
open first if a viewer disagrees about the PCD. The PCD uses the conventional
PCL `x y z rgb` layout (rgb = a float32 holding a packed 0x00RRGGBB uint32),
which is what PCL, CloudCompare, MeshLab and Open3D all expect -- and, for
later, exactly what three.js PCDLoader parses natively.

Pure numpy; no ROS imports and no Open3D needed to write either format.
"""
from typing import Tuple

import numpy as np

_PCD_HEADER = """# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z rgb
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {n}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {n}
DATA binary
"""


def pack_rgb(rgb: np.ndarray) -> np.ndarray:
    """Pack (N,3) uint8 into (N,) float32 carrying 0x00RRGGBB, the PCL convention."""
    rgb = np.asarray(rgb, dtype=np.uint8).reshape(-1, 3).astype(np.uint32)
    packed = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
    # View, not cast: the float32 field must carry the integer's exact bit
    # pattern. Casting would convert the numeric value and destroy the colour.
    return packed.astype(np.uint32).view(np.float32)


def _check(xyz: np.ndarray, rgb: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    rgb = np.asarray(rgb, dtype=np.uint8).reshape(-1, 3)
    if xyz.shape[0] != rgb.shape[0]:
        raise ValueError('xyz and rgb length mismatch: {} vs {}'.format(
            xyz.shape[0], rgb.shape[0]))
    return xyz, rgb


def write_pcd_rgb(path: str, xyz: np.ndarray, rgb: np.ndarray) -> None:
    """Write a binary PCD with the PCL `x y z rgb` layout (16 bytes per point)."""
    xyz, rgb = _check(xyz, rgb)
    n = xyz.shape[0]
    record = np.empty((n, 4), dtype=np.float32)
    record[:, :3] = xyz
    record[:, 3] = pack_rgb(rgb)
    with open(path, 'wb') as fh:
        fh.write(_PCD_HEADER.format(n=n).encode('ascii'))
        fh.write(record.tobytes(order='C'))


def write_ply(path: str, xyz: np.ndarray, rgb: np.ndarray) -> None:
    """Write a binary little-endian PLY with float xyz and uchar rgb per vertex.

    15 bytes per point. Written directly rather than via Open3D so saving a
    cloud needs nothing beyond numpy.
    """
    xyz, rgb = _check(xyz, rgb)
    n = xyz.shape[0]
    record = np.empty(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])
    record['x'], record['y'], record['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    record['red'], record['green'], record['blue'] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    header = ('ply\nformat binary_little_endian 1.0\n'
              'element vertex {}\n'
              'property float x\nproperty float y\nproperty float z\n'
              'property uchar red\nproperty uchar green\nproperty uchar blue\n'
              'end_header\n').format(n)
    with open(path, 'wb') as fh:
        fh.write(header.encode('ascii'))
        fh.write(record.tobytes())


def write_both(stem: str, xyz: np.ndarray, rgb: np.ndarray) -> Tuple[str, str]:
    """Write `<stem>.ply` and `<stem>.pcd`; returns both paths."""
    ply, pcd = stem + '.ply', stem + '.pcd'
    write_ply(ply, xyz, rgb)
    write_pcd_rgb(pcd, xyz, rgb)
    return ply, pcd
