"""SE(3) helpers and a human-editable extrinsic file format.

Every 4x4 here maps points from a child frame into a parent frame, e.g.
`T_lidar_cam` maps a point expressed in the camera optical frame into the lidar
frame. Colorization needs the other direction, so use `invert()`.

Frames used in this project:
  * lidar  -- `livox_frame`: +X forward, +Y left, +Z up (the Mid-360's axes).
  * camera -- `camera_color_optical_frame`, ROS optical convention:
              +Z forward out of the lens, +X right, +Y down.

The live colorizer reads the lidar->camera transform from TF; these helpers are
for tools that edit or save an extrinsic (e.g. calibration). Pure numpy -- no
scipy and no ROS imports -- so this is unit-testable without hardware.
"""
from typing import List, Sequence, Tuple

import numpy as np
import yaml

# Below this, cos(pitch) is treated as zero and roll/yaw stop being separable.
_GIMBAL_EPS = 1e-9


def se3(rpy_deg: Sequence[float], xyz: Sequence[float]) -> np.ndarray:
    """Build a 4x4 SE(3) matrix from roll/pitch/yaw in DEGREES and a translation.

    Rotation order is EXTRINSIC xyz -- roll about fixed X, then pitch about fixed
    Y, then yaw about fixed Z, i.e. R = Rz(yaw) @ Ry(pitch) @ Rx(roll). That is
    the ROS `static_transform_publisher` / tf2 convention, so values here can be
    pasted into a launch file unchanged.
    """
    r, p, y = np.radians(np.asarray(rpy_deg, dtype=float).ravel()[:3])
    sr, cr = np.sin(r), np.cos(r)
    sp, cp = np.sin(p), np.cos(p)
    sy, cy = np.sin(y), np.cos(y)

    T = np.eye(4)
    T[:3, :3] = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    T[:3, 3] = np.asarray(xyz, dtype=float).ravel()[:3]
    return T


def to_rpy_xyz(T: np.ndarray) -> Tuple[List[float], List[float]]:
    """Inverse of `se3`: recover (rpy_deg, xyz) from a 4x4."""
    R = np.asarray(T, dtype=float)[:3, :3]
    sp = -np.clip(R[2, 0], -1.0, 1.0)
    cp = np.sqrt(max(0.0, 1.0 - sp * sp))
    if cp > _GIMBAL_EPS:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arcsin(sp)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock: pitch = +/-90 leaves only (roll -/+ yaw) observable, so
        # pin yaw at 0 and fold the whole rotation into roll. A tuner nudging
        # pitch can drive it there, so it must not crash.
        pitch = np.pi / 2 if sp > 0 else -np.pi / 2
        yaw = 0.0
        roll = (np.arctan2(R[0, 1], R[0, 2]) if sp > 0
                else np.arctan2(-R[0, 1], -R[0, 2]))
    return (np.degrees([roll, pitch, yaw]).tolist(),
            np.asarray(T, dtype=float)[:3, 3].tolist())


def invert(T: np.ndarray) -> np.ndarray:
    """Invert an SE(3) matrix analytically.

    Built from R.T and -R.T @ t rather than np.linalg.inv: for a rigid transform
    that is exact and cheap, where a general inverse would introduce numerical
    error and could silently return something non-rigid if T were malformed.
    """
    T = np.asarray(T, dtype=float)
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def save_extrinsic(path: str, T: np.ndarray, notes: str = '') -> None:
    """Write the extrinsic as human-editable rpy_deg / xyz_m.

    Deliberately NOT a raw 16-number matrix: the interactive tuner nudges these
    six numbers, and a human needs to be able to read and sanity-check them.
    """
    rpy, xyz = to_rpy_xyz(T)
    lines = ['# lidar -> camera_color_optical_frame extrinsic (T_lidar_cam).',
             '# lidar = livox_frame (+X fwd, +Y left, +Z up).',
             '# camera = ROS optical convention (+Z fwd, +X right, +Y down).']
    for line in (notes or '').splitlines():
        lines.append('# {}'.format(line))
    lines.append('')
    # Round and emit as inline flow sequences. Both are for the human: the Euler
    # round-trip otherwise leaves values like -89.99999999999999 in a file whose
    # whole point is to be legible and hand-nudged. 9 decimal places is ~3e-6
    # arcsec, far below anything measurable, so nothing real is lost.
    lines.append('rpy_deg: [{}]'.format(', '.join(_num(v) for v in rpy)))
    lines.append('xyz_m: [{}]'.format(', '.join(_num(v) for v in xyz)))
    with open(path, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')


def _num(v: float) -> str:
    """Format a float for the YAML: rounded, and never in exponent notation."""
    return repr(round(float(v) + 0.0, 9))


def load_extrinsic(path: str) -> np.ndarray:
    """Read an extrinsic YAML into a 4x4 (parent <- child) transform.

    Raises KeyError if either key is absent -- a half-written extrinsic must fail
    loudly here rather than silently colouring everything from the wrong pose.
    """
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    for key in ('rpy_deg', 'xyz_m'):
        if key not in data:
            raise KeyError(
                "extrinsic file {!r} is missing required key {!r}".format(path, key))
    return se3(data['rpy_deg'], data['xyz_m'])


def matrix_to_quat(R: np.ndarray):
    """3x3 rotation -> quaternion (x, y, z, w), robust near 180 degrees."""
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, s / 4)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(max(0.0, 1.0 + R[i, i] - R[j, j] - R[k, k])) * 2
    q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
    q[i] = s / 4
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    return tuple(q)
