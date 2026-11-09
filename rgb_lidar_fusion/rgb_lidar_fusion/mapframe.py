"""The `map` frame: gravity-levelled, floor at z = 0, and robot export frames.

The rig carries its lidar tilted (45 deg on the reference mount), and lidar
odometry's world frame (`camera_init` for FAST-LIO) is just the lidar's first
pose, so it is tilted too. Saved maps are re-expressed in a frame that robots
and people expect:

  * +Z up, from the accelerometer while the rig is held still,
  * +X forward: the lidar's +X axis at the start, flattened onto the floor,
  * origin on the floor directly below the lidar at the start, so the lidar
    starts at (0, 0, lidar_height).

A robot export moves the origin up to that robot's lidar height, as if the
robot's lidar had started where the rig's lidar did: p_robot = p_map - (0, 0, h),
optionally flipped for a lidar whose driver reports it upside down.

All 4x4 matrices are parent <- child (they map child-frame points into the
parent frame). Pure numpy, no ROS.
"""
import numpy as np

# Below this, the lidar X axis is too close to vertical to define "forward".
_MIN_FORWARD = 0.2


def gravity_up(acc: np.ndarray) -> np.ndarray:
    """Unit 'up' vector from accelerometer samples (N,3) taken at rest.

    A resting accelerometer measures the reaction to gravity, which points UP,
    whatever its units (the Mid-360's IMU reports g, others m/s^2).
    """
    a = np.asarray(acc, dtype=float).reshape(-1, 3).mean(axis=0)
    n = np.linalg.norm(a)
    if n < 1e-6:
        raise ValueError('accelerometer mean is zero; no gravity to level with')
    return a / n


def level_frame(up: np.ndarray, forward_hint: np.ndarray, origin: np.ndarray,
                height: float) -> np.ndarray:
    """T_map_world: world points -> the levelled map frame.

    `up`, `forward_hint` and `origin` are expressed in the WORLD frame (e.g.
    camera_init): up from `gravity_up` rotated into the world, forward_hint the
    lidar's +X axis there, origin the lidar's position there.
    """
    up = np.asarray(up, dtype=float)
    up = up / np.linalg.norm(up)
    f = np.asarray(forward_hint, dtype=float)
    f = f - np.dot(f, up) * up               # flatten onto the floor plane
    if np.linalg.norm(f) < _MIN_FORWARD:
        raise ValueError('lidar X axis is nearly vertical; cannot define forward')
    f = f / np.linalg.norm(f)
    left = np.cross(up, f)
    R = np.stack([f, left, up])               # rows: map axes in world coords
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = -R @ np.asarray(origin, dtype=float) + np.array([0.0, 0.0, float(height)])
    return T


def robot_frame(height: float, flip: bool = False) -> np.ndarray:
    """T_robot_map for a robot lidar at `height` above the map's floor origin.

    flip=True gives a lidar reported upside down (180 deg about X): z down,
    floor at z = +height. Most drivers correct an inverted mount themselves
    (e.g. livox_ros_driver2 with roll 180), in which case leave it False.
    """
    T = np.eye(4)
    T[2, 3] = -float(height)
    if flip:
        F = np.diag([1.0, -1.0, -1.0, 1.0])
        T = F @ T
    return T


def apply(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Transform (N,3) points by a 4x4."""
    pts = np.asarray(pts, dtype=float).reshape(-1, 3)
    return pts @ T[:3, :3].T + T[:3, 3]


def rotation_angle(R0: np.ndarray, R1: np.ndarray) -> float:
    """Angle in radians of the rotation taking R0 to R1."""
    c = (np.trace(np.asarray(R0).T @ np.asarray(R1)) - 1.0) / 2.0
    return float(np.arccos(np.clip(c, -1.0, 1.0)))


def thin_path(xyz: np.ndarray, min_step: float = 0.05) -> np.ndarray:
    """Keep a path point only once it is `min_step` from the last kept one.

    Odometry jitters by a few mm per scan even when the rig is still; summing
    raw steps turns that noise into metres "walked". The last point is always
    kept so the path ends where the rig is.
    """
    xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
    if len(xyz) < 2:
        return xyz
    keep = [0]
    for i in range(1, len(xyz)):
        if np.linalg.norm(xyz[i] - xyz[keep[-1]]) >= min_step:
            keep.append(i)
    if keep[-1] != len(xyz) - 1:
        keep.append(len(xyz) - 1)
    return xyz[keep]


def path_length(xyz: np.ndarray, min_step: float = 0.05) -> float:
    """Distance walked along a path, ignoring jitter below `min_step`."""
    p = thin_path(xyz, min_step)
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()) if len(p) > 1 else 0.0
