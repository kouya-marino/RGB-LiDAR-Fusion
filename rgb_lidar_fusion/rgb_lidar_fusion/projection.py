"""Pinhole projection of lidar points into a colour image, plus occlusion rejection.

Deliberately ROS-import-free so it is unit-testable without hardware;
`CameraModel.from_camera_info` only duck-types the fields it needs (`k`, `d`,
`width`, `height`), so a real `sensor_msgs/CameraInfo` works but is not required.

All point arrays are (N,3) float in the CAMERA OPTICAL frame: +Z forward out of
the lens, +X right, +Y down. So `pts[:, 2]` is depth.
"""
from typing import Tuple

import numpy as np

_EPS = 1e-6


class CameraModel:
    """Intrinsics for one camera: K, distortion D, and the image size."""

    __slots__ = ('K', 'D', 'width', 'height', '_has_distortion')

    def __init__(self, K, D, width, height):
        self.K = np.asarray(K, dtype=float).reshape(3, 3)
        D = np.asarray(D, dtype=float).ravel()
        # Pad/trim to the 5-term plumb-bob vector. Some drivers publish an empty
        # D; treating that as "no distortion" is correct and keeps the fast path.
        padded = np.zeros(5)
        padded[:min(5, D.size)] = D[:5]
        self.D = padded
        self.width = int(width)
        self.height = int(height)
        self._has_distortion = bool(np.any(self.D))

    @classmethod
    def from_camera_info(cls, msg) -> 'CameraModel':
        return cls(K=np.asarray(msg.k, dtype=float).reshape(3, 3),
                   D=np.asarray(msg.d, dtype=float),
                   width=msg.width, height=msg.height)

    @property
    def has_distortion(self) -> bool:
        return self._has_distortion

    def __repr__(self):
        return ('CameraModel({}x{}, fx={:.1f}, fy={:.1f}, cx={:.1f}, cy={:.1f}, '
                'distortion={})'.format(self.width, self.height, self.K[0, 0],
                                        self.K[1, 1], self.K[0, 2], self.K[1, 2],
                                        self._has_distortion))


def project(cam: CameraModel, pts_cam: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Project camera-frame points to pixels.

    Returns `(uv, valid)` where `uv` is (N,2) float pixel coordinates and `valid`
    is (N,) bool -- True only for points in front of the camera AND inside the
    image. `uv` is finite everywhere, including for rejected points, so callers
    can index it freely without tripping over NaN.
    """
    pts = np.asarray(pts_cam, dtype=float).reshape(-1, 3)
    n = pts.shape[0]
    if n == 0:
        return np.zeros((0, 2)), np.zeros(0, dtype=bool)

    z = pts[:, 2]
    # Reject non-positive depth BEFORE dividing: this is what keeps the divide
    # from producing inf/NaN (and from raising under np.errstate(all='raise')).
    in_front = z > _EPS
    z_safe = np.where(in_front, z, 1.0)

    xn = pts[:, 0] / z_safe
    yn = pts[:, 1] / z_safe

    if cam.has_distortion:
        xn, yn = _distort(xn, yn, cam.D)

    fx, fy = cam.K[0, 0], cam.K[1, 1]
    cx, cy = cam.K[0, 2], cam.K[1, 2]
    u = fx * xn + cx
    v = fy * yn + cy

    inside = (u >= 0) & (u <= cam.width - 1) & (v >= 0) & (v <= cam.height - 1)
    valid = in_front & inside

    uv = np.empty((n, 2))
    uv[:, 0] = u
    uv[:, 1] = v
    return uv, valid


def _distort(xn: np.ndarray, yn: np.ndarray, D: np.ndarray):
    """Plumb-bob / Brown-Conrady forward distortion on normalised coordinates.

    Skipped entirely when D is all-zero (the D435's factory colour distortion is
    near-zero), which is why `project` gates on `cam.has_distortion`.
    """
    k1, k2, p1, p2, k3 = D
    r2 = xn * xn + yn * yn
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    x_t = xn * radial + 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn)
    y_t = yn * radial + p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn
    return x_t, y_t


def occlusion_mask(uv: np.ndarray, depth: np.ndarray, shape, cell: int = 4,
                   tol: float = 0.10) -> np.ndarray:
    """Reject points hidden behind nearer points that share an image cell.

    The camera and lidar sit at different positions (~10 cm apart on the
    reference mount, much more on some robots), so they disagree about what is
    visible: a surface the lidar sees may be hidden from the camera behind a
    nearer object. Without this, foreground objects paint their colour onto the
    surfaces behind them. The wider the baseline, the more this matters.

    Points are bucketed into `cell` x `cell` pixel cells; within each cell only
    points within `tol` metres of the cell's nearest depth are kept. `tol` needs
    to absorb lidar range noise plus residual extrinsic error and nothing more --
    a loose value is exactly what lets background survive.

    Returns a (N,) bool array, True = keep. Order-independent by construction.
    """
    uv = np.asarray(uv, dtype=float).reshape(-1, 2)
    depth = np.asarray(depth, dtype=float).ravel()
    if uv.shape[0] == 0:
        return np.zeros(0, dtype=bool)

    height, width = int(shape[0]), int(shape[1])
    cell = max(1, int(cell))
    n_cols = (width + cell - 1) // cell
    n_rows = (height + cell - 1) // cell

    cx = np.clip((uv[:, 0] // cell).astype(np.int64), 0, n_cols - 1)
    cy = np.clip((uv[:, 1] // cell).astype(np.int64), 0, n_rows - 1)
    flat = cy * n_cols + cx

    # Per-cell minimum depth via an unbuffered scatter-min.
    nearest = np.full(n_rows * n_cols, np.inf)
    np.minimum.at(nearest, flat, depth)
    return depth <= nearest[flat] + tol
