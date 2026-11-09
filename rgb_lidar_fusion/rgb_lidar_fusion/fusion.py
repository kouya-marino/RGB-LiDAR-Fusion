"""Voxel accumulation of coloured points, fusing colour by best-view-wins.

Colour is NOT averaged across viewpoints. Auto-exposure differences, varying
incidence angle and varying range mean two observations of the same wall
disagree; averaging them yields mud and soft edges. Keeping the single best
observation per voxel keeps edges crisp, and makes `add` order-independent.

Pure numpy -- no ROS imports.
"""
from typing import Optional, Tuple

import numpy as np

# Voxel indices are packed into one int64 as three 21-bit fields. 21 bits gives
# +/-2^20 voxels per axis, i.e. +/-31 km at a 3 cm voxel -- far beyond any room,
# and collision-free by construction rather than by hoping a hash doesn't clash.
_BITS = 21
_OFFSET = 1 << (_BITS - 1)          # 2^20, maps signed index into [0, 2^21)
_LIMIT = 1 << _BITS
_COMPACT_THRESHOLD = 4_000_000      # observations buffered before forced compaction


class Accumulator:
    """Accumulates (position, colour, quality) into a voxel grid, best view wins.

    Deliberately batch/vectorised rather than a per-point dict loop: the
    Mid-360 delivers ~20k points per scan at 10 Hz, so a Python-level loop over
    points would be over a hundred million iterations in a 10-minute run.
    """

    def __init__(self, voxel: float = 0.03):
        if voxel <= 0:
            raise ValueError('voxel must be positive, got {!r}'.format(voxel))
        self.voxel = float(voxel)
        self._keys = np.zeros(0, dtype=np.int64)
        self._xyz = np.zeros((0, 3), dtype=np.float32)
        self._rgb = np.zeros((0, 3), dtype=np.uint8)
        self._q = np.zeros(0, dtype=np.float32)
        self._dirty = False
        self.dropped_out_of_range = 0

    # --- public API -----------------------------------------------------

    def add(self, pts_world: np.ndarray, rgb: np.ndarray,
            quality: np.ndarray) -> None:
        """Fold one batch of coloured observations in. Positions are world-frame."""
        pts = np.asarray(pts_world, dtype=np.float64).reshape(-1, 3)
        if pts.shape[0] == 0:
            return
        rgb = np.asarray(rgb, dtype=np.uint8).reshape(-1, 3)
        q = np.asarray(quality, dtype=np.float32).ravel()
        if not (pts.shape[0] == rgb.shape[0] == q.shape[0]):
            raise ValueError(
                'add() got mismatched lengths: pts={}, rgb={}, quality={}'.format(
                    pts.shape[0], rgb.shape[0], q.shape[0]))

        idx = np.floor(pts / self.voxel).astype(np.int64)
        # Drop rather than wrap anything outside the representable grid: a
        # wrapped index would quietly place a point on the far side of the room.
        ok = np.all((idx >= -_OFFSET) & (idx < _OFFSET), axis=1)
        if not ok.all():
            self.dropped_out_of_range += int((~ok).sum())
            idx, pts, rgb, q = idx[ok], pts[ok], rgb[ok], q[ok]
            if idx.shape[0] == 0:
                return

        keys = self._pack(idx)
        self._keys = np.concatenate([self._keys, keys])
        self._xyz = np.concatenate([self._xyz, pts.astype(np.float32)])
        self._rgb = np.concatenate([self._rgb, rgb])
        self._q = np.concatenate([self._q, q])
        self._dirty = True

        if self._keys.shape[0] > _COMPACT_THRESHOLD:
            self._compact()

    @property
    def count(self) -> int:
        """Number of occupied voxels."""
        self._compact()
        return int(self._keys.shape[0])

    def export(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return `(xyz float32 (M,3), rgb uint8 (M,3))` of the winning views.

        Non-destructive and repeatable -- the colorizer calls this periodically
        to republish the live cloud as well as at save time.
        """
        self._compact()
        return self._xyz.copy(), self._rgb.copy()

    # --- internals ------------------------------------------------------

    @staticmethod
    def _pack(idx: np.ndarray) -> np.ndarray:
        shifted = idx + _OFFSET
        return ((shifted[:, 0] << (2 * _BITS))
                | (shifted[:, 1] << _BITS)
                | shifted[:, 2])

    def _compact(self) -> None:
        """Collapse buffered observations to one winner per voxel."""
        if not self._dirty:
            return
        # Sort by (key asc, quality desc) so the first row of each key group is
        # that voxel's best view; np.unique then hands us exactly those rows.
        order = np.lexsort((-self._q, self._keys))
        keys_sorted = self._keys[order]
        _, first = np.unique(keys_sorted, return_index=True)
        winners = order[first]
        self._keys = self._keys[winners]
        self._xyz = self._xyz[winners]
        self._rgb = self._rgb[winners]
        self._q = self._q[winners]
        self._dirty = False


def view_quality(pts_cam: np.ndarray, uv: np.ndarray,
                 cam, max_range: Optional[float] = None) -> np.ndarray:
    """Score how good a look this camera got at each point, in [0, 1].

    Two factors, multiplied:
      * range      -- nearer points subtend more pixels, so their colour is
                      sharper and less likely to be mixed across a depth edge.
      * centrality -- the D435's colour corners are its softest region, and
                      residual distortion error grows with radius.

    Incidence angle would be a third natural factor but needs per-point normals,
    which raw lidar scans do not carry; range and centrality alone already
    separate good views from bad ones.
    """
    pts = np.asarray(pts_cam, dtype=float).reshape(-1, 3)
    uv = np.asarray(uv, dtype=float).reshape(-1, 2)
    if pts.shape[0] == 0:
        return np.zeros(0, dtype=np.float32)

    z = np.maximum(pts[:, 2], 0.0)
    range_term = 1.0 / (1.0 + z)

    cx, cy = cam.K[0, 2], cam.K[1, 2]
    # Normalise by the distance from the principal point to the furthest corner,
    # so r_norm reaches ~1 at the worst pixel in the image.
    corners = np.array([[0.0, 0.0], [cam.width - 1.0, 0.0],
                        [0.0, cam.height - 1.0], [cam.width - 1.0, cam.height - 1.0]])
    r_max = float(np.max(np.hypot(corners[:, 0] - cx, corners[:, 1] - cy)))
    r_max = r_max if r_max > 0 else 1.0
    r_norm = np.hypot(uv[:, 0] - cx, uv[:, 1] - cy) / r_max
    centre_term = 1.0 - np.clip(r_norm, 0.0, 1.0)

    q = range_term * centre_term
    if max_range is not None:
        q = np.where(z <= max_range, q, 0.0)
    return np.clip(q, 0.0, 1.0).astype(np.float32)
