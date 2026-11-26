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
_COMPACT_THRESHOLD = 100_000        # new voxels buffered before they are inserted into the map


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
        # New batches wait in a list and are merged once, at compaction: adding a
        # scan must not copy the whole map (that is O(map) per scan).
        self._pending = []
        self._pending_n = 0
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

        # Settle every voxel the map already holds right away (cheap: one
        # lookup per point of this scan) and buffer only the new voxels, so
        # compaction never has to re-scan a scan's repeats.
        k, x, c, q = self._best_per_voxel(self._pack(idx), pts.astype(np.float32), rgb, q)
        new = self._update_stored(k, x, c, q)
        if new.any():
            self._pending.append((k[new], x[new], c[new], q[new]))
            self._pending_n += int(new.sum())
            if self._pending_n > _COMPACT_THRESHOLD:
                self._compact()

    @property
    def approx_count(self) -> int:
        """Occupied voxels without compacting: stored plus buffered new voxels
        (a voxel buffered twice before compaction counts twice). For live
        status, where `count` would force a compaction."""
        return int(self._keys.shape[0]) + self._pending_n

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

    # The map is kept sorted by voxel key and is never re-sorted whole:
    # re-sorting it at every compaction made long recordings slow down as the
    # map grew (2 M voxels took ~2.4 s, and the live map asked for it every
    # second). The winners are the same as a full re-sort would pick: the
    # highest quality, and on equal quality the earlier observation (stored
    # before buffered, buffered in arrival order).

    @staticmethod
    def _best_per_voxel(k, x, c, q):
        """One observation per voxel of a batch, sorted by key: the highest
        quality; lexsort is stable, so equal qualities keep arrival order."""
        order = np.lexsort((-q, k))
        pick = order[_first_of_runs(k[order])]
        return k[pick], x[pick], c[pick], q[pick]

    def _update_stored(self, k, x, c, q) -> np.ndarray:
        """Give stored voxels a strictly better view from (sorted, unique) `k`;
        returns the mask of keys the map does not hold yet."""
        pos, hit = _locate(self._keys, k)
        better = np.zeros_like(hit)
        better[hit] = q[hit] > self._q[pos[hit]]
        at = pos[better]
        self._xyz[at], self._rgb[at], self._q[at] = x[better], c[better], q[better]
        return ~hit

    def _compact(self) -> None:
        """Insert the buffered new voxels into the map, one winner per voxel."""
        if not self._pending:
            return
        k, x, c, q = (np.concatenate(a) for a in zip(*self._pending))
        self._pending, self._pending_n = [], 0
        k, x, c, q = self._best_per_voxel(k, x, c, q)
        new = self._update_stored(k, x, c, q)
        if new.any():
            at = np.searchsorted(self._keys, k[new])
            self._keys = np.insert(self._keys, at, k[new])
            self._xyz = _insert_rows(self._xyz, at, x[new])
            self._rgb = _insert_rows(self._rgb, at, c[new])
            self._q = np.insert(self._q, at, q[new])


def _first_of_runs(sorted_keys: np.ndarray) -> np.ndarray:
    """Indices of the first element of each run of equal keys in a sorted array."""
    if sorted_keys.shape[0] == 0:
        return np.zeros(0, dtype=np.int64)
    return np.flatnonzero(np.r_[True, sorted_keys[1:] != sorted_keys[:-1]])


def _insert_rows(a: np.ndarray, at: np.ndarray, rows: np.ndarray) -> np.ndarray:
    """`np.insert(a, at, rows, axis=0)` for an (N, k) array, done on a 1-D view
    of whole rows: the axis=0 insert is ~7x slower on large maps."""
    a = np.ascontiguousarray(a)
    rows = np.ascontiguousarray(rows, dtype=a.dtype)
    row = np.dtype((np.void, a.dtype.itemsize * a.shape[1]))
    out = np.insert(a.view(row).ravel(), at, rows.view(row).ravel())
    return out.view(a.dtype).reshape(-1, a.shape[1])


def _locate(map_keys: np.ndarray, keys: np.ndarray):
    """Where sorted unique `keys` sit in the sorted map: `(pos, hit)`, with
    `hit` True where the key is already in the map (at `pos`); otherwise `pos`
    is where it must be inserted to keep the map sorted."""
    pos = np.searchsorted(map_keys, keys)
    hit = pos < map_keys.shape[0]
    hit[hit] = map_keys[pos[hit]] == keys[hit]
    return pos, hit


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


class VoxelGrid:
    """Uncoloured companion to `Accumulator`: one lidar point per voxel, with its
    intensity, for the FULL lidar map.

    The camera sees only part of the lidar's field of view, so the coloured map
    alone discards most of the geometry. This keeps everything the lidar saw.
    The earliest observation of a voxel is kept (there is no quality score
    without a camera). Like `Accumulator`, repeats of stored voxels are settled
    when a scan is added and only new voxels are buffered, so the whole map is
    never re-sorted.
    """

    def __init__(self, voxel: float = 0.03):
        if voxel <= 0:
            raise ValueError('voxel must be positive, got {!r}'.format(voxel))
        self.voxel = float(voxel)
        self._keys = np.zeros(0, dtype=np.int64)
        self._xyz = np.zeros((0, 3), dtype=np.float32)
        self._i = np.zeros(0, dtype=np.float32)
        self._pending = []
        self._pending_n = 0
        self.dropped_out_of_range = 0

    def add(self, pts: np.ndarray, intensity: np.ndarray) -> None:
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        inten = np.asarray(intensity, dtype=np.float32).ravel()
        if pts.shape[0] != inten.shape[0]:
            raise ValueError('add() got {} points but {} intensities'.format(
                pts.shape[0], inten.shape[0]))
        if pts.shape[0] == 0:
            return
        idx = np.floor(pts / self.voxel).astype(np.int64)
        ok = np.all((idx >= -_OFFSET) & (idx < _OFFSET), axis=1)
        if not ok.all():
            self.dropped_out_of_range += int((~ok).sum())
            idx, pts, inten = idx[ok], pts[ok], inten[ok]
            if idx.shape[0] == 0:
                return
        # Voxels the map already holds keep their earlier point: drop them now,
        # and buffer only the first point of each new voxel.
        k, x, i = self._first_per_voxel(Accumulator._pack(idx), pts.astype(np.float32), inten)
        new = ~_locate(self._keys, k)[1]
        if new.any():
            self._pending.append((k[new], x[new], i[new]))
            self._pending_n += int(new.sum())
            if self._pending_n > _COMPACT_THRESHOLD:
                self._compact()

    @property
    def count(self) -> int:
        self._compact()
        return int(self._keys.shape[0])

    @property
    def approx_count(self) -> int:
        """Occupied voxels without compacting (see `Accumulator.approx_count`)."""
        return int(self._keys.shape[0]) + self._pending_n

    def export(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return `(xyz float32 (M,3), intensity float32 (M,))`."""
        self._compact()
        return self._xyz.copy(), self._i.copy()

    def _compact(self) -> None:
        """Merge buffered points into the map (kept sorted by voxel key); only
        voxels not stored yet are added, so the earliest view of a voxel wins."""
        if not self._pending:
            return
        k, x, i = (np.concatenate(a) for a in zip(*self._pending))
        self._pending, self._pending_n = [], 0
        k, x, i = self._first_per_voxel(k, x, i)
        pos, hit = _locate(self._keys, k)
        new = ~hit
        if new.any():
            at = pos[new]
            self._keys = np.insert(self._keys, at, k[new])
            self._xyz = _insert_rows(self._xyz, at, x[new])
            self._i = np.insert(self._i, at, i[new])

    @staticmethod
    def _first_per_voxel(k, x, i):
        """The earliest point of each voxel, sorted by key (np.unique's
        return_index is the first occurrence)."""
        _, first = np.unique(k, return_index=True)
        return k[first], x[first], i[first]
