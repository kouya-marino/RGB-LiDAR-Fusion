"""Targetless lidar -> camera calibration by edge alignment.

At the right extrinsic a depth discontinuity in the lidar cloud (an object's
outline) projects onto an intensity discontinuity in the image, because both
are the same physical edge. Away from it, depth edges land on flat image areas.
The score is the correlation between the two edge maps, and the rotation is
refined by coarse-to-fine coordinate descent on roll, pitch and yaw.

Needs a STILL rig looking at a scene with strong depth edges 2-4 m away (a
doorway, furniture against a wall): many sweeps are accumulated into one dense
cloud, since a single Mid-360 sweep puts only ~1k points in the camera's view.
Translation is left at its measured/CAD value: at these ranges it is weakly
observable, and tuning it tends to wander into a local optimum.

The extrinsic being refined is T_lidar_opt: the camera's colour OPTICAL frame
in the lidar frame. The search does not move its Euler angles: on a skewed
mount those are strongly coupled, and coordinate descent along them overshoots
and stalls (measured: 1.9 deg left on a synthetic scene). It applies small
rotations about the CAMERA's own axes instead, T = T_seed @ R(tilt, pan, roll):
tilt moves the projection up/down, pan left/right, roll spins it about the view
axis, so each axis has its own, nearly independent effect in the image.

Pure numpy + OpenCV, no ROS: unit-testable on synthetic scenes.
"""
from typing import Callable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from rgb_lidar_fusion.extrinsics import invert, se3
from rgb_lidar_fusion.projection import CameraModel, occlusion_mask, project

# Coarse-to-fine (span_deg, step_deg): wide first, because a taped-on sensor
# or a CAD seed can be a few degrees out; finest step ~ where the score stops
# being informative.
AUTO_SCHEDULE = ((8.0, 1.0), (2.0, 0.25), (0.5, 0.1))
MIN_POINTS = 500


def default_cell(width: int) -> int:
    """Raster cell size in pixels: coarse, since accumulated lidar is still sparse
    per pixel (~6 px at 720p, ~10 px at 1080p)."""
    return max(4, int(round(width / 200)))


def image_edges(rgb: np.ndarray, cam: CameraModel, cell: int) -> np.ndarray:
    """Gradient magnitude of the image, downsampled to the raster grid."""
    rows, cols = (cam.height + cell - 1) // cell, (cam.width + cell - 1) // cell
    gray = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2GRAY)
    small = cv2.resize(gray, (cols, rows), interpolation=cv2.INTER_AREA).astype(np.float32)
    return np.abs(cv2.Sobel(small, cv2.CV_32F, 1, 0, ksize=3)) + \
        np.abs(cv2.Sobel(small, cv2.CV_32F, 0, 1, ksize=3))


def _visible(pts_lidar, cam, T_cam_lidar, min_range, max_range, occl_tol):
    pts_cam = pts_lidar @ T_cam_lidar[:3, :3].T + T_cam_lidar[:3, 3]
    d = pts_cam[:, 2]
    sel = (d > min_range) & (d < max_range)
    pts_cam = pts_cam[sel]
    uv, valid = project(cam, pts_cam)
    uv, pts_cam = uv[valid], pts_cam[valid]
    if uv.shape[0] == 0:
        return uv, np.zeros(0)
    keep = occlusion_mask(uv, pts_cam[:, 2], (cam.height, cam.width), tol=occl_tol)
    return uv[keep], pts_cam[keep, 2]


def alignment_score(pts_lidar: np.ndarray, edges: Sequence[np.ndarray], cam: CameraModel,
                    T_cam_lidar: np.ndarray, cell: int, min_range: float = 0.5,
                    max_range: float = 10.0, occl_tol: float = 0.10) -> Tuple[float, int]:
    """Mean correlation of depth edges with image edges over `edges` (one per
    frame), and the number of lidar points used. NaN if too little overlap."""
    uv, d = _visible(pts_lidar, cam, T_cam_lidar, min_range, max_range, occl_tol)
    if uv.shape[0] < MIN_POINTS:
        return float('nan'), int(uv.shape[0])
    rows, cols = (cam.height + cell - 1) // cell, (cam.width + cell - 1) // cell
    cy = np.clip((uv[:, 1] // cell).astype(np.int64), 0, rows - 1)
    cx = np.clip((uv[:, 0] // cell).astype(np.int64), 0, cols - 1)
    nearest = np.full(rows * cols, np.inf)
    np.minimum.at(nearest, cy * cols + cx, d)
    mask = np.isfinite(nearest).reshape(rows, cols)
    depth = nearest.reshape(rows, cols)
    filled = np.where(mask, depth, np.median(depth[mask])).astype(np.float32)
    gd = np.abs(cv2.Sobel(filled, cv2.CV_32F, 1, 0, ksize=3)) + \
        np.abs(cv2.Sobel(filled, cv2.CV_32F, 0, 1, ksize=3))
    # cells on the rim of the lidar coverage have fake depth gradients
    m = cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    if m.sum() < 200:
        return float('nan'), int(uv.shape[0])
    a = gd[m] - gd[m].mean()
    scores = []
    for gi in edges:
        b = gi[m] - gi[m].mean()
        denom = float(np.linalg.norm(a) * np.linalg.norm(b))
        if denom > 0:
            scores.append(float(np.dot(a, b) / denom))
    return (float(np.mean(scores)) if scores else float('nan')), int(uv.shape[0])


AXES = ('tilt', 'pan', 'roll')          # about camera optical x, y, z


def camera_delta(T_seed: np.ndarray, delta_deg: Sequence[float]) -> np.ndarray:
    """T_seed rotated by (tilt, pan, roll) degrees about its own optical axes."""
    return T_seed @ se3(delta_deg, [0.0, 0.0, 0.0])


def auto_refine(score: Callable[[List[float]], float], schedule=AUTO_SCHEDULE,
                max_passes: int = 4, log: Optional[Callable[[str], None]] = None,
                bound: float = 5.0):
    """Coordinate descent on (tilt, pan, roll), starting at zero. Each round
    repeats until no axis moves (the axes still interact a little). Returns
    (delta_deg, start_score, final_score, edge_hits): edge_hits lists axes whose
    optimum sat at the edge of the final search span, i.e. it may lie outside,
    or at `bound`: no axis may move more than `bound` degrees in total, a
    physical limit for a sensor on a fixed mount (repeated passes could
    otherwise walk the solution arbitrarily far on a bad capture)."""
    log = log or (lambda _s: None)
    delta = [0.0, 0.0, 0.0]
    start = score(delta)
    log('start score {:.4f}'.format(start))
    edge_hits = []
    for rnd, (span, step) in enumerate(schedule, 1):
        offsets = np.arange(-span, span + 1e-9, step)
        for _ in range(max_passes):
            moved, hits = False, []
            for i, axis in enumerate(AXES):
                base = delta[i]
                vals = []
                for off in offsets:
                    trial = list(delta)
                    trial[i] = base + float(off)
                    vals.append(score(trial) if abs(trial[i]) <= bound + 1e-9 else np.nan)
                vals = np.array(vals)
                if not np.isfinite(vals).any():
                    continue
                k = int(np.nanargmax(vals))
                if offsets[k] != 0.0:
                    delta[i] = base + float(offsets[k])
                    moved = True
                if abs(offsets[k]) >= span - 1e-9:
                    hits.append(axis)
                log('  round {} {:<4} {:+.2f} deg -> score {:.4f}'.format(
                    rnd, axis, offsets[k], vals[k]))
            if not moved:
                break
        if rnd == len(schedule):
            edge_hits = hits
    edge_hits += ['{} at the {:.0f} deg limit'.format(a, bound)
                  for a, v in zip(AXES, delta) if abs(v) >= bound - 1e-6]
    final = score(delta)
    return delta, start, final, edge_hits


def render_overlay(rgb: np.ndarray, pts_lidar: np.ndarray, cam: CameraModel,
                   T_cam_lidar: np.ndarray, min_range: float = 0.5, max_range: float = 10.0,
                   occl_tol: float = 0.10, label: str = '') -> np.ndarray:
    """The image (BGR) with lidar points drawn on it, coloured by depth. Depth
    boundaries should sit exactly on object boundaries when calibrated."""
    canvas = np.ascontiguousarray(rgb[:, :, ::-1]).copy()
    canvas = (canvas * 0.6).astype(np.uint8)            # dim, so points stand out
    uv, d = _visible(pts_lidar, cam, T_cam_lidar, min_range, max_range, occl_tol)
    if d.size:
        lo, hi = float(np.percentile(d, 2)), float(np.percentile(d, 98))
        if hi - lo < 2.0:                               # flat scenes: keep contrast sane
            mid = 0.5 * (lo + hi)
            lo, hi = mid - 1.0, mid + 1.0
        t = np.clip((d - lo) / (hi - lo), 0, 1)
        colors = cv2.applyColorMap((t * 255).astype(np.uint8).reshape(-1, 1),
                                   cv2.COLORMAP_TURBO).reshape(-1, 3)
        u = np.clip(np.rint(uv[:, 0]).astype(int), 0, cam.width - 1)
        v = np.clip(np.rint(uv[:, 1]).astype(int), 0, cam.height - 1)
        for du in (0, 1):                               # 2x2 px dots, no Python loop
            for dv in (0, 1):
                canvas[np.clip(v + dv, 0, cam.height - 1),
                       np.clip(u + du, 0, cam.width - 1)] = colors
    if label:
        cv2.putText(canvas, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (255, 255, 255), 2, cv2.LINE_AA)
    return canvas


def prefilter(pts_lidar: np.ndarray, cam: CameraModel, T_cam_lidar: np.ndarray,
              margin: float = 0.25, max_range: float = 10.0) -> np.ndarray:
    """Keep only points that could land in the image under a few degrees of
    extrinsic error: makes each score evaluation much cheaper."""
    pts_cam = pts_lidar @ T_cam_lidar[:3, :3].T + T_cam_lidar[:3, 3]
    z = pts_cam[:, 2]
    ok = (z > 0.2) & (z < max_range)
    fx, fy, cx, cy = cam.K[0, 0], cam.K[1, 1], cam.K[0, 2], cam.K[1, 2]
    zs = np.where(ok, z, 1.0)
    u = fx * pts_cam[:, 0] / zs + cx
    v = fy * pts_cam[:, 1] / zs + cy
    mu, mv = margin * cam.width, margin * cam.height
    ok &= (u > -mu) & (u < cam.width + mu) & (v > -mv) & (v < cam.height + mv)
    return pts_lidar[ok]


def voxel_downsample(pts: np.ndarray, voxel: float) -> np.ndarray:
    """One point per voxel (the first): many still sweeps -> one even cloud."""
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
    if pts.shape[0] == 0:
        return pts
    keys = np.floor(pts / voxel).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(first)]


def static_points(pts: np.ndarray, chunk: np.ndarray, voxel: float = 0.05,
                  min_chunks: Optional[int] = None) -> np.ndarray:
    """Keep points whose `voxel` cell is occupied in at least `min_chunks` of the
    capture's time slices (default: all of them).

    A person walking through the scene occupies any spot only briefly, so their
    points fail this test; walls, furniture and the floor pass. Without it,
    moving people pull the edge alignment towards their outline.
    """
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
    chunk = np.asarray(chunk).ravel()
    n_chunks = int(chunk.max()) + 1 if chunk.size else 0
    need = n_chunks if min_chunks is None else int(min_chunks)
    if n_chunks <= 1 or pts.shape[0] == 0:
        return pts
    keys = np.floor(pts / voxel).astype(np.int64)
    _, cell = np.unique(keys, axis=0, return_inverse=True)
    cell = cell.ravel()
    seen = np.zeros((cell.max() + 1, n_chunks), dtype=bool)
    seen[cell, chunk.astype(np.int64)] = True
    return pts[seen.sum(axis=1)[cell] >= need]


def drop_floor(pts: np.ndarray, up: np.ndarray, clearance: float = 0.10) -> np.ndarray:
    """Remove points within `clearance` of the floor (the lowest surface along
    `up`, from the IMU). With the camera pitched down, the floor fills much of
    the image; its texture has no depth edges and only adds noise to the score.
    Dropping it also removes those cells from the comparison entirely."""
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
    if pts.shape[0] == 0:
        return pts
    up = np.asarray(up, dtype=float)
    h = pts @ (up / np.linalg.norm(up))
    floor = np.percentile(h, 1.0)
    return pts[h > floor + clearance]


def median_image(images: Sequence[np.ndarray]) -> np.ndarray:
    """Per-pixel median of several frames: anything that moves through the view
    (people) is replaced by the static background behind it."""
    stack = np.stack([np.asarray(im) for im in images])
    return np.median(stack, axis=0).astype(np.uint8)


def T_cam_from_rpy(rpy: Sequence[float], xyz: Sequence[float]) -> np.ndarray:
    """T_cam_lidar from the refined T_lidar_opt parameters."""
    return invert(se3(rpy, xyz))
