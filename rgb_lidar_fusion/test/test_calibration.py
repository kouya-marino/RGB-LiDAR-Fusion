"""Edge-alignment calibration on a synthetic scene with a known extrinsic."""
import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from rgb_lidar_fusion.calibration import (  # noqa: E402
    T_cam_from_rpy, alignment_score, auto_refine, camera_delta, coverage_mask, default_cell,
    image_edges, prefilter, render_overlay, voxel_downsample)
from rgb_lidar_fusion.extrinsics import invert  # noqa: E402
from rgb_lidar_fusion.extrinsics import se3, to_rpy_xyz  # noqa: E402
from rgb_lidar_fusion.projection import CameraModel  # noqa: E402

CAM = CameraModel(K=[[450, 0, 320], [0, 450, 180], [0, 0, 1]], D=[0] * 5, width=640, height=360)
TRUE_RPY, TRUE_XYZ = [-65.0, 0.0, -90.0], [0.0842, 0.0325, -0.0669]   # reference mount


def _rect(x0, x1, y0, y1, z, step=0.02):
    xs, ys = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step))
    return np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, z)], 1)


# scene in the camera optical frame (z forward, x right, y down)
BOX = (-0.6, 0.4, -0.5, 0.7, 2.5)
POLE = (1.0, 1.25, -2.0, 2.0, 3.0)
WALL_Z = 4.0


def _scene_points():
    pts = np.vstack([_rect(-3, 3, -2, 2, WALL_Z), _rect(*BOX), _rect(*POLE)])
    T_lidar_cam = se3(TRUE_RPY, TRUE_XYZ)
    return pts @ T_lidar_cam[:3, :3].T + T_lidar_cam[:3, 3]      # into the lidar frame


def _scene_image():
    """What the true camera sees: wall 70, pole 140, box 210 (nearest wins)."""
    u, v = np.meshgrid(np.arange(CAM.width), np.arange(CAM.height))
    xn, yn = (u - 320) / 450.0, (v - 180) / 450.0
    img = np.full((CAM.height, CAM.width), 70, np.uint8)
    for (x0, x1, y0, y1, z), val in ((POLE, 140), (BOX, 210)):   # far to near
        x, y = xn * z, yn * z
        img[(x >= x0) & (x < x1) & (y >= y0) & (y < y1)] = val
    return np.repeat(img[:, :, None], 3, axis=2)


@pytest.fixture(scope='module')
def scene():
    pts, rgb = _scene_points(), _scene_image()
    cell = default_cell(CAM.width)
    return pts, rgb, cell, [image_edges(rgb, CAM, cell)]


def test_score_is_highest_at_the_true_extrinsic(scene):
    pts, _, cell, edges = scene
    true = alignment_score(pts, edges, CAM, T_cam_from_rpy(TRUE_RPY, TRUE_XYZ), cell)[0]
    off = alignment_score(pts, edges, CAM, T_cam_from_rpy(
        [TRUE_RPY[0] + 3, TRUE_RPY[1], TRUE_RPY[2]], TRUE_XYZ), cell)[0]
    assert true > off + 0.05


def test_auto_refine_recovers_a_few_degrees_of_error(scene):
    pts, _, cell, edges = scene
    seed = [TRUE_RPY[0] + 2.0, TRUE_RPY[1] - 1.5, TRUE_RPY[2] + 2.5]
    T_seed = se3(seed, TRUE_XYZ)
    pts_f = prefilter(pts, CAM, invert(T_seed))

    mask = coverage_mask(pts_f, CAM, invert(T_seed), cell)

    def score(delta):
        return alignment_score(pts_f, edges, CAM, invert(camera_delta(T_seed, delta)), cell,
                               mask=mask)[0]

    delta, start, final, edge_hits = auto_refine(score)
    assert final > start
    assert not edge_hits
    # compare as rotations
    R_got = camera_delta(T_seed, delta)[:3, :3]
    R_true = se3(TRUE_RPY, TRUE_XYZ)[:3, :3]
    err = np.degrees(np.arccos(np.clip((np.trace(R_got.T @ R_true) - 1) / 2, -1, 1)))
    assert err < 0.5, 'residual rotation error {:.2f} deg'.format(err)


def test_too_little_overlap_gives_nan(scene):
    pts, _, cell, edges = scene
    behind = T_cam_from_rpy([TRUE_RPY[0], TRUE_RPY[1], TRUE_RPY[2] + 180], TRUE_XYZ)
    assert np.isnan(alignment_score(pts, edges, CAM, behind, cell)[0])


def test_overlay_and_helpers(scene):
    pts, rgb, _, _ = scene
    over = render_overlay(rgb, pts, CAM, T_cam_from_rpy(TRUE_RPY, TRUE_XYZ), label='x')
    assert over.shape == (CAM.height, CAM.width, 3) and over.dtype == np.uint8
    dense = np.vstack([pts, pts])                              # every point twice
    assert voxel_downsample(dense, 0.01).shape[0] <= pts.shape[0]
    rpy, xyz = to_rpy_xyz(se3(TRUE_RPY, TRUE_XYZ))
    np.testing.assert_allclose(xyz, TRUE_XYZ, atol=1e-12)


def test_static_points_drops_a_person_walking_through():
    from rgb_lidar_fusion.calibration import static_points
    wall = _rect(-1, 1, -1, 1, 4.0, step=0.05)
    chunks_pts, chunks_id = [], []
    for c in range(4):                                  # wall seen in every quarter
        person = _rect(-1.0 + 0.6 * c, -0.7 + 0.6 * c, -1, 1, 2.0, step=0.05)  # moves along
        chunks_pts += [wall, person]
        chunks_id += [np.full(len(wall), c), np.full(len(person), c)]
    kept = static_points(np.vstack(chunks_pts), np.concatenate(chunks_id))
    assert np.all(np.abs(kept[:, 2] - 4.0) < 1e-6)      # only the wall survives
    assert len(kept) == 4 * len(wall)


def test_median_image_removes_a_passer_by():
    from rgb_lidar_fusion.calibration import median_image
    bg = np.full((10, 10, 3), 50, np.uint8)
    frames = [bg.copy() for _ in range(5)]
    frames[1][2:6, 2:6] = 255                           # someone in two frames,
    frames[3][4:8, 4:8] = 255                           # in different places
    np.testing.assert_array_equal(median_image(frames), bg)


def test_drop_floor_keeps_objects():
    from rgb_lidar_fusion.calibration import drop_floor
    up = np.array([0.0, 0.0, 1.0])
    floor = np.c_[np.random.RandomState(0).uniform(-3, 3, (500, 2)), np.full(500, -0.7)]
    box = np.c_[np.random.RandomState(1).uniform(-0.5, 0.5, (100, 2)),
                np.random.RandomState(2).uniform(-0.6, 0.3, 100)]
    kept = drop_floor(np.vstack([floor, box]), up)
    assert kept[:, 2].min() > -0.6 - 1e-6          # floor gone, box above 10 cm stays
    assert len(kept) >= 85


def test_auto_refine_never_moves_beyond_the_bound():
    # a score that keeps rising forever along tilt: the bound must stop it
    delta, _, _, hits = auto_refine(lambda d: d[0], bound=5.0)
    assert abs(delta[0]) <= 5.0 + 1e-9
    assert any('limit' in h for h in hits)
