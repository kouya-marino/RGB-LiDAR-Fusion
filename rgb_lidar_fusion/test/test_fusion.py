"""Unit tests for the best-view voxel colour accumulator."""
import numpy as np
import pytest

from rgb_lidar_fusion.fusion import Accumulator, view_quality
from rgb_lidar_fusion.projection import CameraModel

CAM = CameraModel(K=np.array([[600., 0., 640.], [0., 600., 360.], [0., 0., 1.]]),
                  D=np.zeros(5), width=1280, height=720)


def _add(acc, pts, rgb, q):
    acc.add(np.asarray(pts, dtype=float),
            np.asarray(rgb, dtype=np.uint8).reshape(-1, 3),
            np.asarray(q, dtype=float))


# --- voxelisation -------------------------------------------------------

def test_voxel_dedup_collapses_near_duplicates():
    a = Accumulator(voxel=0.10)
    _add(a, [[0.00, 0., 0.], [0.02, 0., 0.]], [[255, 0, 0], [255, 0, 0]], [1.0, 1.0])
    assert a.count == 1


def test_distinct_voxels_kept():
    a = Accumulator(voxel=0.10)
    _add(a, [[0., 0., 0.], [1., 0., 0.]], [[0, 0, 0], [0, 0, 0]], [1.0, 1.0])
    assert a.count == 2


def test_negative_coordinates_do_not_collide_with_positive():
    """A sign-handling bug here silently merges opposite corners of the room."""
    a = Accumulator(voxel=0.10)
    pts = [[-1., -2., -3.], [1., 2., 3.], [-1., 2., -3.], [1., -2., 3.]]
    _add(a, pts, [[0, 0, 0]] * 4, [1.0] * 4)
    assert a.count == 4


def test_voxel_keys_are_collision_free_over_a_large_random_cloud():
    a = Accumulator(voxel=0.05)
    rng = np.random.RandomState(0)
    pts = rng.uniform(-30, 30, size=(20000, 3))
    _add(a, pts, rng.randint(0, 256, (20000, 3)), rng.rand(20000))
    # ground truth: unique voxel triples computed independently
    expected = len(set(map(tuple, np.floor(pts / 0.05).astype(np.int64))))
    assert a.count == expected


# --- best-view fusion ---------------------------------------------------

def test_higher_quality_wins_regardless_of_order():
    for first, second in ((0.2, 0.9), (0.9, 0.2)):
        a = Accumulator(voxel=0.10)
        _add(a, [[0., 0., 0.]], [[10, 10, 10]], [first])
        _add(a, [[0., 0., 0.]], [[200, 200, 200]], [second])
        _, rgb = a.export()
        expected = 200 if second > first else 10
        assert rgb[0][0] == expected, (first, second)


def test_best_wins_within_a_single_batch():
    a = Accumulator(voxel=0.10)
    _add(a, [[0., 0., 0.]] * 3, [[1, 1, 1], [9, 9, 9], [5, 5, 5]], [0.1, 0.9, 0.5])
    _, rgb = a.export()
    assert a.count == 1 and rgb[0][0] == 9


def test_export_shapes_and_dtypes():
    a = Accumulator(voxel=0.05)
    rng = np.random.RandomState(1)
    _add(a, rng.rand(50, 3), rng.randint(0, 256, (50, 3)), rng.rand(50))
    xyz, rgb = a.export()
    assert xyz.dtype == np.float32 and rgb.dtype == np.uint8
    assert xyz.shape[1] == 3 and rgb.shape[1] == 3
    assert xyz.shape[0] == rgb.shape[0] == a.count


def test_export_keeps_the_winning_observations_position():
    """We store the winner's actual xyz, not the voxel centre."""
    a = Accumulator(voxel=1.0)
    _add(a, [[0.10, 0.20, 0.30]], [[1, 1, 1]], [0.1])
    _add(a, [[0.70, 0.80, 0.90]], [[2, 2, 2]], [0.9])
    xyz, rgb = a.export()
    assert rgb[0][0] == 2
    np.testing.assert_allclose(xyz[0], [0.7, 0.8, 0.9], atol=1e-6)


def test_empty_accumulator_exports_empty_not_crash():
    a = Accumulator(voxel=0.05)
    xyz, rgb = a.export()
    assert a.count == 0 and xyz.shape == (0, 3) and rgb.shape == (0, 3)


def test_adding_empty_batch_is_a_noop():
    a = Accumulator(voxel=0.05)
    _add(a, [[1., 1., 1.]], [[7, 7, 7]], [0.5])
    a.add(np.zeros((0, 3)), np.zeros((0, 3), np.uint8), np.zeros(0))
    assert a.count == 1


def test_export_is_repeatable_and_nondestructive():
    a = Accumulator(voxel=0.1)
    _add(a, [[0., 0., 0.], [1., 0., 0.]], [[1, 2, 3], [4, 5, 6]], [0.5, 0.5])
    first = a.export()
    second = a.export()
    np.testing.assert_array_equal(first[0], second[0])
    np.testing.assert_array_equal(first[1], second[1])
    assert a.count == 2


def test_many_incremental_adds_stay_correct():
    """Mimics the real per-scan loop: many small adds, overlapping voxels."""
    a = Accumulator(voxel=0.1)
    for i in range(200):
        q = i / 200.0
        _add(a, [[0., 0., 0.]], [[i % 256, 0, 0]], [q])
    _, rgb = a.export()
    assert a.count == 1 and rgb[0][0] == 199 % 256


# --- view quality -------------------------------------------------------

def test_quality_prefers_near_and_centred():
    pts = np.array([[0., 0., 2.0], [0., 0., 8.0]])
    uv = np.array([[640., 360.], [640., 360.]])
    q = view_quality(pts, uv, CAM)
    assert q[0] > q[1]                                   # nearer is better
    q2 = view_quality(np.array([[0., 0., 2.0]]), np.array([[20., 20.]]), CAM)
    assert q[0] > q2[0]                                  # centred is better


def test_quality_is_bounded_unit_interval():
    rng = np.random.RandomState(2)
    pts = np.column_stack([rng.uniform(-5, 5, 500), rng.uniform(-5, 5, 500),
                           rng.uniform(0.1, 40, 500)])
    uv = np.column_stack([rng.uniform(0, 1279, 500), rng.uniform(0, 719, 500)])
    q = view_quality(pts, uv, CAM)
    assert q.shape == (500,)
    assert np.all(q >= 0.0) and np.all(q <= 1.0)
    assert np.isfinite(q).all()


def test_quality_empty_input():
    q = view_quality(np.zeros((0, 3)), np.zeros((0, 2)), CAM)
    assert q.shape == (0,)


# --- VoxelGrid (full lidar map) -------------------------------------------

def test_voxelgrid_one_point_per_voxel_and_earliest_wins():
    from rgb_lidar_fusion.fusion import VoxelGrid
    g = VoxelGrid(voxel=0.1)
    g.add([[0.01, 0.01, 0.01], [0.02, 0.02, 0.02], [1.0, 1.0, 1.0]], [5.0, 9.0, 7.0])
    g.add([[0.03, 0.03, 0.03]], [99.0])                   # same voxel as the first
    assert g.count == 2
    xyz, inten = g.export()
    order = np.argsort(xyz[:, 0])
    # earliest observation kept: 5.0 (not 9.0 or 99.0) for the voxel at the origin
    np.testing.assert_array_equal(inten[order], [5.0, 7.0])


def test_voxelgrid_mismatched_lengths_raise():
    from rgb_lidar_fusion.fusion import VoxelGrid
    with pytest.raises(ValueError):
        VoxelGrid().add(np.zeros((3, 3)), np.zeros(2))


def test_voxelgrid_empty_export():
    from rgb_lidar_fusion.fusion import VoxelGrid
    xyz, inten = VoxelGrid().export()
    assert xyz.shape == (0, 3) and inten.shape == (0,)
