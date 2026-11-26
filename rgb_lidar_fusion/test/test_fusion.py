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


# --- incremental compaction == the original full re-sort --------------------
# The maps used to be re-sorted whole at every compaction, which made long
# recordings slow down as the map grew. The merge that replaced it must pick
# exactly the same winners, in the same order. These references are the
# original implementations.

def _reference_accumulator(batches, voxel):
    from rgb_lidar_fusion.fusion import _OFFSET, Accumulator as A
    keys = np.zeros(0, np.int64)
    xyz = np.zeros((0, 3), np.float32)
    rgb = np.zeros((0, 3), np.uint8)
    q = np.zeros(0, np.float32)
    for pts, col, qual, compact in batches:
        idx = np.floor(pts / voxel).astype(np.int64)
        ok = np.all((idx >= -_OFFSET) & (idx < _OFFSET), axis=1)
        keys = np.concatenate([keys, A._pack(idx[ok])])
        xyz = np.concatenate([xyz, pts[ok].astype(np.float32)])
        rgb = np.concatenate([rgb, col[ok]])
        q = np.concatenate([q, qual[ok]])
        if compact:
            order = np.lexsort((-q, keys))
            _, first = np.unique(keys[order], return_index=True)
            w = order[first]
            keys, xyz, rgb, q = keys[w], xyz[w], rgb[w], q[w]
    order = np.lexsort((-q, keys))
    _, first = np.unique(keys[order], return_index=True)
    w = order[first]
    return xyz[w], rgb[w]


def _reference_voxelgrid(batches, voxel):
    from rgb_lidar_fusion.fusion import Accumulator as A
    keys = np.zeros(0, np.int64)
    xyz = np.zeros((0, 3), np.float32)
    inten = np.zeros(0, np.float32)
    for pts, i, compact in batches:
        keys = np.concatenate([keys, A._pack(np.floor(pts / voxel).astype(np.int64))])
        xyz = np.concatenate([xyz, pts.astype(np.float32)])
        inten = np.concatenate([inten, i])
        if compact:
            _, first = np.unique(keys, return_index=True)
            keys, xyz, inten = keys[first], xyz[first], inten[first]
    _, first = np.unique(keys, return_index=True)
    return xyz[first], inten[first]


def _batches(rng, n_batches, n, coarse_quality):
    """Overlapping batches; coarse qualities make exact ties common (view_quality
    returns 0 beyond max_range, so ties happen in real recordings too)."""
    out = []
    for b in range(n_batches):
        pts = rng.uniform(-1.0, 1.0, (n, 3))
        col = rng.integers(0, 256, (n, 3), dtype=np.uint8)
        q = rng.integers(0, 3, n).astype(np.float32) / 2 if coarse_quality \
            else rng.random(n).astype(np.float32)
        out.append((pts, col, q, b % 3 == 2))          # compact every third batch
    return out


@pytest.mark.parametrize('coarse_quality', [False, True])
def test_incremental_accumulator_matches_full_resort(coarse_quality):
    rng = np.random.default_rng(7)
    batches = _batches(rng, 12, 4000, coarse_quality)
    acc = Accumulator(voxel=0.05)
    for pts, col, q, compact in batches:
        acc.add(pts, col, q)
        if compact:
            acc.count                                     # forces a compaction
    xyz, rgb = acc.export()
    ref_xyz, ref_rgb = _reference_accumulator(batches, 0.05)
    np.testing.assert_array_equal(xyz, ref_xyz)
    np.testing.assert_array_equal(rgb, ref_rgb)


def test_incremental_voxelgrid_matches_full_resort():
    from rgb_lidar_fusion.fusion import VoxelGrid
    rng = np.random.default_rng(11)
    batches = [(rng.uniform(-1.0, 1.0, (4000, 3)), rng.random(4000).astype(np.float32),
                b % 3 == 2) for b in range(12)]
    g = VoxelGrid(voxel=0.05)
    for pts, i, compact in batches:
        g.add(pts, i)
        if compact:
            g.count
    xyz, inten = g.export()
    ref_xyz, ref_i = _reference_voxelgrid(batches, 0.05)
    np.testing.assert_array_equal(xyz, ref_xyz)
    np.testing.assert_array_equal(inten, ref_i)


def test_equal_quality_keeps_the_stored_view():
    acc = Accumulator(voxel=0.1)
    acc.add(np.array([[0.01, 0.01, 0.01]]), np.array([[10, 10, 10]]), np.array([0.5]))
    acc.count
    acc.add(np.array([[0.02, 0.02, 0.02]]), np.array([[99, 99, 99]]), np.array([0.5]))
    xyz, rgb = acc.export()
    assert rgb.tolist() == [[10, 10, 10]]
    np.testing.assert_allclose(xyz, [[0.01, 0.01, 0.01]], atol=1e-6)
