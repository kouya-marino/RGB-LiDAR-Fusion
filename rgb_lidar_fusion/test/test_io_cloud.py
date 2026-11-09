"""Unit tests for XYZRGB cloud output (binary PLY + binary PCD)."""
import numpy as np
import pytest

from rgb_lidar_fusion.io_cloud import pack_rgb, write_pcd_rgb, write_ply


def test_pack_rgb_bit_layout():
    packed = pack_rgb(np.array([[255, 128, 1]], np.uint8))
    as_int = packed.view(np.uint32)[0]
    assert (as_int >> 16) & 0xFF == 255
    assert (as_int >> 8) & 0xFF == 128
    assert as_int & 0xFF == 1


def test_pack_rgb_is_float32_for_pcl_compat():
    """PCL stores rgb as a float32 field holding a packed uint32."""
    packed = pack_rgb(np.zeros((4, 3), np.uint8))
    assert packed.dtype == np.float32 and packed.shape == (4,)


def test_pack_rgb_roundtrip_all_channels():
    rng = np.random.RandomState(0)
    rgb = rng.randint(0, 256, (500, 3)).astype(np.uint8)
    as_int = pack_rgb(rgb).view(np.uint32)
    back = np.stack([(as_int >> 16) & 0xFF, (as_int >> 8) & 0xFF, as_int & 0xFF], 1)
    np.testing.assert_array_equal(back.astype(np.uint8), rgb)


def test_pcd_header_declares_rgb(tmp_path):
    p = tmp_path / 'c.pcd'
    write_pcd_rgb(str(p), np.zeros((7, 3), np.float32), np.zeros((7, 3), np.uint8))
    head = p.read_bytes()[:400].decode('ascii', 'ignore')
    assert 'FIELDS x y z rgb' in head
    assert 'SIZE 4 4 4 4' in head
    assert 'TYPE F F F F' in head
    assert 'POINTS 7' in head and 'WIDTH 7' in head and 'HEIGHT 1' in head
    assert 'DATA binary' in head


def test_pcd_payload_is_exactly_16_bytes_per_point(tmp_path):
    p = tmp_path / 'c.pcd'
    n = 11
    write_pcd_rgb(str(p), np.zeros((n, 3), np.float32), np.zeros((n, 3), np.uint8))
    raw = p.read_bytes()
    payload = raw.split(b'DATA binary\n', 1)[1]
    assert len(payload) == n * 16


def test_pcd_readable_by_open3d_with_colour(tmp_path):
    o3d = pytest.importorskip('open3d')
    p = tmp_path / 'c.pcd'
    xyz = np.array([[1., 2., 3.], [-4., 5., -6.]], np.float32)
    rgb = np.array([[255, 0, 0], [0, 128, 255]], np.uint8)
    write_pcd_rgb(str(p), xyz, rgb)
    pc = o3d.io.read_point_cloud(str(p))
    assert len(pc.points) == 2 and pc.has_colors()
    np.testing.assert_allclose(np.asarray(pc.points), xyz, atol=1e-5)
    got = (np.asarray(pc.colors) * 255).round().astype(np.uint8)
    np.testing.assert_array_equal(got, rgb)


def test_ply_readable_by_open3d(tmp_path):
    o3d = pytest.importorskip('open3d')
    p = tmp_path / 'c.ply'
    xyz = np.random.rand(20, 3).astype(np.float32)
    write_ply(str(p), xyz, np.full((20, 3), 200, np.uint8))
    pc = o3d.io.read_point_cloud(str(p))
    assert len(pc.points) == 20 and pc.has_colors()


def test_ply_preserves_exact_colours(tmp_path):
    o3d = pytest.importorskip('open3d')
    p = tmp_path / 'c.ply'
    rgb = np.array([[0, 0, 0], [255, 255, 255], [17, 99, 200]], np.uint8)
    write_ply(str(p), np.zeros((3, 3), np.float32), rgb)
    got = (np.asarray(o3d.io.read_point_cloud(str(p)).colors) * 255).round()
    np.testing.assert_array_equal(got.astype(np.uint8), rgb)


def test_empty_cloud_writes_valid_files(tmp_path):
    """A run that colours nothing must still produce readable files, not crash."""
    o3d = pytest.importorskip('open3d')
    pcd, ply = tmp_path / 'e.pcd', tmp_path / 'e.ply'
    write_pcd_rgb(str(pcd), np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8))
    write_ply(str(ply), np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8))
    assert 'POINTS 0' in pcd.read_bytes()[:400].decode('ascii', 'ignore')
    assert len(o3d.io.read_point_cloud(str(ply)).points) == 0


def test_mismatched_lengths_raise(tmp_path):
    with pytest.raises(ValueError):
        write_pcd_rgb(str(tmp_path / 'x.pcd'),
                      np.zeros((3, 3), np.float32), np.zeros((2, 3), np.uint8))


# --- PLY checks that need no Open3D ------------------------------------

def _read_ply(path):
    raw = open(path, 'rb').read()
    head, _, body = raw.partition(b'end_header\n')
    n = int([ln for ln in head.decode().splitlines()
             if ln.startswith('element vertex')][0].split()[-1])
    rec = np.frombuffer(body, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                     ('r', 'u1'), ('g', 'u1'), ('b', 'u1')], count=n)
    return head.decode(), rec, len(body)


def test_ply_header_and_size_without_open3d(tmp_path):
    p = tmp_path / 'c.ply'
    write_ply(str(p), np.zeros((7, 3)), np.zeros((7, 3), dtype=np.uint8))
    head, _, nbytes = _read_ply(str(p))
    assert 'format binary_little_endian 1.0' in head
    assert 'element vertex 7' in head
    assert nbytes == 7 * 15


def test_ply_roundtrip_exact_without_open3d(tmp_path):
    rng = np.random.RandomState(1)
    xyz = rng.uniform(-5, 5, size=(50, 3)).astype(np.float32)
    rgb = rng.randint(0, 256, size=(50, 3)).astype(np.uint8)
    p = tmp_path / 'c.ply'
    write_ply(str(p), xyz, rgb)
    _, rec, _ = _read_ply(str(p))
    np.testing.assert_array_equal(np.stack([rec['x'], rec['y'], rec['z']], 1), xyz)
    np.testing.assert_array_equal(np.stack([rec['r'], rec['g'], rec['b']], 1), rgb)
