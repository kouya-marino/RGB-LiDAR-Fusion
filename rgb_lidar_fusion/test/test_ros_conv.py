"""Unit tests for the colorizer's pure helpers (no node, no DDS).

These carry real weight -- a wrong quaternion convention or a mis-parsed
PointCloud2 would put colour on the wrong surfaces with no obvious symptom.
"""
import numpy as np
import pytest

from rgb_lidar_fusion.ros_conv import (
    angle_between,
    image_to_rgb,
    pointcloud2_to_xyz,
    quat_to_matrix,
    slerp,
    stamp_to_sec,
    transform_to_matrix,
    xyzrgb_to_msg,
)

sensor_msgs = pytest.importorskip('sensor_msgs.msg')
Image = sensor_msgs.Image
PointCloud2 = sensor_msgs.PointCloud2
PointField = sensor_msgs.PointField


# --- quaternions --------------------------------------------------------

def test_identity_quaternion_is_identity_matrix():
    np.testing.assert_allclose(quat_to_matrix(np.array([0., 0., 0., 1.])),
                               np.eye(3), atol=1e-12)


def test_quat_90_about_z():
    q = np.array([0., 0., np.sin(np.pi / 4), np.cos(np.pi / 4)])   # x,y,z,w
    R = quat_to_matrix(q)
    np.testing.assert_allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-12)
    np.testing.assert_allclose(R @ [0, 1, 0], [-1, 0, 0], atol=1e-12)


def test_quat_to_matrix_is_orthonormal_for_random_quats():
    rng = np.random.RandomState(0)
    for _ in range(50):
        q = rng.normal(size=4)
        R = quat_to_matrix(q)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10)
        assert np.linalg.det(R) > 0


def test_slerp_endpoints():
    q0 = np.array([0., 0., 0., 1.])
    q1 = np.array([0., 0., np.sin(np.pi / 4), np.cos(np.pi / 4)])
    np.testing.assert_allclose(np.abs(slerp(q0, q1, 0.0)), np.abs(q0), atol=1e-12)
    np.testing.assert_allclose(np.abs(slerp(q0, q1, 1.0)), np.abs(q1), atol=1e-12)


def test_slerp_midpoint_is_half_the_rotation():
    q0 = np.array([0., 0., 0., 1.])
    q1 = np.array([0., 0., np.sin(np.pi / 4), np.cos(np.pi / 4)])   # 90 deg
    mid = slerp(q0, q1, 0.5)
    assert np.degrees(angle_between(q0, mid)) == pytest.approx(45.0, abs=1e-6)


def test_slerp_stays_unit_norm():
    rng = np.random.RandomState(1)
    for _ in range(30):
        q0 = rng.normal(size=4)
        q1 = rng.normal(size=4)
        for u in (0.0, 0.25, 0.5, 0.9, 1.0):
            assert np.linalg.norm(slerp(q0, q1, u)) == pytest.approx(1.0, abs=1e-12)


def test_slerp_takes_the_short_way_round():
    """q and -q are the same rotation; interpolating must not go the long way."""
    q0 = np.array([0., 0., 0., 1.])
    q1 = -np.array([0., 0., np.sin(np.pi / 8), np.cos(np.pi / 8)])   # 45 deg, negated
    mid = slerp(q0, q1, 0.5)
    assert np.degrees(angle_between(q0, mid)) == pytest.approx(22.5, abs=1e-6)


def test_slerp_near_identical_quats_does_not_blow_up():
    q0 = np.array([0., 0., 0., 1.])
    q1 = q0 + np.array([1e-9, 0., 0., 0.])
    out = slerp(q0, q1, 0.5)
    assert np.isfinite(out).all()
    assert np.linalg.norm(out) == pytest.approx(1.0, abs=1e-12)


def test_angle_between_is_sign_invariant():
    q = np.array([0.1, 0.2, 0.3, 0.9])
    assert angle_between(q, -q) == pytest.approx(0.0, abs=1e-7)


# --- PointCloud2 parsing ------------------------------------------------

def _cloud(points, extra_fields=True):
    """Build an all-float32 PointCloud2 with extra fields (PointXYZINormal-like)."""
    names = ['x', 'y', 'z']
    if extra_fields:
        names += ['intensity', 'normal_x', 'normal_y', 'normal_z', 'curvature']
    msg = PointCloud2()
    msg.height = 1
    msg.width = len(points)
    msg.point_step = 4 * len(names)
    msg.row_step = msg.point_step * len(points)
    msg.fields = [PointField(name=n, offset=4 * i,
                             datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(names)]
    rec = np.zeros((len(points), len(names)), dtype=np.float32)
    rec[:, :3] = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    rec[:, 3:] = 7.0                      # junk in the extra fields
    msg.data = rec.tobytes()
    return msg


def test_pointcloud2_extracts_xyz_ignoring_extra_fields():
    pts = [[1., 2., 3.], [-4., 5., -6.]]
    got = pointcloud2_to_xyz(_cloud(pts))
    assert got.shape == (2, 3)
    np.testing.assert_allclose(got, pts, atol=1e-6)


def test_pointcloud2_plain_xyz_also_works():
    pts = [[9., 8., 7.]]
    np.testing.assert_allclose(pointcloud2_to_xyz(_cloud(pts, extra_fields=False)),
                               pts, atol=1e-6)


def test_pointcloud2_empty():
    assert pointcloud2_to_xyz(_cloud([])).shape == (0, 3)


def _livox_cloud(points):
    """Build a PointCloud2 with livox_ros_driver2's packed 26-byte layout:
    x y z intensity (float32), tag line (uint8), timestamp (float64)."""
    dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
                   ('tag', 'u1'), ('line', 'u1'), ('timestamp', '<f8')])
    assert dt.itemsize == 26
    rec = np.zeros(len(points), dtype=dt)
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    rec['x'], rec['y'], rec['z'] = pts[:, 0], pts[:, 1], pts[:, 2]
    rec['intensity'], rec['tag'], rec['line'] = 42.0, 0x10, 3
    rec['timestamp'] = 1.7e18
    F = PointField
    msg = PointCloud2()
    msg.height, msg.width, msg.point_step = 1, len(points), 26
    msg.row_step = 26 * len(points)
    msg.fields = [F(name='x', offset=0, datatype=F.FLOAT32, count=1),
                  F(name='y', offset=4, datatype=F.FLOAT32, count=1),
                  F(name='z', offset=8, datatype=F.FLOAT32, count=1),
                  F(name='intensity', offset=12, datatype=F.FLOAT32, count=1),
                  F(name='tag', offset=16, datatype=F.UINT8, count=1),
                  F(name='line', offset=17, datatype=F.UINT8, count=1),
                  F(name='timestamp', offset=18, datatype=F.FLOAT64, count=1)]
    msg.data = rec.tobytes()
    return msg


def test_pointcloud2_livox_packed_layout():
    pts = [[1.5, -2.25, 0.125], [10., 0., -3.]]
    np.testing.assert_allclose(pointcloud2_to_xyz(_livox_cloud(pts)), pts, atol=1e-6)


def test_pointcloud2_without_xyz_raises():
    msg = PointCloud2()
    msg.height, msg.width, msg.point_step = 1, 1, 4
    msg.fields = [PointField(name='intensity', offset=0,
                             datatype=PointField.FLOAT32, count=1)]
    msg.data = np.zeros(1, dtype=np.float32).tobytes()
    with pytest.raises(ValueError):
        pointcloud2_to_xyz(msg)


# --- image decoding -----------------------------------------------------

def test_image_rgb8_roundtrip():
    img = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    msg = Image(height=2, width=3, encoding='rgb8', step=9)
    msg.data = img.tobytes()
    np.testing.assert_array_equal(image_to_rgb(msg), img)


def test_image_bgr8_is_channel_swapped():
    bgr = np.array([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)
    msg = Image(height=1, width=2, encoding='bgr8', step=6)
    msg.data = bgr.tobytes()
    np.testing.assert_array_equal(image_to_rgb(msg), bgr[:, :, ::-1])


def test_image_handles_row_padding():
    """step > width*3 means padded rows; the padding must be cropped off."""
    width, height = 2, 2
    step = 9                                  # 3 px of storage for 2 px of image
    raw = np.zeros((height, step // 3, 3), dtype=np.uint8)
    raw[:, :width, :] = [[1, 1, 1], [2, 2, 2]]
    raw[:, width:, :] = 99                    # padding must not appear
    msg = Image(height=height, width=width, encoding='rgb8', step=step)
    msg.data = raw.tobytes()
    out = image_to_rgb(msg)
    assert out.shape == (height, width, 3)
    assert 99 not in out


def test_image_unsupported_encoding_raises():
    msg = Image(height=1, width=1, encoding='yuv422', step=2)
    msg.data = bytes(2)
    with pytest.raises(ValueError):
        image_to_rgb(msg)


def test_stamp_to_sec():
    class S:
        sec, nanosec = 5, 250_000_000
    assert stamp_to_sec(S()) == pytest.approx(5.25)


# --- TF transform and XYZRGB output -------------------------------------

class _V:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def test_transform_to_matrix_rotation_and_translation():
    q = [0., 0., np.sin(np.pi / 4), np.cos(np.pi / 4)]          # 90 deg about z
    tf = _V(translation=_V(x=1., y=2., z=3.),
            rotation=_V(x=q[0], y=q[1], z=q[2], w=q[3]))
    T = transform_to_matrix(tf)
    np.testing.assert_allclose(T @ [1, 0, 0, 1], [1, 3, 3, 1], atol=1e-12)


def test_xyzrgb_to_msg_layout_and_colour_bits():
    xyz = np.array([[1., 2., 3.]], dtype=np.float32)
    rgb = np.array([[255, 128, 1]], dtype=np.uint8)
    msg = xyzrgb_to_msg(xyz, rgb, 'livox_frame', None)
    assert msg.header.frame_id == 'livox_frame'
    assert [f.name for f in msg.fields] == ['x', 'y', 'z', 'rgb']
    assert msg.point_step == 16 and msg.width == 1
    rec = np.frombuffer(bytes(msg.data), dtype=np.float32)
    np.testing.assert_allclose(rec[:3], [1, 2, 3])
    assert rec[3:4].view(np.uint32)[0] == 0xFF8001


def test_pointcloud2_field_reads_livox_intensity_and_defaults():
    from rgb_lidar_fusion.ros_conv import pointcloud2_field
    msg = _livox_cloud([[1., 2., 3.], [4., 5., 6.]])
    np.testing.assert_array_equal(pointcloud2_field(msg, 'intensity'), [42., 42.])
    np.testing.assert_array_equal(pointcloud2_field(msg, 'missing', -1.), [-1., -1.])


def test_xyzi_to_msg_roundtrip():
    from rgb_lidar_fusion.ros_conv import pointcloud2_field, xyzi_to_msg
    msg = xyzi_to_msg(np.array([[1., 2., 3.]]), np.array([77.]), 'livox_frame', None)
    assert [f.name for f in msg.fields] == ['x', 'y', 'z', 'intensity']
    msg.data = bytes(msg.data)
    np.testing.assert_allclose(pointcloud2_to_xyz(msg), [[1, 2, 3]])
    np.testing.assert_array_equal(pointcloud2_field(msg, 'intensity'), [77.])
