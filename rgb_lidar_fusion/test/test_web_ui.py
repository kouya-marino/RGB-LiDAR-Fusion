"""Encoders between ROS clouds / saved files and the web viewer's binary format."""
import numpy as np
import pytest

pytest.importorskip('sensor_msgs.msg')
pytest.importorskip('ament_index_python')

from rgb_lidar_fusion.colorizer_node import valid_map_name  # noqa: E402
from rgb_lidar_fusion.io_cloud import write_pcd_rgb, write_pcd_xyzi  # noqa: E402
from rgb_lidar_fusion.ros_conv import xyzi_to_msg, xyzrgb_to_msg  # noqa: E402
from rgb_lidar_fusion.web_ui_node import (  # noqa: E402
    _Latest, encode_colored_msg, encode_lidar_msg, encode_saved)


def _decode_rgb(b):
    n = int(np.frombuffer(b[:4], np.uint32)[0])
    xyz = np.frombuffer(b, '<f4', 3 * n, 4).reshape(n, 3)
    rgb = np.frombuffer(b, np.uint8, 3 * n, 4 + 12 * n).reshape(n, 3)
    return xyz, rgb


def _decode_i(b):
    n = int(np.frombuffer(b[:4], np.uint32)[0])
    return (np.frombuffer(b, '<f4', 3 * n, 4).reshape(n, 3),
            np.frombuffer(b, '<f4', n, 4 + 12 * n))


def _bytes_msg(msg):
    msg.data = bytes(msg.data)
    return msg


def test_colored_msg_roundtrip():
    xyz = np.array([[1, 2, 3], [4, 5, 6]], np.float32)
    rgb = np.array([[255, 0, 10], [1, 2, 3]], np.uint8)
    msg = _bytes_msg(xyzrgb_to_msg(xyz, rgb, 'f', None))
    got_xyz, got_rgb = _decode_rgb(encode_colored_msg(msg))
    np.testing.assert_array_equal(got_xyz, xyz)
    np.testing.assert_array_equal(got_rgb, rgb)


def test_lidar_msg_keeps_all_points_by_default_and_filters_on_request():
    xyz = np.array([[0, 0, 0], [3, 0, 0]], np.float32)       # Mid-360 no-return = origin
    msg = _bytes_msg(xyzi_to_msg(xyz, [1., 9.], 'f', None))
    assert _decode_i(encode_lidar_msg(msg))[0].shape == (2, 3)          # map frame: keep
    got_xyz, got_i = _decode_i(encode_lidar_msg(msg, min_range=0.1))    # raw scan: drop
    np.testing.assert_array_equal(got_xyz, [[3, 0, 0]])
    np.testing.assert_array_equal(got_i, [9.])


def test_encode_saved_both_kinds(tmp_path):
    xyz = np.array([[1, 1, 1]], np.float32)
    write_pcd_rgb(str(tmp_path / 'colored.pcd'), xyz, np.array([[7, 8, 9]], np.uint8))
    write_pcd_xyzi(str(tmp_path / 'full.pcd'), xyz, np.array([42.], np.float32))
    assert _decode_rgb(encode_saved(str(tmp_path / 'colored.pcd')))[1].tolist() == [[7, 8, 9]]
    assert _decode_i(encode_saved(str(tmp_path / 'full.pcd')))[1].tolist() == [42.]


def test_latest_encodes_lazily_once():
    calls = []
    latest = _Latest(lambda m: calls.append(m) or b'x')
    assert latest.get() == (0, np.uint32(0).tobytes())       # nothing yet
    latest.put('a')
    latest.put('b')                                           # only the newest is encoded
    assert latest.get() == (2, b'x') and latest.get() == (2, b'x')
    assert calls == ['b']


@pytest.mark.parametrize('name,ok', [('kitchen_01', True), ('a-b', True), ('', False),
                                     ('../etc', False), ('x' * 65, False), ('a b', False)])
def test_map_names(name, ok):
    assert valid_map_name(name) is ok
