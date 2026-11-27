"""The max colour distance (`max_range`): set per recording, applied to the points
that actually get coloured, refused while recording, and saved in info.json."""
import copy
import json
import time
from collections import deque

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('sensor_msgs.msg')

from builtin_interfaces.msg import Time as TimeMsg  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

from rgb_lidar_fusion.colorizer_node import ColorizerNode, color_range_error  # noqa: E402
from rgb_lidar_fusion.projection import CameraModel  # noqa: E402


@pytest.mark.parametrize('value,ok', [(15.0, True), (1, True), (40, True), (0.5, False),
                                      (41, False), (float('nan'), False), ('x', False),
                                      (None, False)])
def test_color_range_limits(value, ok):
    assert (color_range_error(value) is None) is ok


@pytest.fixture
def node(tmp_path):
    rclpy.init(args=['--ros-args', '-p', 'output_dir:={}'.format(tmp_path)])
    n = ColorizerNode()
    # a 64 x 48 pinhole camera that sees the lidar frame directly (identity TF)
    n.cam = CameraModel(K=[[50, 0, 32], [0, 50, 24], [0, 0, 1]], D=[], width=64, height=48)
    n.camera_frame = 'cam'
    n._lookup = lambda *a, **k: np.eye(4)
    n._estimate_map_frame = lambda height: (np.eye(4), True)
    img = Image(height=48, width=64, encoding='rgb8', step=64 * 3,
                data=bytes(np.full((48, 64, 3), 200, np.uint8).tobytes()))
    n._images = deque([(0.0, img)])
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _scan():
    """One point at each distance, in different directions (no occlusion)."""
    z = np.array([2.0, 10.0, 20.0, 30.0])
    return np.c_[np.array([-0.3, -0.1, 0.1, 0.3]) * z, np.zeros(4), z]


def _set(node, value):
    return node.set_parameters([Parameter('max_range', Parameter.Type.DOUBLE, value)])[0]


def test_default_is_the_configured_value(node):
    st = node.status()
    assert st['color_range'] == st['default_color_range'] == 10.0


def test_range_cuts_the_coloured_points_and_is_saved(node, tmp_path):
    assert _set(node, 15.0).successful
    ok, msg = node.start_map('near')
    assert ok, msg
    node._colorize(0.0, node.lidar_frame, TimeMsg(), _scan())
    xyz, _ = node.acc.export()
    assert sorted(np.round(xyz[:, 2]).tolist()) == [2.0, 10.0]      # 20 and 30 m dropped
    assert node.status()['rec_color_range'] == 15.0

    ok, _ = node.stop_map(save=True)
    assert ok
    info = json.loads((tmp_path / 'near' / 'info.json').read_text())
    assert info['color_range_m'] == 15.0 and info['colored_points'] == 2


def test_a_wider_range_keeps_far_points(node):
    assert _set(node, 25.0).successful
    assert node.start_map('far')[0]
    node._colorize(0.0, node.lidar_frame, TimeMsg(), _scan())
    assert node.acc.export()[0].shape[0] == 3                         # 30 m still dropped


def test_range_is_fixed_while_recording(node):
    assert node.start_map('fixed')[0]
    assert _set(node, 25.0).successful           # accepted, but only for the next recording
    node._colorize(0.0, node.lidar_frame, TimeMsg(), _scan())
    # the 10 m limit still applies (cut-off is 'closer than', so only the 2 m point)
    assert node.max_range == 10.0 and node.acc.export()[0].shape[0] == 1


def test_scan_waiting_at_start_is_not_recorded(node):
    """A scan received before Start and still waiting for its image is not
    part of the recording: no colors from it, and not counted as colored."""
    node._pending.append((5.0, time.monotonic(), node.lidar_frame, TimeMsg(), _scan()))
    assert node.start_map('fresh')[0]
    img = copy.deepcopy(node._images[0][1])
    img.header.stamp.sec = 5                     # the image the scan was waiting for
    node._handle_image(img)
    assert node.acc.export()[0].shape[0] == 0
    assert node.recording_stats()['scans_colored'] == 0


def test_last_end_tells_saved_from_cancelled(node):
    assert node.start_map('one')[0]
    node.stop_map(save=False)
    assert node.status()['last_end']['result'] == 'cancelled'
    assert node.start_map('two')[0]
    node.stop_map(save=True)
    end = node.status()['last_end']
    assert end['result'] == 'saved' and end['name'] == 'two' and end['by'] == 'user'


def test_out_of_range_value_refuses_start(node):
    assert _set(node, 0.5).successful            # accepted as a parameter ...
    ok, msg = node.start_map('bad')              # ... but Start checks it
    assert not ok and 'out of range' in msg and not node.recording


# --- auto-stop ---------------------------------------------------------------

from rgb_lidar_fusion.colorizer_node import auto_stop_error  # noqa: E402


@pytest.mark.parametrize('value,ok', [(0, True), (10, True), (3600, True), (9, False),
                                      (3601, False), (float('nan'), False), ('x', False)])
def test_auto_stop_limits(value, ok):
    assert (auto_stop_error(value) is None) is ok


def _set_auto(node, seconds):
    return node.set_parameters([Parameter('auto_stop_s', Parameter.Type.INTEGER, seconds)])[0]


def test_auto_stop_saves_on_time(node, tmp_path):
    assert _set_auto(node, 10).successful
    assert node.start_map('timed')[0]
    assert node.status()['auto_stop_s'] == 10
    node._check_auto_stop()
    assert node.recording                                # not yet
    node.rec_started -= 10.1                             # 10 s later
    node._check_auto_stop()
    assert not node.recording
    info = json.loads((tmp_path / 'timed' / 'info.json').read_text())
    assert info['auto_stop_s'] == 10 and info['stopped_by'] == 'auto-stop'
    end = node.status()['last_end']
    assert end['result'] == 'saved' and end['by'] == 'auto-stop' and end['name'] == 'timed'


def test_failed_auto_stop_save_is_reported(node, monkeypatch):
    assert _set_auto(node, 10).successful
    assert node.start_map('broken')[0]
    monkeypatch.setattr(node, 'save_map', lambda: (_ for _ in ()).throw(OSError('disk full')))
    node.rec_started -= 11
    node._check_auto_stop()
    end = node.status()['last_end']
    assert not node.recording and end['result'] == 'failed' and 'disk full' in end['error']


def test_no_auto_stop_by_default(node, tmp_path):
    assert node.start_map('manual')[0]
    node.rec_started -= 7200
    node._check_auto_stop()
    assert node.recording                                # only a hand stop ends it
    node.stop_map(save=True)
    info = json.loads((tmp_path / 'manual' / 'info.json').read_text())
    assert info['auto_stop_s'] is None and info['stopped_by'] == 'user'


def test_auto_stop_out_of_range_refuses_start(node):
    assert _set_auto(node, 5).successful
    ok, msg = node.start_map('short')
    assert not ok and 'auto-stop' in msg


def test_auto_stop_is_one_shot_through_the_service(node):
    from std_srvs.srv import Trigger
    assert _set_auto(node, 60).successful
    node.set_parameters([Parameter('map_name', Parameter.Type.STRING, 'svc')])
    res = node._srv_start(Trigger.Request(), Trigger.Response())
    assert res.success and node.rec_auto_stop == 60
    assert node.get_parameter('auto_stop_s').value == 0   # the next Start is not timed
