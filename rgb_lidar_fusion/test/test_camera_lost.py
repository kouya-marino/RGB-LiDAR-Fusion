"""Camera loss: the status reports how long ago the last image arrived (the web UI
warns past `camera_stale_s`), and a recording lists the stretches without
images in info.json (e.g. the USB link dropped mid-walk)."""
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('sensor_msgs.msg')

from sensor_msgs.msg import Image  # noqa: E402

import rgb_lidar_fusion.colorizer_node as cn  # noqa: E402


class Clock:
    def __init__(self, t):
        self.t = t

    def __call__(self):
        return self.t


@pytest.fixture
def node(tmp_path, monkeypatch):
    clock = Clock(100.0)
    monkeypatch.setattr(cn.time, 'monotonic', clock)
    rclpy.init(args=['--ros-args', '-p', 'output_dir:={}'.format(tmp_path)])
    n = cn.ColorizerNode()
    n.clock_ = clock
    yield n
    n.destroy_node()
    rclpy.shutdown()


def image_at(n, t):
    n.clock_.t = t
    n._handle_image(Image())


def start_recording(n, t):
    n.clock_.t = t
    n.recording, n.rec_started = True, 0.0
    n._reset_recording_stats()


def test_status_age_of_last_image(node):
    assert node.status()['camera_age_s'] is None          # none yet: the UI says lost
    image_at(node, 100.0)
    node.clock_.t = 103.0
    st = node.status()
    assert st['camera_age_s'] == 3.0 and st['camera_stale_s'] == 2.0


def test_short_gaps_are_not_losses(node):
    start_recording(node, 100.0)
    for t in (100.0, 101.5, 103.0, 104.9):                 # gaps up to 1.9 s
        image_at(node, t)
    node.clock_.t = 105.0
    assert node.recording_stats()['camera_lost'] == dict(times=0, total_s=0.0, gaps_s=[])
    assert node.status()['rec_camera_lost'] == [0, 0.0]


def test_losses_closed_and_still_open(node):
    image_at(node, 99.0)
    start_recording(node, 100.0)
    image_at(node, 101.0)
    image_at(node, 110.0)                                  # lost 101 -> 110
    node.clock_.t = 120.0                                  # lost again since 110
    lost = node.recording_stats()['camera_lost']
    assert lost == dict(times=2, total_s=19.0, gaps_s=[[1.0, 9.0], [10.0, 10.0]])
    assert node.status()['rec_camera_lost'] == [2, 19.0]


def test_camera_dead_from_the_start(node):
    image_at(node, 50.0)                                   # long before the recording
    start_recording(node, 100.0)
    node.clock_.t = 105.0
    assert node.recording_stats()['camera_lost']['gaps_s'] == [[0.0, 5.0]]
    image_at(node, 106.0)                                  # back: the loss is closed
    node.clock_.t = 107.0
    assert node.recording_stats()['camera_lost']['gaps_s'] == [[0.0, 6.0]]
