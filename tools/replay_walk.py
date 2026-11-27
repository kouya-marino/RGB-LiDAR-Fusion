"""Replay the 13-minute test-2 walk through the real colorizer node, in virtual time.

usage (inside the Docker image, ROS sourced):
    python3 tools/replay_walk.py <map_dir with full.pcd colored.pcd trajectory.csv> <label> [minutes]

Scans arrive at 10 Hz in virtual time. The cloud subscription is KEEP_LAST depth 5,
so while a callback runs, newer scans queue and the oldest beyond 5 are lost. Each
scan is ~20k points drawn from the saved full map within 12 m of the rig's position
(interpolated from trajectory.csv), plus ~5k colored points within 5 m, added the way
_colorize adds them. Image callbacks and the per-scan projection are not simulated:
this measures the map work that grows with the map, which is what slowed long walks.
Subscribers to /colored_map and /lidar_map are attached, as the web UI does.
"""
import sys
import time as _time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RosTime
from sensor_msgs.msg import PointCloud2

import rgb_lidar_fusion.colorizer_node as cn
from rgb_lidar_fusion.fusion import view_quality  # noqa: F401  (same import path as the node)
from rgb_lidar_fusion.ros_conv import xyzi_to_msg

MAP, LABEL = sys.argv[1], sys.argv[2]
MINUTES = float(sys.argv[3]) if len(sys.argv) > 3 else None
RATE, DEPTH, N_FULL, N_COL = 10.0, 5, 20000, 5000
rng = np.random.default_rng(1)


def load_pcd(path):
    with open(path, 'rb') as f:
        hdr = {}
        while True:
            k, *v = f.readline().decode().strip().split(' ')
            hdr[k] = v
            if k == 'DATA':
                break
        n = int(hdr['POINTS'][0])
        return np.frombuffer(f.read(n * 16), dtype=np.float32).reshape(n, 4).copy()


class Sampler:
    """Random points of a cloud within r metres (XY) of a position, via 1 m cells."""

    def __init__(self, cloud):
        self.cloud = cloud
        ij = np.floor(cloud[:, :2]).astype(np.int64)
        self.lo = ij.min(0)
        ij -= self.lo
        self.w = int(ij[:, 0].max()) + 1
        cell = ij[:, 1] * self.w + ij[:, 0]
        order = np.argsort(cell, kind='stable')
        self.cloud = cloud[order]
        cell = cell[order]
        self.cells, self.start, self.count = np.unique(cell, return_index=True, return_counts=True)

    def sample(self, xy, r, n):
        c = np.floor(xy).astype(np.int64) - self.lo
        cx, cy = self.cells % self.w, self.cells // self.w
        near = (cx - c[0]) ** 2 + (cy - c[1]) ** 2 <= r * r
        if not near.any():
            return self.cloud[:0]
        cnt = self.count[near]
        pick = rng.choice(np.flatnonzero(near), n, p=cnt / cnt.sum())
        idx = self.start[pick] + (rng.random(n) * self.count[pick]).astype(np.int64)
        return self.cloud[idx]


full, col = load_pcd(MAP + '/full.pcd'), load_pcd(MAP + '/colored.pcd')
traj = np.loadtxt(MAP + '/trajectory.csv', delimiter=',', skiprows=1)
t0 = traj[0, 0]
duration = traj[-1, 0] - t0 if MINUTES is None else MINUTES * 60
n_scans = int(duration * RATE)
ts = np.arange(n_scans) / RATE
pos = np.stack([np.interp(ts, traj[:, 0] - t0, traj[:, k]) for k in (1, 2)], 1)
S_full, S_col = Sampler(full), Sampler(col)
print('{}: {} scans over {:.0f} s; full map {:,} pts, colored {:,}'.format(
    LABEL, n_scans, duration, len(full), len(col)), flush=True)

# --- virtual clock: the node's time.monotonic() follows simulated time ---------
VNOW = [0.0]
_call_start = [None]


class _Clock:
    def __getattr__(self, name):
        return getattr(_time, name)

    @staticmethod
    def monotonic():
        extra = _time.perf_counter() - _call_start[0] if _call_start[0] is not None else 0.0
        return VNOW[0] + extra


cn.time = _Clock()

rclpy.init()
node = cn.ColorizerNode()
viewer = Node('replay_viewer')
viewer.create_subscription(PointCloud2, '/colored_map', lambda m: None, 1)
viewer.create_subscription(PointCloud2, '/lidar_map', lambda m: None, 1)
ex = rclpy.executors.SingleThreadedExecutor()
ex.add_node(node)
ex.add_node(viewer)
end = _time.time() + 3
while _time.time() < end and (node.map_pub.get_subscription_count() == 0 or
                              node.full_pub.get_subscription_count() == 0):
    ex.spin_once(timeout_sec=0.1)
print('subscribers: colored_map={} lidar_map={}'.format(
    node.map_pub.get_subscription_count(), node.full_pub.get_subscription_count()), flush=True)

# start "recording" directly: identity map frame, no camera (colors added below)
node.recording, node.map_name, node.rec_T = True, 'replay', np.eye(4)
node.rec_started, node.rec_height, node.rec_color_range = _time.time(), 1.7, 10.0
node._world = 'camera_init'
node.cam = None
if hasattr(node, '_reset_recording_stats'):
    node._reset_recording_stats()
has_display = hasattr(node, 'acc_display')


def callback(i):
    f = S_full.sample(pos[i], 12, N_FULL)
    f = f[:, :3] + rng.normal(0, 0.004, (len(f), 3)).astype(np.float32)
    msg = xyzi_to_msg(f, np.ones(len(f), np.float32), 'camera_init',
                      RosTime(seconds=t0 + ts[i]).to_msg())
    c = S_col.sample(pos[i], 5, N_COL)
    cxyz = c[:, :3] + rng.normal(0, 0.004, (len(c), 3)).astype(np.float32)
    u = c[:, 3].view(np.uint32)
    rgb = np.stack([(u >> 16) & 255, (u >> 8) & 255, u & 255], 1).astype(np.uint8)
    q = rng.random(len(c)).astype(np.float32)
    _call_start[0] = _time.perf_counter()
    t = _time.perf_counter()
    node._on_cloud(msg)                   # the real callback: full maps + publishing
    node.acc.add(cxyz, rgb, q)            # what _colorize adds for this scan
    if has_display:
        node.acc_display.add(cxyz, rgb, q)
    d = _time.perf_counter() - t
    _call_start[0] = None
    return d


queue, free_at, nxt = deque(), 0.0, 0
WALL0 = _time.time()
done, dropped, per_min = [], np.zeros(n_scans, bool), {}
while nxt < n_scans or queue:
    while nxt < n_scans and nxt / RATE <= free_at:
        queue.append(nxt)
        nxt += 1
        if len(queue) > DEPTH:
            dropped[queue.popleft()] = True
    if not queue:
        free_at = nxt / RATE
        continue
    i = queue.popleft()
    VNOW[0] = free_at
    d = callback(i)
    if len(done) % 200 == 0:
        print('  scan {} (virtual {:.0f} s) callback {:.0f} ms, wall {:.0f} s'.format(i, free_at, d * 1e3, _time.time() - WALL0), flush=True)
    free_at += d
    done.append((i, d))
    m = int(i / RATE // 60)
    s = per_min.setdefault(m, [0, 0.0])
    s[0] += 1
    s[1] = max(s[1], d)

print('minute  scans_processed  rate_Hz  slowest_callback_ms')
for m in sorted(per_min):
    n = per_min[m][0]
    span = min(60.0, duration - 60 * m)
    print('{:4d}    {:6d}          {:5.2f}    {:8.0f}'.format(m, n, n / span, per_min[m][1] * 1e3))
d = np.array([x[1] for x in done]) * 1e3
print('{}: processed {}/{} scans ({:.1f}%), dropped {}; callback ms p50 {:.0f} p95 {:.0f} p99 {:.0f} max {:.0f}'.format(
    LABEL, len(done), n_scans, 100 * len(done) / n_scans, int(dropped.sum()),
    *np.percentile(d, [50, 95, 99]), d.max()))
print('final maps: colored {:,}  full {:,}'.format(node.acc.count, node.full.count))
if hasattr(node, 'recording_stats'):
    st = node.recording_stats()
    print('recording_stats:', {k: st[k] for k in ('scans_received', 'scans_expected', 'scan_period_s')},
          'cloud callback_ms', st['callback_ms'].get('cloud'))
