"""Numbers for the paper from one saved RGB-LiDAR-Fusion map.

usage: python3 tools/walk_report.py output/<map name>

Reads info.json and trajectory.csv and prints: scan rate per minute, scans
received vs expected, how many were colored (and why not), callback times, and
the start-to-end position difference (a drift measure if you returned the rig to
a marked start point with the same heading).

It also saves <map>/top_view.png: the full map from above (floor grey, walls and
furniture dark), the walked path, start and end, and the numbers in the title.
"""
import json
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

folder = sys.argv[1].rstrip('/')
info = json.load(open(folder + '/info.json'))
traj = np.loadtxt(folder + '/trajectory.csv', delimiter=',', skiprows=1, ndmin=2)
t = traj[:, 0] - traj[0, 0]

print('map {!r}: {:.1f} s, {:.1f} m walked, lidar at {:.3f} m'.format(
    info['name'], info['duration_s'], info['path_length_m'], info['lidar_height_m']))
print('points: colored {:,}  full {:,}  (voxel {} m)'.format(
    info['colored_points'], info['full_points'], info['voxel_m']))

print('\nminute  scans  rate_Hz  max_gap_s')
d = np.diff(t)
for m in range(int(t[-1] // 60) + 1):
    sel = (t >= 60 * m) & (t < 60 * (m + 1))
    span = min(60.0, t[-1] - 60 * m) or 1.0
    gaps = d[sel[1:]] if sel[1:].any() else np.zeros(1)
    print('{:4d}   {:5d}   {:5.2f}    {:5.2f}'.format(m, sel.sum(), sel.sum() / span, gaps.max()))

st = info.get('stats')
if st:
    rec, exp = st['scans_received'], st['scans_expected']
    print('\nscans received {:,} of {:,} expected ({:.1f}%), period {} s'.format(
        rec, exp, 100 * rec / max(exp, 1), st['scan_period_s']))
    print('scans colored {:,} ({:.1f}% of received); images received {:,}'.format(
        st['scans_colored'], 100 * st['scans_colored'] / max(rec, 1), st['images_received']))
    print('not colored:', ', '.join('{} {}'.format(k, v) for k, v in st['not_colored'].items()))
    cl = st.get('camera_lost')
    if cl is not None:
        print('camera lost {} times, {:.1f} s in total{}'.format(
            cl['times'], cl['total_s'], ''.join(
                '\n  at {:.0f} s for {:.1f} s'.format(*g) for g in cl['gaps_s'])))
    for k, v in st.get('callback_ms', {}).items():
        print('{} callback ms: p50 {} p95 {} p99 {} max {} (n={:,})'.format(
            k, v['p50'], v['p95'], v['p99'], v['max'], v['n']))
else:
    print('\n(no "stats" in info.json: recorded before the stats were added)')

p0, p1 = traj[0, 1:], traj[-1, 1:]
print('\nstart {}  end {}'.format(np.round(p0, 3), np.round(p1, 3)))
gap_h, gap_v = float(np.linalg.norm(p1[:2] - p0[:2])), float(p1[2] - p0[2])
print('start-to-end: horizontal {:.3f} m, vertical {:+.3f} m, over {:.1f} m walked'.format(
    gap_h, gap_v, info['path_length_m']))


def load_pcd(path):
    with open(path, 'rb') as f:
        hdr = {}
        while True:
            k, *v = f.readline().decode().strip().split(' ')
            hdr[k] = v
            if k == 'DATA':
                break
        n = int(hdr['POINTS'][0])
        return np.frombuffer(f.read(n * 16), dtype=np.float32).reshape(n, 4)[:, :3].copy()


def top_view(out_png):
    """Full map from above: floor-level points grey, obstacles (0.15-2.0 m) dark."""
    xyz = load_pcd(folder + '/full.pcd')
    xyz = xyz[xyz[:, 2] < 2.2]                          # drop the ceiling
    lo = np.percentile(xyz[:, :2], 0.2, axis=0) - 2.0   # ignore stray far points
    hi = np.percentile(xyz[:, :2], 99.8, axis=0) + 2.0
    lo, hi = np.minimum(lo, traj[:, 1:3].min(0) - 2), np.maximum(hi, traj[:, 1:3].max(0) + 2)
    res = max(float((hi - lo).max()) / 1400, 0.01)      # m per pixel
    w, h = (np.ceil((hi - lo) / res)).astype(int) + 1
    ij = ((xyz[:, :2] - lo) / res).astype(int)
    ok = (ij[:, 0] >= 0) & (ij[:, 0] < w) & (ij[:, 1] >= 0) & (ij[:, 1] < h)
    ij, z = ij[ok], xyz[ok, 2]
    img = np.full((h, w), np.nan)                       # NaN = no points (grid shows)
    any_pt = np.zeros((h, w), bool)
    any_pt[ij[:, 1], ij[:, 0]] = True
    img[any_pt] = 0.80
    obst = (z > 0.15) & (z < 2.0)
    cnt = np.zeros((h, w))
    np.add.at(cnt, (ij[obst, 1], ij[obst, 0]), 1)
    img[cnt >= 3] = 0.15

    H = 10 * h / w                                      # map height in inches
    fig = plt.figure(figsize=(10, H + 0.9), dpi=150)
    ax = fig.add_axes([0.03, 0.02, 0.94, H / (H + 0.9) * 0.96])
    ax.set_facecolor('0.96')
    ext = [lo[0], lo[0] + w * res, lo[1], lo[1] + h * res]
    cmap = plt.get_cmap('gray').copy()
    cmap.set_bad(alpha=0)
    ax.imshow(np.ma.masked_invalid(img), cmap=cmap, vmin=0, vmax=1, origin='lower', extent=ext,
              interpolation='nearest', zorder=2)
    for g in np.arange(np.floor(lo[0]), hi[0] + 1):
        ax.axvline(g, color='0.8', lw=0.4, zorder=1)
    for g in np.arange(np.floor(lo[1]), hi[1] + 1):
        ax.axhline(g, color='0.8', lw=0.4, zorder=1)
    ax.plot(traj[:, 1], traj[:, 2], color='#ff7f0e', lw=2.5, zorder=3)
    ax.plot(*traj[0, 1:3], 'o', color='green', ms=9, zorder=4)
    ax.plot(*traj[-1, 1:3], 'o', color='red', ms=9, zorder=4)
    ax.annotate('Start', traj[0, 1:3], xytext=(8, 6), textcoords='offset points', color='green', weight='bold')
    ax.annotate('End', traj[-1, 1:3], xytext=(8, -14), textcoords='offset points', color='red', weight='bold')
    x0, y0 = ext[0] + 0.5, ext[2] + 0.5                 # 5 m scale bar
    ax.plot([x0, x0 + 5], [y0, y0], color='k', lw=3, zorder=4)
    ax.text(x0 + 5.3, y0, '5 m', va='center', fontsize=9, zorder=4)
    ax.set_xlim(ext[:2])
    ax.set_ylim(ext[2:])
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_visible(False)
    when = info.get('started', '')[:16].replace('T', ' ')
    rate = ''
    if st:
        rate = ', scans {:,} of {:,} ({:.1f}%)'.format(
            st['scans_received'], st['scans_expected'], 100 * st['scans_received'] / max(st['scans_expected'], 1))
    fig.text(0.03, 0.99, '{}  -  {}\n{:.1f} min, walked {:.1f} m, end-to-start gap {:.2f} m (vertical {:+.2f} m){}\n'
             'gravity-levelled map, lidar at {:.3f} m; colored {:,} pts, full {:,} pts ({} m voxels)'.format(
                 info['name'], when, info['duration_s'] / 60, info['path_length_m'], gap_h, gap_v, rate,
                 info['lidar_height_m'], info['colored_points'], info['full_points'], info['voxel_m']),
             va='top', family='monospace', fontsize=10)
    fig.savefig(out_png)
    print('\nsaved', out_png)


top_view(folder + '/top_view.png')
