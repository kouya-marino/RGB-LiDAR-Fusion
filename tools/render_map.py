"""Render paper figures from a saved RGB-LiDAR-Fusion map folder.

usage: python3 tools/render_map.py <map_dir> <out_dir> [<colored.pcd> [<full.pcd>]]
The optional third argument replaces the map's colored.pcd (e.g. a copy cleaned in
AlchemyCloud); the full lidar map is then cropped to that cloud's XY extent, unless
a cleaned full map is given as the fourth argument (used as is).
PCD files may be binary or ascii, with fields x y z [rgb|intensity].
Writes <out_dir>/fullroom_topdown.png and <out_dir>/fullroom_oblique.png.
"""
import csv
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import cm
from PIL import Image, ImageDraw, ImageFont

CLIP_Z = 2.2          # hide ceiling (m)
BG = np.array([11, 13, 20], np.uint8)


def load_pcd(path):
    """N x 4 float32 (x, y z, rgb-or-intensity; 0 if the file has only x y z)."""
    with open(path, 'rb') as f:
        hdr = {}
        while True:
            k, *v = f.readline().decode().strip().split(' ')
            hdr[k] = v
            if k == 'DATA':
                break
        n, nf = int(hdr['POINTS'][0]), len(hdr['FIELDS'])
        if hdr['DATA'][0] == 'binary':
            a = np.frombuffer(f.read(n * 4 * nf), dtype=np.float32).reshape(n, nf)
        else:                          # ascii: packed rgb floats must stay float32
            a = np.loadtxt(f, dtype=np.float32, ndmin=2)
    out = np.zeros((len(a), 4), np.float32)
    out[:, :nf] = a[:, :4]
    return out


def unpack_rgb(f):
    u = f.view(np.uint32)
    return np.stack([(u >> 16) & 255, (u >> 8) & 255, u & 255], 1).astype(np.uint8)


def height_rgb(z):
    t = np.clip(z / CLIP_Z, 0, 1)
    return (cm.viridis(0.25 + 0.75 * t)[:, :3] * 255).astype(np.uint8)


def dominant_yaw(xy):
    """Yaw that aligns the walls with the image axes (sharpest x/y histograms)."""
    best, best_a = -1, 0.0
    for a in np.deg2rad(np.arange(0, 90, 0.5)):
        c, s = np.cos(a), np.sin(a)
        r = xy @ np.array([[c, -s], [s, c]])
        score = 0
        for k in range(2):
            h, _ = np.histogram(r[:, k], bins=np.arange(r[:, k].min(), r[:, k].max() + 0.05, 0.05))
            score += (h.astype(float) ** 2).sum()
        if score > best:
            best, best_a = score, a
    return best_a


def paint(img, ij, rgb, order, splat=1):
    H, W, _ = img.shape
    ij, rgb = ij[order], rgb[order]
    for dy in range(splat):
        for dx in range(splat):
            i, j = ij[:, 0] + dy, ij[:, 1] + dx
            ok = (i >= 0) & (i < H) & (j >= 0) & (j < W)
            img[i[ok], j[ok]] = rgb[ok]   # later entries win


def bold_font(size):
    try:
        return ImageFont.truetype('DejaVuSans-Bold.ttf', size)
    except OSError:                    # font not installed: PIL's built-in one
        return ImageFont.load_default()


def label(im, text, xy=None):
    strip = max(40, im.width // 28)
    out = Image.new('RGB', (im.width, im.height + strip), tuple(int(v) for v in BG))
    out.paste(im, (0, strip))
    im = out
    d = ImageDraw.Draw(im)
    font = bold_font(max(18, im.width // 40))
    w = d.textlength(text, font=font)
    d.text(xy or ((im.width - w) / 2, strip * 0.2), text, fill=(235, 235, 240), font=font)
    return im


def main(map_dir, out_dir, colored_path=None, full_path=None):
    col = load_pcd(colored_path or map_dir + '/colored.pcd')
    full = load_pcd(full_path or map_dir + '/full.pcd')
    if colored_path and not full_path:
        lo_c = col[:, :2].min(0) - 0.2
        hi_c = col[:, :2].max(0) + 0.2
        inside = np.all((full[:, :2] >= lo_c) & (full[:, :2] <= hi_c), axis=1)
        full = full[inside]
    with open(map_dir + '/trajectory.csv') as f:
        traj = np.array([[float(r['x']), float(r['y']), float(r['z'])] for r in csv.DictReader(f)])

    cxyz, crgb = col[:, :3], unpack_rgb(col[:, 3])
    fxyz = full[:, :3]
    keep_c = (cxyz[:, 2] < CLIP_Z) & (cxyz[:, 2] > -0.3)
    keep_f = (fxyz[:, 2] < CLIP_Z) & (fxyz[:, 2] > -0.3)
    cxyz, crgb, fxyz = cxyz[keep_c], crgb[keep_c], fxyz[keep_f]

    walls = fxyz[(fxyz[:, 2] > 0.8) & (fxyz[:, 2] < 2.0), :2]
    sub = walls[np.random.default_rng(0).choice(len(walls), min(300000, len(walls)), replace=False)]
    yaw = dominant_yaw(sub)
    c, s = np.cos(yaw), np.sin(yaw)
    R2 = np.array([[c, -s], [s, c]])

    def rot(p):
        q = p.copy()
        q[:, :2] = p[:, :2] @ R2
        return q
    cxyz, fxyz, traj = rot(cxyz), rot(fxyz), rot(traj)

    # Bounds: where the walls are (robust to points seen through windows/doors).
    lo = np.percentile(fxyz[:, :2], 0.5, axis=0) - 0.5
    hi = np.percentile(fxyz[:, :2], 99.5, axis=0) + 0.5
    if hi[0] - lo[0] < hi[1] - lo[1]:       # landscape panels
        swap = True
        cxyz, fxyz, traj = cxyz[:, [1, 0, 2]], fxyz[:, [1, 0, 2]], traj[:, [1, 0, 2]]
        lo, hi = lo[::-1], hi[::-1]
    else:
        swap = False
    W = 1800
    res = (hi[0] - lo[0]) / W
    H = int(np.ceil((hi[1] - lo[1]) / res))

    def to_ij(p):
        return np.stack([((hi[1] - p[:, 1]) / res).astype(int), ((p[:, 0] - lo[0]) / res).astype(int)], 1)

    # ---- top-down: three panels, like the demo GIF ----
    fh = height_rgb(fxyz[:, 2])
    panels = []
    # 1. lidar + camera: everything sorted by height, camera colour wins ties
    def vkey(p):
        k = np.floor(p / 0.03).astype(np.int64) + (1 << 20)
        return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]
    only_lidar = ~np.isin(vkey(fxyz), vkey(cxyz))
    allp = np.vstack([fxyz[only_lidar], cxyz])
    allc = np.vstack([fh[only_lidar], crgb])
    pri = np.concatenate([np.zeros(only_lidar.sum()), np.ones(len(cxyz))])
    img = np.tile(BG, (H, W, 1))
    paint(img, to_ij(allp), allc, np.lexsort((pri, allp[:, 2])))
    panels.append(('Lidar + camera', img))
    img = np.tile(BG, (H, W, 1))
    paint(img, to_ij(cxyz), crgb, np.argsort(cxyz[:, 2]))
    panels.append(('Camera colors', img))
    img = np.tile(BG, (H, W, 1))
    paint(img, to_ij(fxyz), fh, np.argsort(fxyz[:, 2]))
    panels.append(('Full lidar (height) + path', img))

    ims = []
    for title, a in panels:
        im = Image.fromarray(a)
        if 'path' in title:
            d = ImageDraw.Draw(im)
            pts = [tuple(v) for v in to_ij(traj)[:, ::-1]]
            d.line(pts, fill=(255, 80, 60), width=3)
            d.ellipse([pts[0][0] - 8, pts[0][1] - 8, pts[0][0] + 8, pts[0][1] + 8], fill=(255, 255, 255))
        ims.append(label(im, title))
    # scale bar on the first panel
    H = ims[0].height
    d = ImageDraw.Draw(ims[0])
    L = int(5.0 / res)
    d.rectangle([20, H - 30, 20 + L, H - 24], fill=(235, 235, 240))
    d.text((20, H - 58), '5 m', fill=(235, 235, 240),
           font=bold_font(max(16, W // 50)))
    gap = 12
    out = Image.new('RGB', (W, 3 * H + 2 * gap), tuple(int(v) for v in BG))
    vertical = True
    if W / H < 1.6:  # tall-ish rooms: lay panels side by side
        vertical = False
        out = Image.new('RGB', (3 * W + 2 * gap, H), tuple(int(v) for v in BG))
    for k, im in enumerate(ims):
        out.paste(im, (0, k * (H + gap)) if vertical else (k * (W + gap), 0))
    out.save(out_dir + '/fullroom_topdown.png', optimize=True)

    # ---- oblique perspective view of the combined map ----
    ctr = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, 0.0])
    span = max(hi - lo)
    elev, azim = np.deg2rad(50), np.deg2rad(-60)
    fwd = -np.array([np.cos(elev) * np.cos(azim), np.cos(elev) * np.sin(azim), np.sin(elev)])
    eye = ctr - fwd * span * 1.45
    right = np.cross(fwd, [0, 0, 1.0]); right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    W2, H2 = 2400, 1500
    f = W2 * 0.95

    def cam(p):
        v = p - eye
        z = v @ fwd
        u = f * (v @ right) / z + W2 / 2
        w = -f * (v @ up) / z + H2 / 2
        return np.stack([w.astype(int), u.astype(int)], 1), z
    ij, z = cam(allp)
    img = np.tile(BG, (H2, W2, 1))
    order = np.lexsort((pri, -z))          # far first, camera colour after lidar at equal depth
    paint(img, ij, allc, order, splat=2)
    im = Image.fromarray(img)
    im = label(im, 'Full map: camera colors, lidar elsewhere (ceiling hidden)')
    im.save(out_dir + '/fullroom_oblique.png', optimize=True)
    print('yaw %.1f deg, swap %s, top-down %dx%d px (%.1f x %.1f m), res %.3f m/px'
          % (np.rad2deg(yaw), swap, W, H, hi[0] - lo[0], hi[1] - lo[1], res))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], *sys.argv[3:5])
