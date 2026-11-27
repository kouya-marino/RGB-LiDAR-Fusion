"""Animated GIF of a saved map: three synchronized views swaying around the map.

usage: python3 tools/render_orbit_gif.py <map_dir> <out.gif> [--width 1212] [--frames 19]

Views, left to right: camera colors inside the full lidar map, the camera-colored
points only, and the full lidar map colored by height. The ceiling (above 2.2 m)
is hidden. Reads <map_dir>/colored.pcd and full.pcd (binary or ascii PCD), e.g.
examples/full_floor_sample.
"""
import argparse
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_map import BG, CLIP_Z, bold_font, height_rgb, load_pcd, paint, unpack_rgb  # noqa: E402


def voxel_keys(p, v):
    k = np.floor(p / v).astype(np.int64) + (1 << 20)
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map_dir')
    ap.add_argument('out')
    ap.add_argument('--width', type=int, default=1212, help='whole GIF, three panels')
    ap.add_argument('--frames', type=int, default=19)
    ap.add_argument('--sway', type=float, default=25.0, help='azimuth swing, degrees')
    a = ap.parse_args()

    col = load_pcd(os.path.join(a.map_dir, 'colored.pcd'))
    full = load_pcd(os.path.join(a.map_dir, 'full.pcd'))
    cxyz, crgb = col[:, :3].astype(np.float64), unpack_rgb(col[:, 3])
    fxyz = full[:, :3].astype(np.float64)
    keep_c = (cxyz[:, 2] < CLIP_Z) & (cxyz[:, 2] > -0.3)
    keep_f = (fxyz[:, 2] < CLIP_Z) & (fxyz[:, 2] > -0.3)
    cxyz, crgb, fxyz = cxyz[keep_c], crgb[keep_c], fxyz[keep_f]
    fh = height_rgb(fxyz[:, 2])

    # view 1: lidar points only where no camera color fell in the same 10 cm voxel
    only_lidar = ~np.isin(voxel_keys(fxyz, 0.10), voxel_keys(cxyz, 0.10))
    views = [
        ('Lidar + camera', np.vstack([fxyz[only_lidar], cxyz]), np.vstack([fh[only_lidar], crgb]),
         np.r_[np.zeros(only_lidar.sum()), np.ones(len(cxyz))]),
        ('Camera colors', cxyz, crgb, np.ones(len(cxyz))),
        ('Full lidar (height)', fxyz, fh, np.zeros(len(fxyz))),
    ]

    lo = np.percentile(fxyz[:, :2], 1, axis=0)
    hi = np.percentile(fxyz[:, :2], 99, axis=0)
    ctr = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, 0.0])
    span = float(max(hi - lo))
    pw = a.width // 3
    strip = 28
    ph = int(round(pw * 0.63))
    font = bold_font(15)
    elev = np.deg2rad(48)

    frames = []
    for i in range(a.frames):
        azim = np.deg2rad(-60 + a.sway * np.sin(2 * np.pi * i / a.frames))
        fwd = -np.array([np.cos(elev) * np.cos(azim), np.cos(elev) * np.sin(azim), np.sin(elev)])
        eye = ctr - fwd * span * 1.75
        right = np.cross(fwd, [0, 0, 1.0])
        right /= np.linalg.norm(right)
        up = np.cross(right, fwd)
        f = pw * 1.05
        frame = Image.new('RGB', (pw * 3, ph + strip), tuple(int(v) for v in BG))
        for k, (title, pts, rgb, pri) in enumerate(views):
            v = pts - eye
            z = v @ fwd
            ok = z > 0.1
            u = f * (v[ok] @ right) / z[ok] + pw / 2
            w = -f * (v[ok] @ up) / z[ok] + ph / 2
            img = np.tile(BG, (ph, pw, 1))
            paint(img, np.stack([w.astype(int), u.astype(int)], 1), rgb[ok],
                  np.lexsort((pri[ok], -z[ok])), splat=2)        # far first; camera wins ties
            panel = Image.fromarray(img)
            frame.paste(panel, (k * pw, strip))
            d = ImageDraw.Draw(frame)
            tw = d.textlength(title, font=font)
            d.text((k * pw + (pw - tw) / 2, 5), title, fill=(235, 235, 240), font=font)
        frames.append(frame.quantize(colors=255, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE))
        print('frame', i + 1, '/', a.frames, flush=True)
    frames[0].save(a.out, save_all=True, append_images=frames[1:], duration=140, loop=0,
                   optimize=True)
    print('saved', a.out, '({:.1f} MB)'.format(os.path.getsize(a.out) / 1e6))


if __name__ == '__main__':
    main()
