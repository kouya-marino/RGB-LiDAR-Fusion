"""Robot exports: which robots get a copy of each saved map, and writing them.

A robot export is the saved map (map frame: floor at z = 0) re-expressed with
the origin at that robot's lidar, as if the robot's lidar had started where the
rig's lidar did: p_robot = p_map - (0, 0, height), then rotated for a lidar
that is tilted: pitch (nose down positive) and roll about its forward axis
(180 = upside down), as its driver reports it (see mapframe.robot_frame).

The list is edited in the web UI and kept in <output_dir>/robots.json, so it
survives restarts without touching the tracked config. Pure Python + numpy.
"""
import json
import os
import re
import shutil
from typing import List, Optional, Sequence, Tuple

import numpy as np

from rgb_lidar_fusion.io_cloud import write_both, write_pcd_xyzi, write_ply_xyzi
from rgb_lidar_fusion.mapframe import apply, robot_frame

NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,32}$')
MAX_ROBOTS = 10
MIN_HEIGHT, MAX_HEIGHT = 0.05, 5.0
MIN_ROLL, MAX_ROLL = 0.0, 180.0
MIN_PITCH, MAX_PITCH = -90.0, 90.0
FILENAME = 'robots.json'


def validate(robots) -> Tuple[Optional[List[dict]], str]:
    """(clean list, '') or (None, reason). Each robot: name, height (m), roll and
    pitch (deg). An older `flip: true` is read as roll 180."""
    if not isinstance(robots, list):
        return None, 'robots must be a list'
    if len(robots) > MAX_ROBOTS:
        return None, 'at most {} robots'.format(MAX_ROBOTS)
    clean, seen = [], set()
    for i, r in enumerate(robots, 1):
        if not isinstance(r, dict):
            return None, 'robot {} is not an object'.format(i)
        name = str(r.get('name', '')).strip()
        if not NAME_RE.match(name):
            return None, 'robot {}: name must be 1-32 of A-Z a-z 0-9 _ -'.format(i)
        if name.lower() in seen:
            return None, 'robot name {!r} is used twice'.format(name)
        seen.add(name.lower())
        try:
            height = float(r.get('height'))
        except (TypeError, ValueError):
            return None, 'robot {!r}: height must be a number'.format(name)
        if not MIN_HEIGHT <= height <= MAX_HEIGHT:
            return None, 'robot {!r}: height must be {:.0f}-{:.0f} cm'.format(
                name, MIN_HEIGHT * 100, MAX_HEIGHT * 100)
        angles = {}
        for key, lo, hi in (('roll', MIN_ROLL, MAX_ROLL), ('pitch', MIN_PITCH, MAX_PITCH)):
            default = 180.0 if key == 'roll' and r.get('flip') else 0.0
            try:
                angles[key] = float(r.get(key, default))
            except (TypeError, ValueError):
                return None, 'robot {!r}: {} must be a number'.format(name, key)
            if not lo <= angles[key] <= hi:
                return None, 'robot {!r}: {} must be {:.0f} to {:.0f} deg'.format(name, key, lo, hi)
        clean.append(dict(name=name, height=round(height, 4), roll=round(angles['roll'], 2),
                          pitch=round(angles['pitch'], 2)))
    return clean, ''


def load(output_dir: str) -> Optional[List[dict]]:
    """The saved list, or None if there is none (or it is unreadable/invalid)."""
    path = os.path.join(output_dir, FILENAME)
    try:
        with open(path) as fh:
            robots, _ = validate(json.load(fh))
    except (OSError, ValueError):
        return None
    return robots


def save(output_dir: str, robots: Sequence[dict]) -> None:
    os.makedirs(output_dir, exist_ok=True)
    tmp = os.path.join(output_dir, '.' + FILENAME + '.tmp')
    with open(tmp, 'w') as fh:
        json.dump(list(robots), fh, indent=2)
    os.replace(tmp, os.path.join(output_dir, FILENAME))


def describe(robots: Sequence[dict]) -> List[dict]:
    """The robots as recorded in a map's info.json."""
    return [dict(r, frame='lidar at {:.3f} m, roll {:g} deg, pitch {:g} deg'.format(
        r['height'], r['roll'], r['pitch'])) for r in robots]


def write_exports(folder: str, xyz_c: np.ndarray, rgb: np.ndarray, xyz_f: np.ndarray,
                  inten: np.ndarray, robots: Sequence[dict],
                  remove: Sequence[str] = ()) -> None:
    """Write <folder>/<robot>/{colored,full}.{pcd,ply} for each robot from the
    map-frame clouds, and delete the folders of robots in `remove` that are no
    longer in the list (only those: unknown folders are never touched)."""
    for r in robots:
        where = os.path.join(folder, r['name'])
        os.makedirs(where, exist_ok=True)
        T = robot_frame(r['height'], r.get('roll', 0.0), r.get('pitch', 0.0))
        write_both(os.path.join(where, 'colored'), apply(T, xyz_c), rgb)
        pts = apply(T, xyz_f)
        write_pcd_xyzi(os.path.join(where, 'full.pcd'), pts, inten)
        write_ply_xyzi(os.path.join(where, 'full.ply'), pts, inten)
    keep = {r['name'] for r in robots}
    for name in remove:
        if name not in keep and NAME_RE.match(str(name)):
            shutil.rmtree(os.path.join(folder, name), ignore_errors=True)
