"""Robot export list: validation, persistence, writing and re-exporting."""
import os

import numpy as np

from rgb_lidar_fusion.io_cloud import read_pcd
from rgb_lidar_fusion.robots import describe, load, save, validate, write_exports


def test_validate_accepts_and_normalises():
    clean, err = validate([{'name': ' go2 ', 'height': '0.45'},
                           {'name': 'g1', 'height': 1.25, 'roll': 180, 'pitch': '-5'},
                           {'name': 'old', 'height': 1.0, 'flip': True}])      # older format
    assert err == ''
    assert clean == [dict(name='go2', height=0.45, roll=0.0, pitch=0.0),
                     dict(name='g1', height=1.25, roll=180.0, pitch=-5.0),
                     dict(name='old', height=1.0, roll=180.0, pitch=0.0)]
    assert validate([])[0] == []                                  # no robots is allowed


def test_validate_rejects_bad_input():
    for bad in ([{'name': '', 'height': 0.4}], [{'name': 'a b', 'height': 0.4}],
                [{'name': 'go2', 'height': 0.4}, {'name': 'GO2', 'height': 0.5}],
                [{'name': 'x', 'height': 0.01}], [{'name': 'x', 'height': 9}],
                [{'name': 'x', 'height': 'tall'}], [{'name': '../x', 'height': 0.4}],
                [{'name': 'x', 'height': 0.4, 'roll': 190}],
                [{'name': 'x', 'height': 0.4, 'roll': -5}],
                [{'name': 'x', 'height': 0.4, 'pitch': 95}],
                [{'name': 'r%d' % i, 'height': 0.5} for i in range(11)], 'go2'):
        clean, err = validate(bad)
        assert clean is None and err, bad


def test_save_and_load_roundtrip(tmp_path):
    robots = [dict(name='go2', height=0.45, roll=0.0, pitch=0.0)]
    assert load(str(tmp_path)) is None
    save(str(tmp_path), robots)
    assert load(str(tmp_path)) == robots
    (tmp_path / 'robots.json').write_text('not json')
    assert load(str(tmp_path)) is None                            # broken file: ignored


def test_write_exports_shifts_height_and_removes_dropped_robots(tmp_path):
    xyz = np.array([[1.0, 2.0, 0.0], [0.0, 0.0, 1.5]], np.float32)
    rgb = np.array([[1, 2, 3], [4, 5, 6]], np.uint8)
    inten = np.array([7.0, 8.0], np.float32)
    folder = str(tmp_path)
    write_exports(folder, xyz, rgb, xyz, inten, [dict(name='go2', height=0.45, roll=0, pitch=0),
                                                 dict(name='g1', height=1.25, roll=0, pitch=0)])
    np.testing.assert_allclose(read_pcd(os.path.join(folder, 'g1', 'full.pcd'))['z'],
                               [-1.25, 0.25], atol=1e-6)
    os.makedirs(os.path.join(folder, 'notes'))                    # not an export: untouched
    write_exports(folder, xyz, rgb, xyz, inten, [dict(name='go2w', height=0.53, roll=0, pitch=0)],
                  remove=['go2', 'g1'])
    assert sorted(os.listdir(folder)) == ['go2w', 'notes']
    np.testing.assert_allclose(read_pcd(os.path.join(folder, 'go2w', 'full.pcd'))['z'],
                               [-0.53, 0.97], atol=1e-6)


def test_describe():
    assert describe([dict(name='g1', height=1.25, roll=180.0, pitch=0.0)])[0]['frame'] == \
        'lidar at 1.250 m, roll 180 deg, pitch 0 deg'
