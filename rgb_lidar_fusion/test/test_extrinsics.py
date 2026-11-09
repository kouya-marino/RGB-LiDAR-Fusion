"""Unit tests for the lidar->camera extrinsic representation.

These are pure-maths tests: no ROS, no robot, no camera.
"""
import numpy as np
import pytest

from rgb_lidar_fusion.extrinsics import (
    invert,
    load_extrinsic,
    save_extrinsic,
    se3,
    to_rpy_xyz,
)


def test_se3_roundtrip():
    rpy, xyz = [10.0, -20.0, 30.0], [0.1, -0.2, 0.3]
    T = se3(rpy, xyz)
    assert T.shape == (4, 4)
    np.testing.assert_allclose(T[3], [0, 0, 0, 1])
    np.testing.assert_allclose(T[:3, 3], xyz)
    rpy2, xyz2 = to_rpy_xyz(T)
    np.testing.assert_allclose(rpy2, rpy, atol=1e-9)
    np.testing.assert_allclose(xyz2, xyz, atol=1e-12)


def test_invert_is_true_inverse():
    T = se3([5.0, 12.0, -80.0], [1.0, 2.0, 3.0])
    np.testing.assert_allclose(invert(T) @ T, np.eye(4), atol=1e-12)
    np.testing.assert_allclose(T @ invert(T), np.eye(4), atol=1e-12)


def test_rotation_is_orthonormal():
    R = se3([33.0, -44.0, 55.0], [0, 0, 0])[:3, :3]
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(R) > 0


def test_identity():
    np.testing.assert_allclose(se3([0, 0, 0], [0, 0, 0]), np.eye(4), atol=1e-15)


def test_yaml_roundtrip(tmp_path):
    p = tmp_path / 'e.yaml'
    T = se3([1.0, 2.0, 3.0], [0.05, 0.0, 0.12])
    save_extrinsic(str(p), T, notes='unit test')
    # 1e-10, not 1e-12: save_extrinsic rounds rpy to 9 dp so the file stays
    # legible for hand-editing. That is ~3e-6 arcsec -- physically nothing.
    np.testing.assert_allclose(load_extrinsic(str(p)), T, atol=1e-10)


def test_yaml_is_human_editable(tmp_path):
    """The tuner in Task 4 hand-edits this file, so it must hold rpy/xyz --
    not an opaque 16-number matrix."""
    p = tmp_path / 'e.yaml'
    save_extrinsic(str(p), se3([-90.0, 0.0, -90.0], [0.48, 0.0, 0.0]),
                   notes='measured 2026-08-27')
    text = p.read_text()
    assert 'rpy_deg' in text and 'xyz_m' in text
    assert 'measured 2026-08-27' in text


def test_yaml_notes_survive_roundtrip_without_breaking_parse(tmp_path):
    p = tmp_path / 'e.yaml'
    T = se3([0.0, 0.0, 0.0], [1.0, 2.0, 3.0])
    save_extrinsic(str(p), T, notes='line one\nline two with: a colon')
    np.testing.assert_allclose(load_extrinsic(str(p)), T, atol=1e-10)


def test_load_rejects_missing_keys(tmp_path):
    p = tmp_path / 'bad.yaml'
    p.write_text('rpy_deg: [0, 0, 0]\n')       # xyz_m missing
    with pytest.raises(KeyError):
        load_extrinsic(str(p))


# --- the reference mount (hardware/mount/) ------------------------------
# livox_frame -> camera optical frame for the Mid-360 (scan axis 45 deg up) +
# D435i (20 deg down) mount: rpy [-65, 0, -90] deg, origin at camera_link.
# Documented in hardware/mount/README.md as T_lidar_optical.

MOUNT_RPY = [-65.0, 0.0, -90.0]
MOUNT_XYZ = [0.0842, 0.0175, -0.0669]


def test_reference_mount_matches_documented_matrix():
    T = se3(MOUNT_RPY, MOUNT_XYZ)
    expected = np.array([[0.0, 0.4226, 0.9063, 0.0842],
                         [-1.0, 0.0, 0.0, 0.0175],
                         [0.0, -0.9063, 0.4226, -0.0669],
                         [0.0, 0.0, 0.0, 1.0]])
    np.testing.assert_allclose(T, expected, atol=1e-4)


def test_reference_mount_camera_right_is_lidar_right():
    R = se3(MOUNT_RPY, MOUNT_XYZ)[:3, :3]
    # camera +X (right) -> lidar -Y (right, since lidar +Y is left)
    np.testing.assert_allclose(R @ [1, 0, 0], [0, -1, 0], atol=1e-12)


def test_point_ahead_of_lidar_lands_in_front_of_camera():
    """A point 5 m along lidar +X must have positive depth in the camera frame.
    A sign error on the baseline or a flipped axis fails this immediately."""
    T_lidar_cam = se3(MOUNT_RPY, MOUNT_XYZ)
    p_cam = invert(T_lidar_cam) @ np.array([5.0, 0.0, 0.0, 1.0])
    assert p_cam[2] > 0, 'point ahead of the lidar must be in front of the camera'
    # 25 deg between the lidar X axis and the camera optical axis
    np.testing.assert_allclose(p_cam[2], (5.0 - 0.0842) * np.cos(np.radians(25))
                               + 0.0669 * np.sin(np.radians(25)), atol=1e-3)
    # only the 17.5 mm sideways offset separates them horizontally ...
    np.testing.assert_allclose(p_cam[0], 0.0175, atol=1e-4)
    # ... and the lidar X axis points 25 deg below the camera axis (+Y is down)
    assert p_cam[1] > 0
    np.testing.assert_allclose(np.degrees(np.arctan2(p_cam[1] + 0.0963, p_cam[2] + 0.0480)),
                               25.0, atol=0.05)


def test_yaml_values_are_legible_not_float_noise(tmp_path):
    """A file meant for hand-nudging must not contain -89.99999999999999."""
    p = tmp_path / 'e.yaml'
    save_extrinsic(str(p), se3([-90.0, 0.0, -90.0], [0.48, 0.0, 0.0]))
    text = p.read_text()
    assert 'rpy_deg: [-90.0, 0.0, -90.0]' in text, text
    assert 'xyz_m: [0.48, 0.0, 0.0]' in text, text
    assert 'e-' not in text.split('rpy_deg')[1]     # no exponent notation


def test_no_scipy_dependency():
    """The interpreter ROS nodes run under (/usr/bin/python3) has an apt scipy
    1.3.3 paired with numpy 1.24, and that scipy calls the long-removed
    np.typeDict -- so importing it anywhere here kills every launched node.

    Checked via the AST, not a substring search: the module docstring legitimately
    mentions `from scipy...` while explaining exactly this, and a naive text match
    fails on its own explanation.
    """
    import ast
    import rgb_lidar_fusion.extrinsics as ex
    tree = ast.parse(open(ex.__file__.replace('.pyc', '.py')).read())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(a.name.split('.')[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                imported.add(node.module.split('.')[0])
    assert 'scipy' not in imported, sorted(imported)


def test_euler_roundtrip_over_a_wide_sweep():
    for roll in (-170.0, -90.0, -12.0, 0.0, 45.0, 179.0):
        for pitch in (-80.0, -30.0, 0.0, 30.0, 80.0):
            for yaw in (-175.0, -60.0, 0.0, 60.0, 175.0):
                T = se3([roll, pitch, yaw], [0.1, 0.2, 0.3])
                rpy2, _ = to_rpy_xyz(T)
                np.testing.assert_allclose(se3(rpy2, [0.1, 0.2, 0.3]), T,
                                           atol=1e-9,
                                           err_msg=str((roll, pitch, yaw)))


def test_gimbal_lock_still_yields_the_same_rotation():
    """At pitch = +/-90 roll and yaw are not separable; we must still recover a
    (roll, pitch, yaw) that rebuilds the SAME matrix."""
    for pitch in (90.0, -90.0):
        for roll, yaw in ((0.0, 0.0), (30.0, 0.0), (0.0, 40.0), (25.0, -35.0)):
            T = se3([roll, pitch, yaw], [0.0, 0.0, 0.0])
            rpy2, _ = to_rpy_xyz(T)
            np.testing.assert_allclose(se3(rpy2, [0.0, 0.0, 0.0]), T, atol=1e-9,
                                       err_msg=str((roll, pitch, yaw)))
