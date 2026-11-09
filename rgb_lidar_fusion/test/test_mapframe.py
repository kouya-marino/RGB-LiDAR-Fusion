"""The levelled map frame and the robot export frames."""
import numpy as np
import pytest

from rgb_lidar_fusion.extrinsics import se3
from rgb_lidar_fusion.mapframe import (apply, gravity_up, level_frame, robot_frame,
                                       rotation_angle)


def test_gravity_up_from_g_units_and_m_s2():
    np.testing.assert_allclose(gravity_up([[0, 0, 1.0], [0, 0, 0.98]]), [0, 0, 1])
    np.testing.assert_allclose(gravity_up([[0, 9.81, 0]]), [0, 1, 0])
    with pytest.raises(ValueError):
        gravity_up([[0, 0, 0]])


def test_level_frame_identity_when_world_is_already_level():
    T = level_frame(up=[0, 0, 1], forward_hint=[1, 0, 0], origin=[0, 0, 0], height=1.0)
    np.testing.assert_allclose(apply(T, [[0, 0, 0], [2, 0, -1]]), [[0, 0, 1], [2, 0, 0]])


def test_tilted_lidar_on_pole_levels_floor_to_zero():
    """Reference mount: lidar pitched 45 deg nose-down, 1.0 m above the floor.
    The world frame is the lidar's own start pose (as FAST-LIO's camera_init)."""
    h = 1.0
    R_true = se3([0, 45, 0], [0, 0, 0])[:3, :3]        # true lidar orientation in a level frame
    up_world = R_true.T @ [0, 0, 1]                      # 'up' as the tilted lidar sees it
    T = level_frame(up=up_world, forward_hint=[1, 0, 0], origin=[0, 0, 0], height=h)
    # floor points as the tilted lidar sees them: take level-frame points, express in lidar
    floor_level = np.array([[1.0, 0.5, -h], [3.0, -2.0, -h], [-1.0, 1.0, -h]])
    floor_lidar = floor_level @ R_true                   # = R_true.T @ p for each row
    got = apply(T, floor_lidar)
    np.testing.assert_allclose(got[:, 2], 0.0, atol=1e-9)     # floor at z = 0
    np.testing.assert_allclose(got[:, :2], floor_level[:, :2], atol=1e-9)  # x forward kept


def test_origin_is_on_the_floor_below_the_start_lidar():
    origin = np.array([2.0, -1.0, 0.3])                  # lidar start position in the world
    T = level_frame(up=[0, 0, 1], forward_hint=[0, 1, 0], origin=origin, height=1.2)
    np.testing.assert_allclose(apply(T, [origin]), [[0, 0, 1.2]], atol=1e-12)
    # forward was world +Y, so world +Y becomes map +X
    np.testing.assert_allclose(apply(T, [origin + [0, 1, 0]]), [[1, 0, 1.2]], atol=1e-12)


def test_level_frame_rejects_vertical_forward():
    with pytest.raises(ValueError):
        level_frame(up=[0, 0, 1], forward_hint=[0, 0, 1], origin=[0, 0, 0], height=1)


def test_robot_frames_go2_and_g1():
    floor_pt = [[3.0, 1.0, 0.0]]
    np.testing.assert_allclose(apply(robot_frame(0.45), floor_pt), [[3, 1, -0.45]])
    np.testing.assert_allclose(apply(robot_frame(1.25), floor_pt), [[3, 1, -1.25]])
    # a lidar reported upside down: z flips, y flips (180 deg about X)
    np.testing.assert_allclose(apply(robot_frame(1.25, flip=True), floor_pt), [[3, -1, 1.25]])


def test_rotation_angle():
    R0 = np.eye(3)
    R1 = se3([0, 0, 30], [0, 0, 0])[:3, :3]
    assert abs(np.degrees(rotation_angle(R0, R1)) - 30) < 1e-9


def test_path_length_ignores_jitter_but_counts_walking():
    from rgb_lidar_fusion.mapframe import path_length, thin_path
    rng = np.random.RandomState(0)
    still = rng.normal(scale=0.003, size=(300, 3))          # 3 mm jitter, standing still
    assert path_length(still) < 0.05
    walk = np.stack([np.linspace(0, 5, 200), np.zeros(200), np.zeros(200)], 1)
    walk += rng.normal(scale=0.003, size=walk.shape)
    assert abs(path_length(walk) - 5.0) < 0.1
    thinned = thin_path(walk)
    np.testing.assert_allclose(thinned[-1], walk[-1])       # ends where the rig is
