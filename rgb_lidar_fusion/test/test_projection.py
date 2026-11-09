"""Unit tests for pinhole projection and z-buffer occlusion rejection.

Pure maths -- no ROS, no camera. `from_camera_info` is tested against a
duck-typed stand-in so that projection.py stays ROS-import-free.
"""
import numpy as np

from rgb_lidar_fusion.projection import CameraModel, occlusion_mask, project

CAM = CameraModel(K=np.array([[600., 0., 640.], [0., 600., 360.], [0., 0., 1.]]),
                  D=np.zeros(5), width=1280, height=720)


# --- projection ---------------------------------------------------------

def test_principal_axis_hits_principal_point():
    uv, valid = project(CAM, np.array([[0.0, 0.0, 5.0]]))
    assert valid[0]
    np.testing.assert_allclose(uv[0], [640.0, 360.0], atol=1e-9)


def test_known_offset_pixel():
    # x=1, z=5, fx=600 -> u = 640 + 600*(1/5) = 760 ; +Y is DOWN so v grows
    uv, valid = project(CAM, np.array([[1.0, 1.0, 5.0]]))
    assert valid[0]
    np.testing.assert_allclose(uv[0], [760.0, 480.0], atol=1e-9)


def test_depth_scales_disparity():
    """Same ray at twice the depth must land at half the pixel offset."""
    uv, _ = project(CAM, np.array([[1.0, 0.0, 5.0], [2.0, 0.0, 10.0]]))
    np.testing.assert_allclose(uv[0], uv[1], atol=1e-9)


def test_behind_and_outside_are_invalid():
    pts = np.array([[0., 0., -2.],     # behind the camera
                    [0., 0., 0.],      # at the pinhole
                    [50., 0., 1.]])    # far outside the image
    _, valid = project(CAM, pts)
    assert not valid.any()


def test_no_warnings_or_nonfinite_at_zero_depth():
    """z=0 must be rejected before the divide, not after."""
    with np.errstate(all='raise'):
        uv, valid = project(CAM, np.array([[1.0, 1.0, 0.0]]))
    assert not valid[0]
    assert np.isfinite(uv).all()


def test_image_corners_are_valid_and_centre_of_pixel_grid():
    # a point projecting exactly to (0,0) and one to just inside (1279,719)
    pts = np.array([[-640. / 600. * 3., -360. / 600. * 3., 3.],
                    [639. / 600. * 3., 359. / 600. * 3., 3.]])
    uv, valid = project(CAM, pts)
    assert valid.all()
    np.testing.assert_allclose(uv[0], [0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(uv[1], [1279.0, 719.0], atol=1e-6)


def test_empty_input_is_handled():
    uv, valid = project(CAM, np.zeros((0, 3)))
    assert uv.shape == (0, 2) and valid.shape == (0,)


def test_distortion_changes_pixel_off_axis():
    cam_d = CameraModel(K=CAM.K, D=np.array([0.1, -0.05, 0., 0., 0.]),
                        width=1280, height=720)
    p = np.array([[1.0, 0.5, 3.0]])
    assert not np.allclose(project(cam_d, p)[0], project(CAM, p)[0])


def test_distortion_leaves_principal_axis_alone():
    """Radial distortion is zero on the optical axis whatever D is."""
    cam_d = CameraModel(K=CAM.K, D=np.array([0.3, -0.2, 0.01, 0.01, 0.05]),
                        width=1280, height=720)
    uv, valid = project(cam_d, np.array([[0.0, 0.0, 4.0]]))
    assert valid[0]
    np.testing.assert_allclose(uv[0], [640.0, 360.0], atol=1e-9)


def test_from_camera_info_reads_k_d_and_size():
    class FakeInfo:
        k = [600., 0., 640., 0., 600., 360., 0., 0., 1.]
        d = [0.1, 0.2, 0.0, 0.0, 0.0]
        width, height = 1280, 720
    cam = CameraModel.from_camera_info(FakeInfo())
    np.testing.assert_allclose(cam.K, CAM.K)
    np.testing.assert_allclose(cam.D[:5], [0.1, 0.2, 0.0, 0.0, 0.0])
    assert (cam.width, cam.height) == (1280, 720)


def test_from_camera_info_pads_short_distortion():
    class FakeInfo:
        k = [600., 0., 640., 0., 600., 360., 0., 0., 1.]
        d = []                     # some drivers publish an empty D
        width, height = 1280, 720
    cam = CameraModel.from_camera_info(FakeInfo())
    assert cam.D.shape == (5,)
    assert not np.any(cam.D)


# --- occlusion ----------------------------------------------------------

def test_occlusion_keeps_near_rejects_far():
    uv = np.array([[100.0, 100.0], [100.0, 100.0]])
    depth = np.array([2.0, 3.0])
    keep = occlusion_mask(uv, depth, (720, 1280), cell=4, tol=0.15)
    assert keep[0] and not keep[1]


def test_occlusion_keeps_both_within_tolerance():
    uv = np.array([[100.0, 100.0], [100.0, 100.0]])
    depth = np.array([2.0, 2.05])
    keep = occlusion_mask(uv, depth, (720, 1280), cell=4, tol=0.15)
    assert keep.all()


def test_occlusion_is_per_cell_not_global():
    """A far point in its own cell must survive a near point elsewhere."""
    uv = np.array([[10.0, 10.0], [900.0, 500.0]])
    depth = np.array([1.0, 20.0])
    keep = occlusion_mask(uv, depth, (720, 1280), cell=4, tol=0.10)
    assert keep.all()


def test_occlusion_order_independent():
    uv = np.array([[50.0, 50.0]] * 3)
    for order in ([1.0, 5.0, 9.0], [9.0, 5.0, 1.0], [5.0, 9.0, 1.0]):
        keep = occlusion_mask(uv, np.array(order), (720, 1280), cell=4, tol=0.10)
        # exactly the nearest survives
        assert keep.tolist() == [d == min(order) for d in order], order


def test_occlusion_larger_cell_merges_neighbours():
    """Two points 3 px apart share a cell at cell=8 but not at cell=1."""
    uv = np.array([[100.0, 100.0], [103.0, 100.0]])
    depth = np.array([2.0, 9.0])
    assert occlusion_mask(uv, depth, (720, 1280), cell=8, tol=0.1).tolist() == [True, False]
    assert occlusion_mask(uv, depth, (720, 1280), cell=1, tol=0.1).tolist() == [True, True]


def test_occlusion_empty_input():
    keep = occlusion_mask(np.zeros((0, 2)), np.zeros(0), (720, 1280))
    assert keep.shape == (0,)


def test_occlusion_default_tol_is_tight():
    """The lidar-camera baseline makes this mask load-bearing, so the default
    must not be loose enough to pass background through."""
    uv = np.array([[100.0, 100.0], [100.0, 100.0]])
    keep = occlusion_mask(uv, np.array([2.0, 2.2]), (720, 1280))
    assert keep.tolist() == [True, False]
