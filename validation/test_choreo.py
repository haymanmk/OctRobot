import numpy as np
import choreo


def test_rpy_to_matrix_matches_home_pose():
    # Firmware home orientation is RPY (deg) = (-90, 0, -90), ZYX.
    # Expected rotation matrix from robot_geometry.c home config M:
    R = choreo.rpy_to_matrix(-90.0, 0.0, -90.0)
    expected = np.array([[0, 0, 1],
                         [-1, 0, 0],
                         [0, -1, 0]], dtype=float)
    np.testing.assert_allclose(R, expected, atol=1e-6)


def test_rpy_matrix_roundtrip():
    for rpy in [(-90, 0, -90), (10, 20, 30), (0, 0, 0), (45, -15, 170)]:
        R = choreo.rpy_to_matrix(*rpy)
        back = choreo.matrix_to_rpy(R)
        R2 = choreo.rpy_to_matrix(*back)
        np.testing.assert_allclose(R, R2, atol=1e-6)


def test_load_model_shapes():
    m = choreo.load_model()
    assert m["Slist"].shape == (6, 6)
    assert m["M"].shape == (4, 4)
    assert m["Blist"].shape == (6, 6)
    assert m["jmin"].shape == (6,) and m["jmax"].shape == (6,)
    assert m["n"] == 6


def test_fk_zero_equals_home():
    m = choreo.load_model()
    T = choreo.fk(m, np.zeros(6))
    np.testing.assert_allclose(T, m["M"], atol=1e-9)


def test_ik_recovers_home():
    m = choreo.load_model()
    theta, ok = choreo.ik_solve(m, m["M"], seed=np.zeros(6))
    assert ok
    # FK of the solution returns to the home pose
    np.testing.assert_allclose(choreo.fk(m, theta), m["M"], atol=1e-4)


def test_orientation_from_axis_points_z_along_target():
    p = np.array([0.0, 0.0, 0.0])
    target = np.array([0.3, 0.1, 0.05])
    R = choreo.orientation_from_axis(target - p)
    z = R[:, 2]
    expected = (target - p) / np.linalg.norm(target - p)
    np.testing.assert_allclose(z, expected, atol=1e-6)
    # R must be a proper rotation (orthonormal, det +1)
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-6)
    np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-6)


def test_pin_pose_keeps_tip_at_target():
    target = np.array([0.2, 0.0, 0.18])
    R = choreo.orientation_from_axis(np.array([-1.0, 0.0, 0.0]))
    L = 0.02
    p, Rout = choreo.pin_pose(target, R, L)
    tip = p + Rout @ np.array([0.0, 0.0, L])
    np.testing.assert_allclose(tip, target, atol=1e-9)


def test_orientation_from_axis_roll_continuity():
    axis1 = np.array([0.3, 0.2, 0.5])
    axis2 = np.array([0.35, 0.2, 0.5])
    R1 = choreo.orientation_from_axis(axis1)
    R2 = choreo.orientation_from_axis(axis2, prev_R=R1)
    # z still points along axis2
    np.testing.assert_allclose(R2[:, 2], axis2 / np.linalg.norm(axis2), atol=1e-6)
    # frame is still a proper rotation
    np.testing.assert_allclose(np.linalg.det(R2), 1.0, atol=1e-6)
    # with continuity, x-axis stays at least as aligned to the previous frame
    dot_with = np.dot(R1[:, 0], R2[:, 0])
    dot_without = np.dot(R1[:, 0], choreo.orientation_from_axis(axis2)[:, 0])
    assert dot_with >= dot_without - 1e-6
