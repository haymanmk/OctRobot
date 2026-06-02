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
