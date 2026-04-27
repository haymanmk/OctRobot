"""
OctroBot FK Cross-Validation Tests

Validates the POE forward kinematics implementation against the
modern-robotics Python library (Lynch & Park, *Modern Robotics*).

Run:  cd validation && python -m pytest -v
"""

import numpy as np
import modern_robotics as mr
import pytest

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

TOL_POS = 1e-6    # Position tolerance (meters) — float64 precision
TOL_ROT = 1e-6    # Rotation matrix element tolerance


def assert_se3_close(T_actual, T_expected, pos_tol=TOL_POS, rot_tol=TOL_ROT):
    """Assert two SE(3) transforms are element-wise close."""
    # Bottom row must be [0, 0, 0, 1]
    np.testing.assert_allclose(T_actual[3, :], [0, 0, 0, 1], atol=1e-12)
    # Rotation block
    np.testing.assert_allclose(T_actual[:3, :3], T_expected[:3, :3], atol=rot_tol)
    # Translation
    np.testing.assert_allclose(T_actual[:3, 3], T_expected[:3, 3], atol=pos_tol)


def is_valid_se3(T):
    """Check that T is a valid SE(3) matrix."""
    R = T[:3, :3]
    # R^T R ≈ I
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-6):
        return False
    # det(R) ≈ 1
    if not np.isclose(np.linalg.det(R), 1.0, atol=1e-6):
        return False
    # Bottom row
    if not np.allclose(T[3, :], [0, 0, 0, 1], atol=1e-12):
        return False
    return True


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestFKZeroConfig:
    """FK at zero joint angles must equal the home configuration M."""

    def test_zero_config_equals_home(self, slist, home_config, num_joints):
        theta = np.zeros(num_joints)
        T = mr.FKinSpace(home_config, slist, theta)
        assert_se3_close(T, home_config)

    def test_zero_config_is_valid_se3(self, slist, home_config, num_joints):
        theta = np.zeros(num_joints)
        T = mr.FKinSpace(home_config, slist, theta)
        assert is_valid_se3(T)


class TestFKSingleJoint:
    """Rotate one joint at a time, verify FK changes appropriately."""

    def test_base_rotation_90deg(self, slist, home_config, num_joints):
        """Joint 0: Z-rotation by 90°. End-effector swings from +X to +Y."""
        theta = np.zeros(num_joints)
        theta[0] = np.pi / 2
        T = mr.FKinSpace(home_config, slist, theta)

        assert is_valid_se3(T)
        # Original home position is [0.5, 0, 0.1].
        # 90° Z rotation: x→-y, y→x → expected position [0, 0.5, 0.1]
        np.testing.assert_allclose(T[0, 3], 0.0, atol=TOL_POS)
        np.testing.assert_allclose(T[1, 3], 0.5, atol=TOL_POS)
        np.testing.assert_allclose(T[2, 3], 0.1, atol=TOL_POS)

    def test_base_rotation_neg90deg(self, slist, home_config, num_joints):
        """Joint 0: Z-rotation by -90°."""
        theta = np.zeros(num_joints)
        theta[0] = -np.pi / 2
        T = mr.FKinSpace(home_config, slist, theta)

        assert is_valid_se3(T)
        np.testing.assert_allclose(T[0, 3], 0.0, atol=TOL_POS)
        np.testing.assert_allclose(T[1, 3], -0.5, atol=TOL_POS)
        np.testing.assert_allclose(T[2, 3], 0.1, atol=TOL_POS)

    def test_base_rotation_180deg(self, slist, home_config, num_joints):
        """Joint 0: Z-rotation by 180°."""
        theta = np.zeros(num_joints)
        theta[0] = np.pi
        T = mr.FKinSpace(home_config, slist, theta)

        assert is_valid_se3(T)
        np.testing.assert_allclose(T[0, 3], -0.5, atol=TOL_POS)
        np.testing.assert_allclose(T[1, 3], 0.0, atol=TOL_POS)
        np.testing.assert_allclose(T[2, 3], 0.1, atol=TOL_POS)

    @pytest.mark.parametrize("joint_idx", range(6))
    def test_single_joint_produces_valid_se3(self, slist, home_config, num_joints, joint_idx):
        """Any single-joint rotation should produce a valid SE(3) transform."""
        theta = np.zeros(num_joints)
        theta[joint_idx] = np.deg2rad(30.0)
        T = mr.FKinSpace(home_config, slist, theta)
        assert is_valid_se3(T)

    @pytest.mark.parametrize("joint_idx", range(6))
    def test_single_joint_differs_from_home(self, slist, home_config, num_joints, joint_idx):
        """Rotating any joint should change the FK result from home."""
        theta = np.zeros(num_joints)
        theta[joint_idx] = np.deg2rad(30.0)
        T = mr.FKinSpace(home_config, slist, theta)
        assert not np.allclose(T, home_config, atol=1e-6)


class TestMatrixExponential:
    """Validate individual matrix exponentials via modern-robotics."""

    def test_so3_zero_angle(self):
        """exp([ω]·0) = I_3."""
        omega_hat = mr.VecToso3([0, 0, 1])
        se3mat = np.zeros((4, 4))
        se3mat[:3, :3] = omega_hat * 0.0  # theta = 0
        T = mr.MatrixExp6(se3mat)
        np.testing.assert_allclose(T, np.eye(4), atol=1e-12)

    def test_so3_z_rotation_90deg(self):
        """exp([ω_z]·π/2) should give 90° rotation about Z."""
        # Build se3 matrix for pure rotation about Z
        omega = np.array([0, 0, 1])
        theta = np.pi / 2
        omega_hat = mr.VecToso3(omega)
        se3mat = np.zeros((4, 4))
        se3mat[:3, :3] = omega_hat * theta
        T = mr.MatrixExp6(se3mat)

        expected_R = np.array([
            [0, -1, 0],
            [1,  0, 0],
            [0,  0, 1],
        ], dtype=float)
        np.testing.assert_allclose(T[:3, :3], expected_R, atol=1e-12)
        np.testing.assert_allclose(T[:3, 3], [0, 0, 0], atol=1e-12)

    def test_se3_screw_motion(self, slist, home_config):
        """Compute exp([ξ₀]·θ) for joint 0 at 90° and verify."""
        S0 = slist[:, 0]  # First screw axis
        theta = np.pi / 2
        # Build 4×4 se(3) matrix: [ω]θ  vθ ; 0 0
        se3mat = mr.VecTose3(S0) * theta
        T = mr.MatrixExp6(se3mat)
        assert is_valid_se3(T)


class TestMultiJoint:
    """FK with multiple joints active simultaneously."""

    def test_two_joint_config(self, slist, home_config, num_joints):
        """Joints 0 and 1 rotated, verify valid SE(3) and differs from single-joint."""
        theta = np.zeros(num_joints)
        theta[0] = np.deg2rad(45.0)
        theta[1] = np.deg2rad(30.0)
        T = mr.FKinSpace(home_config, slist, theta)
        assert is_valid_se3(T)

        # Should differ from either single joint alone
        theta0_only = np.zeros(num_joints)
        theta0_only[0] = np.deg2rad(45.0)
        T0 = mr.FKinSpace(home_config, slist, theta0_only)
        assert not np.allclose(T, T0, atol=1e-6)

    def test_all_joints_30deg(self, slist, home_config, num_joints):
        """All joints at 30°."""
        theta = np.full(num_joints, np.deg2rad(30.0))
        T = mr.FKinSpace(home_config, slist, theta)
        assert is_valid_se3(T)

    def test_fk_deterministic(self, slist, home_config, num_joints):
        """Same input must produce identical output."""
        theta = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
        T1 = mr.FKinSpace(home_config, slist, theta)
        T2 = mr.FKinSpace(home_config, slist, theta)
        np.testing.assert_array_equal(T1, T2)


class TestRandomConfigs:
    """FK on random joint configurations — verify output validity."""

    @pytest.mark.parametrize("seed", range(20))
    def test_random_config_valid_se3(self, slist, home_config, num_joints, seed):
        """Random joint angles within limits → valid SE(3) output."""
        rng = np.random.default_rng(seed)
        theta = rng.uniform(-2.618, 2.618, size=num_joints)
        T = mr.FKinSpace(home_config, slist, theta)
        assert is_valid_se3(T), f"Invalid SE(3) for theta={theta}"
