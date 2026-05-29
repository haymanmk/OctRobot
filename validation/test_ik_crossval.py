"""
OctroBot IK Cross-Validation Tests

Validates the body-frame IK reference (modern_robotics.IKinBody) and the body
screw-list derivation Blist = Adjoint(TransInv(M)) @ Slist that the C solver
uses. Mirrors the FK cross-validation harness.

Run:  cd validation && python -m pytest test_ik_crossval.py -v
"""

import numpy as np
import modern_robotics as mr
import pytest

EOMG = 1e-3
EV = 1e-4


@pytest.fixture(scope="session")
def blist(slist, home_config):
    """Body screw list: each column B_i = Adjoint(TransInv(M)) @ S_i."""
    return mr.Adjoint(mr.TransInv(home_config)) @ slist


class TestBlistDerivation:
    """Body FK with Blist must equal space FK with Slist for any config."""

    @pytest.mark.parametrize("seed", range(10))
    def test_body_fk_matches_space_fk(self, slist, blist, home_config,
                                      num_joints, seed):
        rng = np.random.default_rng(seed)
        theta = rng.uniform(-np.pi / 2, np.pi / 2, size=num_joints)
        T_space = mr.FKinSpace(home_config, slist, theta)
        T_body = mr.FKinBody(home_config, blist, theta)
        np.testing.assert_allclose(T_body, T_space, atol=1e-9)


class TestIKinBodyRoundTrip:
    """IKinBody must recover a pose-equivalent solution from a nearby seed."""

    @pytest.mark.parametrize("seed", range(15))
    def test_round_trip(self, slist, blist, home_config, num_joints, seed):
        rng = np.random.default_rng(1000 + seed)
        theta_true = rng.uniform(-np.pi / 3, np.pi / 3, size=num_joints)
        T_target = mr.FKinSpace(home_config, slist, theta_true)

        guess = theta_true + rng.uniform(-0.1, 0.1, size=num_joints)
        theta_sol, ok = mr.IKinBody(blist, home_config, T_target,
                                    guess.copy(), EOMG, EV)
        assert ok, f"IKinBody failed to converge (seed {seed})"

        T_check = mr.FKinSpace(home_config, slist, theta_sol)
        np.testing.assert_allclose(T_check[:3, 3], T_target[:3, 3], atol=1e-3)
        np.testing.assert_allclose(T_check[:3, :3], T_target[:3, :3], atol=1e-3)

    def test_home_target_solves_to_zero(self, slist, blist, home_config,
                                        num_joints):
        theta_sol, ok = mr.IKinBody(blist, home_config, home_config,
                                    np.zeros(num_joints), EOMG, EV)
        assert ok
        np.testing.assert_allclose(theta_sol, np.zeros(num_joints), atol=1e-3)
