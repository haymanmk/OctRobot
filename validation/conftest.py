"""
OctroBot FK Cross-Validation — Shared pytest fixtures.

Loads robot_config.yaml and provides Slist, M matrices
in the format expected by the modern-robotics library.
"""

import os
import numpy as np
import pytest
import yaml


@pytest.fixture(scope="session")
def robot_config():
    """Load robot configuration from YAML."""
    config_path = os.path.join(os.path.dirname(__file__), "robot_config.yaml")
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


@pytest.fixture(scope="session")
def slist(robot_config):
    """
    Build Slist matrix for modern-robotics library.

    mr.FKinSpace expects Slist as a 6×n matrix where each COLUMN
    is a screw axis [omega; v].  Our YAML stores each row as
    [omega_x, omega_y, omega_z, v_x, v_y, v_z], so we transpose.
    """
    axes = np.array(robot_config["screw_axes"], dtype=float)  # (6, 6)
    return axes.T  # (6, 6) — each column is one screw axis


@pytest.fixture(scope="session")
def home_config(robot_config):
    """Home configuration M as 4×4 numpy array."""
    return np.array(robot_config["home_config"], dtype=float)


@pytest.fixture(scope="session")
def num_joints(robot_config):
    return robot_config["num_joints"]
