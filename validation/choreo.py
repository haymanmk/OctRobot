#!/usr/bin/env python3
"""choreo.py — IK reveal demo choreographer.

Generates, validates, and streams a pose sequence that reveals the inverse
kinematics: Act 1 aims a virtual tool axis at an invisible fixed point; Act 2
pins a virtual tool tip to that point while the arm reconfigures around it.

Pure functions are importable for tests; main() drives the serial stream.
"""

import os

import numpy as np
import yaml
import modern_robotics as mr

# --- RPY <-> matrix (ZYX, replicates app/src/kinematics/kinematics_math.c) ---


def rpy_to_matrix(roll_deg, pitch_deg, yaw_deg):
    """R = Rz(yaw) * Ry(pitch) * Rx(roll). Angles in degrees. Matches firmware."""
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def matrix_to_rpy(R):
    """Inverse of rpy_to_matrix. Returns (roll, pitch, yaw) in degrees, ZYX."""
    sy = np.hypot(R[0, 0], R[1, 0])
    if sy < 1e-6:  # gimbal lock: pitch = +/-90
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    else:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees([roll, pitch, yaw])


# --- robot model -----------------------------------------------------------


def load_model(config_path=None):
    """Load robot_config.yaml into a model dict (reuses the FK/IK test config)."""
    if config_path is None:
        config_path = os.path.join(os.path.dirname(__file__), "robot_config.yaml")
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    Slist = np.array(cfg["screw_axes"], dtype=float).T   # columns = screw axes
    M = np.array(cfg["home_config"], dtype=float)
    Blist = mr.Adjoint(mr.TransInv(M)) @ Slist
    return {
        "Slist": Slist,
        "M": M,
        "Blist": Blist,
        "jmin": np.array(cfg["joint_limits"]["min"], dtype=float),
        "jmax": np.array(cfg["joint_limits"]["max"], dtype=float),
        "n": int(cfg["num_joints"]),
    }


def fk(model, theta):
    return mr.FKinSpace(model["M"], model["Slist"], np.asarray(theta, dtype=float))


def ik_solve(model, T_target, seed, eomg=1e-4, ev=1e-4):
    """Body-frame IK. Returns (theta wrapped to [-pi,pi], success bool)."""
    theta, success = mr.IKinBody(model["Blist"], model["M"], T_target,
                                 np.asarray(seed, dtype=float), eomg, ev)
    theta = (theta + np.pi) % (2 * np.pi) - np.pi
    return theta, bool(success)


def pose_matrix(p, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(p, dtype=float)
    return T


# --- pose constraints ------------------------------------------------------

TOOL_LENGTH = 0.02  # metres; virtual tool tip distance along flange +Z


def orientation_from_axis(axis, prev_R=None):
    """Build a rotation whose local +Z points along `axis`.

    The roll-about-axis DOF is free; if prev_R is given, choose it to minimize
    change from the previous frame (continuity). Otherwise use a world up-vector.
    """
    z = np.asarray(axis, dtype=float)
    z = z / np.linalg.norm(z)
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(z, up)) > 0.95:    # axis nearly vertical: pick a different ref
        up = np.array([1.0, 0.0, 0.0])
    x = np.cross(up, z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])
    if prev_R is not None:
        R = _align_roll(R, prev_R, z)
    return R


def _align_roll(R, prev_R, z):
    """Rotate R about `z` so its x-axis best matches prev_R's x-axis projected
    onto the plane perpendicular to `z`."""
    x_prev = prev_R[:, 0]
    x_prev_p = x_prev - np.dot(x_prev, z) * z   # project onto plane perp to z
    n = np.linalg.norm(x_prev_p)
    if n < 1e-6:
        return R
    x_prev_p /= n
    cos_a = np.clip(np.dot(R[:, 0], x_prev_p), -1.0, 1.0)
    sin_a = np.dot(np.cross(R[:, 0], x_prev_p), z)
    angle = np.arctan2(sin_a, cos_a)
    return _rot_about(z, angle) @ R


def _rot_about(axis, angle):
    """Rodrigues rotation matrix about a unit `axis` by `angle` radians."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]],
                  [a[2], 0, -a[0]],
                  [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def pin_pose(target, R, L=TOOL_LENGTH):
    """Flange pose whose virtual tool tip lands on `target` at orientation R."""
    p = np.asarray(target, dtype=float) - R @ np.array([0.0, 0.0, L])
    return p, R.copy()


# --- parametric path primitives -------------------------------------------


def _plane_basis(normal):
    """Two orthonormal vectors spanning the plane perpendicular to `normal`."""
    n = np.asarray(normal, dtype=float)
    n = n / np.linalg.norm(n)
    ref = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(n, ref)
    u = u / np.linalg.norm(u)
    v = np.cross(n, u)
    return u, v


def orbit(center, radius, n, normal=(0, 0, 1), phase0=0.0):
    """`n` points on a circle of `radius` around `center` in the given plane."""
    center = np.asarray(center, dtype=float)
    u, v = _plane_basis(normal)
    pts = []
    for i in range(n):
        a = phase0 + 2 * np.pi * i / n
        pts.append(center + radius * (np.cos(a) * u + np.sin(a) * v))
    return pts


def figure_eight(center, size, n, normal=(0, 0, 1)):
    """`n` points on a Gerono lemniscate (figure-8) centered at `center`."""
    center = np.asarray(center, dtype=float)
    u, v = _plane_basis(normal)
    pts = []
    for i in range(n):
        t = 2 * np.pi * i / n
        pts.append(center + size * np.cos(t) * u + size * np.sin(t) * np.cos(t) * v)
    return pts


def spherical_sweep(base_dir, half_angle_deg, turns, n):
    """`n` unit directions spiralling out to `half_angle_deg` off `base_dir`
    and back, winding `turns` times around it. Used to sweep the tool axis."""
    base = np.asarray(base_dir, dtype=float)
    base = base / np.linalg.norm(base)
    u, v = _plane_basis(base)
    dirs = []
    for i in range(n):
        frac = i / (n - 1) if n > 1 else 0.0
        ang = np.radians(half_angle_deg) * np.sin(np.pi * frac)  # out and back
        az = 2 * np.pi * turns * frac
        d = (np.cos(ang) * base
             + np.sin(ang) * (np.cos(az) * u + np.sin(az) * v))
        dirs.append(d / np.linalg.norm(d))
    return dirs
