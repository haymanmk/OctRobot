# IK Reveal Demo ("The Invisible Point") Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `validation/choreo.py`, a host-side tool that generates an IK-revealing pose choreography (Act 1 "aim at an invisible point", Act 2 "pin a virtual tool tip to that point"), validates every pose offline against the robot model, and streams it to the firmware as `$movec` console commands.

**Architecture:** Pure-Python pipeline reusing `validation/robot_config.yaml` + `modern_robotics`. Four stages — parametric path generation → aim/pin pose solve → offline FK/IK validation gate → serial streaming. No firmware changes; the firmware solves each full 6-DOF `$movec` pose it is handed. The convention-critical RPY↔matrix conversion replicates the firmware's own `Rz·Ry·Rx` formula and is locked by a test against the known home pose.

**Tech Stack:** Python 3, numpy, modern_robotics, pyyaml, pyserial, pytest.

---

## File Structure

- **Create** `validation/choreo.py` — all logic: model loader, RPY conversion, pose primitives (aim/pin), path primitives (orbit/figure-eight/spherical-sweep), act builders + presets, validation gate, serial streamer, `argparse` CLI. Importable pure functions so pytest can exercise them without hardware.
- **Create** `validation/test_choreo.py` — unit tests for the pure functions + a soundness test asserting both shipped presets pass the validation gate (the "prove the choreography in CI" requirement).
- **Modify** `validation/requirements.txt` — add `pyserial`.
- **Create** `validation/README-choreo.md` — short operator usage + pre-flight checklist.

All paths below are relative to repo root `/home/hayman/Workspace/octrobot`. Run pytest from `validation/` (matches existing `pytest.ini`).

Conventions used across tasks (defined once, referenced everywhere):
- A **frame** is a tuple `(p, R)` where `p` is a length-3 numpy array (metres, base frame) and `R` is a 3×3 numpy rotation matrix.
- A **model** is the dict returned by `load_model()` with keys `Slist (6,6)`, `M (4,4)`, `Blist (6,6)`, `jmin (6,)`, `jmax (6,)`, `n (int)`.
- Tool axis = flange local **+Z** = third column of `R`. Virtual tool length `L = 0.02 m`.

---

## Task 1: RPY ↔ rotation-matrix conversion (convention lock)

**Files:**
- Create: `validation/choreo.py`
- Test: `validation/test_choreo.py`

- [ ] **Step 1: Write the failing test**

Create `validation/test_choreo.py`:

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd validation && python -m pytest test_choreo.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'choreo'` (or `AttributeError` once the file exists).

- [ ] **Step 3: Write minimal implementation**

Create `validation/choreo.py`:

```python
#!/usr/bin/env python3
"""choreo.py — IK reveal demo choreographer.

Generates, validates, and streams a pose sequence that reveals the inverse
kinematics: Act 1 aims a virtual tool axis at an invisible fixed point; Act 2
pins a virtual tool tip to that point while the arm reconfigures around it.

Pure functions are importable for tests; main() drives the serial stream.
"""

import numpy as np

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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd validation && python -m pytest test_choreo.py -v`
Expected: PASS (2 passed).

- [ ] **Step 5: Commit**

```bash
git add validation/choreo.py validation/test_choreo.py
git commit -m "feat(choreo): RPY<->matrix conversion locked to firmware ZYX convention"
```

---

## Task 2: Robot model loader + FK/IK wrappers

**Files:**
- Modify: `validation/choreo.py`
- Test: `validation/test_choreo.py`

- [ ] **Step 1: Write the failing test**

Append to `validation/test_choreo.py`:

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd validation && python -m pytest test_choreo.py -k "model or fk or ik" -v`
Expected: FAIL — `AttributeError: module 'choreo' has no attribute 'load_model'`.

- [ ] **Step 3: Write minimal implementation**

Add to `validation/choreo.py` (imports at top, functions below the RPY block):

```python
import os
import yaml
import modern_robotics as mr

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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd validation && python -m pytest test_choreo.py -k "model or fk or ik" -v`
Expected: PASS (3 passed).

- [ ] **Step 5: Commit**

```bash
git add validation/choreo.py validation/test_choreo.py
git commit -m "feat(choreo): robot model loader + FK/IK wrappers over modern_robotics"
```

---

## Task 3: Orientation-from-axis + aim/pin pose helpers

**Files:**
- Modify: `validation/choreo.py`
- Test: `validation/test_choreo.py`

- [ ] **Step 1: Write the failing test**

Append to `validation/test_choreo.py`:

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd validation && python -m pytest test_choreo.py -k "orientation or pin_pose" -v`
Expected: FAIL — `AttributeError: module 'choreo' has no attribute 'orientation_from_axis'`.

- [ ] **Step 3: Write minimal implementation**

Add to `validation/choreo.py`:

```python
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
    """Rotate R about its z-axis so its x-axis best matches prev_R's x-axis."""
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
    return p, R
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd validation && python -m pytest test_choreo.py -k "orientation or pin_pose" -v`
Expected: PASS (2 passed).

- [ ] **Step 5: Commit**

```bash
git add validation/choreo.py validation/test_choreo.py
git commit -m "feat(choreo): aim orientation-from-axis + pinned-tip pose helpers"
```

---

## Task 4: Parametric path primitives

**Files:**
- Modify: `validation/choreo.py`
- Test: `validation/test_choreo.py`

- [ ] **Step 1: Write the failing test**

Append to `validation/test_choreo.py`:

```python
def test_orbit_lies_on_circle():
    center = np.array([0.1, 0.0, 0.2])
    pts = choreo.orbit(center, radius=0.05, n=24, normal=(0, 0, 1))
    assert len(pts) == 24
    for q in pts:
        # distance from center is the radius, and z stays in the plane
        assert abs(np.linalg.norm(q - center) - 0.05) < 1e-9
        assert abs((q - center)[2]) < 1e-9   # normal is +z -> constant height


def test_figure_eight_count_and_centered():
    center = np.array([0.1, 0.0, 0.2])
    pts = choreo.figure_eight(center, size=0.04, n=40, normal=(0, 1, 0))
    assert len(pts) == 40
    # mean is approximately the center (lemniscate is symmetric)
    np.testing.assert_allclose(np.mean(pts, axis=0), center, atol=1e-2)


def test_spherical_sweep_unit_dirs_near_base():
    base = np.array([-1.0, 0.0, 0.0])
    dirs = choreo.spherical_sweep(base, half_angle_deg=30, turns=2.0, n=50)
    assert len(dirs) == 50
    for d in dirs:
        assert abs(np.linalg.norm(d) - 1.0) < 1e-9          # unit vectors
        assert np.degrees(np.arccos(np.clip(np.dot(d, base / np.linalg.norm(base)), -1, 1))) <= 30 + 1e-6
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd validation && python -m pytest test_choreo.py -k "orbit or figure_eight or spherical" -v`
Expected: FAIL — `AttributeError: module 'choreo' has no attribute 'orbit'`.

- [ ] **Step 3: Write minimal implementation**

Add to `validation/choreo.py`:

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd validation && python -m pytest test_choreo.py -k "orbit or figure_eight or spherical" -v`
Expected: PASS (3 passed).

- [ ] **Step 5: Commit**

```bash
git add validation/choreo.py validation/test_choreo.py
git commit -m "feat(choreo): parametric path primitives (orbit, figure-eight, spherical sweep)"
```

---

## Task 5: Act builders + shipped presets

**Files:**
- Modify: `validation/choreo.py`
- Test: `validation/test_choreo.py`

- [ ] **Step 1: Write the failing test**

Append to `validation/test_choreo.py`:

```python
def test_build_aim_act_frames_point_at_target():
    target = np.array([0.20, 0.0, 0.18])
    frames = choreo.build_aim_act(target, n=60)
    assert len(frames) == 60
    for p, R in frames:
        z = R[:, 2]
        want = (target - p) / np.linalg.norm(target - p)
        np.testing.assert_allclose(z, want, atol=1e-6)


def test_build_pin_act_tip_stays_on_target():
    target = np.array([0.20, 0.0, 0.18])
    frames = choreo.build_pin_act(target, L=0.02, n=60)
    assert len(frames) == 60
    for p, R in frames:
        tip = p + R @ np.array([0.0, 0.0, 0.02])
        np.testing.assert_allclose(tip, target, atol=1e-9)


def test_presets_exist_and_return_frames():
    seq = choreo.build_sequence(choreo.DEFAULT_CONFIG)
    assert "aim" in seq and "pin" in seq
    assert len(seq["aim"]) > 0 and len(seq["pin"]) > 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd validation && python -m pytest test_choreo.py -k "build_aim or build_pin or presets" -v`
Expected: FAIL — `AttributeError: module 'choreo' has no attribute 'build_aim_act'`.

- [ ] **Step 3: Write minimal implementation**

Add to `validation/choreo.py` (the dataclass import goes with the other imports at top):

```python
from dataclasses import dataclass

# --- act builders + presets ------------------------------------------------


@dataclass
class Config:
    target: tuple = (0.20, 0.0, 0.18)   # invisible fixed point, base frame (m)
    tool_length: float = TOOL_LENGTH
    aim_n: int = 120
    pin_n: int = 120
    speed_mps: float = 0.05             # EE speed -> per-segment TIME_MS
    continuity_rad: float = 0.15        # max joint step between frames (rad)


DEFAULT_CONFIG = Config()


def build_aim_act(target, n=120):
    """Act 1: flange orbits + traces a figure-8 while the tool axis aims at T."""
    target = np.asarray(target, dtype=float)
    center = target + np.array([-0.10, 0.0, 0.02])   # stand off from the target
    frames = []
    prev = None
    half = max(2, n // 2)
    for p in orbit(center, radius=0.06, n=half, normal=(0, 0, 1)):
        R = orientation_from_axis(target - p, prev)
        prev = R
        frames.append((p, R))
    for p in figure_eight(center, size=0.05, n=n - half, normal=(0, 1, 0)):
        R = orientation_from_axis(target - p, prev)
        prev = R
        frames.append((p, R))
    return frames


def build_pin_act(target, L=TOOL_LENGTH, n=120):
    """Act 2: tool tip pinned at T while the tool axis sweeps a cone."""
    target = np.asarray(target, dtype=float)
    base_dir = np.array([-1.0, 0.0, 0.0])   # nominal tool axis (points toward base)
    frames = []
    prev = None
    for d in spherical_sweep(base_dir, half_angle_deg=35, turns=2.0, n=n):
        R = orientation_from_axis(d, prev)
        prev = R
        p, _ = pin_pose(target, R, L)
        frames.append((p, R))
    return frames


def build_sequence(config=DEFAULT_CONFIG):
    """Return {'aim': [...frames...], 'pin': [...frames...]} for the two acts."""
    return {
        "aim": build_aim_act(config.target, n=config.aim_n),
        "pin": build_pin_act(config.target, L=config.tool_length, n=config.pin_n),
    }
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd validation && python -m pytest test_choreo.py -k "build_aim or build_pin or presets" -v`
Expected: PASS (3 passed).

- [ ] **Step 5: Commit**

```bash
git add validation/choreo.py validation/test_choreo.py
git commit -m "feat(choreo): aim/pin act builders + default two-act preset"
```

---

## Task 6: Offline validation gate

**Files:**
- Modify: `validation/choreo.py`
- Test: `validation/test_choreo.py`

- [ ] **Step 1: Write the failing test**

Append to `validation/test_choreo.py`:

```python
def test_validate_flags_unreachable():
    m = choreo.load_model()
    # A target 5 m away is far outside the workspace -> IK cannot converge.
    bad = choreo.build_pin_act(np.array([5.0, 0.0, 0.0]), L=0.02, n=10)
    report = choreo.validate_sequence(m, bad)
    assert len(report["errors"]) > 0


def test_validate_flags_discontinuity():
    m = choreo.load_model()
    # Two reachable poses that are far apart in joint space: the second forces
    # a ~0.6 rad jump on joint 0, which must trip the continuity check.
    Ta = choreo.fk(m, np.zeros(6))
    Tb = choreo.fk(m, np.array([0.6, 0.0, 0.0, 0.0, 0.0, 0.0]))
    frames = [(Ta[:3, 3].copy(), Ta[:3, :3].copy()),
              (Tb[:3, 3].copy(), Tb[:3, :3].copy())]
    report = choreo.validate_sequence(m, frames, continuity_rad=0.1)
    assert any("discontinuity" in e[1] for e in report["errors"])
    assert report["max_step"] > 0.1


def test_validate_passes_clean_short_sequence():
    m = choreo.load_model()
    home = choreo.fk(m, np.zeros(6))
    frames = [(home[:3, 3].copy(), home[:3, :3].copy()) for _ in range(5)]
    report = choreo.validate_sequence(m, frames)
    assert report["errors"] == []
    assert report["n"] == 5
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd validation && python -m pytest test_choreo.py -k "validate" -v`
Expected: FAIL — `AttributeError: module 'choreo' has no attribute 'validate_sequence'`.

- [ ] **Step 3: Write minimal implementation**

Add to `validation/choreo.py`:

```python
# --- validation gate -------------------------------------------------------


def _ang_diff(a, b):
    """Element-wise shortest angular distance |a-b| wrapped to [0, pi]."""
    d = (a - b + np.pi) % (2 * np.pi) - np.pi
    return np.abs(d)


def validate_sequence(model, frames, continuity_rad=0.15):
    """IK-check every frame offline. Returns a report dict:

        errors:    list of (frame_index, reason) — empty means the take is safe
        max_step:  largest per-frame joint move (rad), shortest-arc
        min_margin: smallest distance to a joint limit (rad) over the sequence
        n:         number of frames
    """
    seed = np.zeros(model["n"])
    prev_theta = None
    errors = []
    max_step = 0.0
    min_margin = np.inf
    for idx, (p, R) in enumerate(frames):
        T = pose_matrix(p, R)
        theta, ok = ik_solve(model, T, seed)
        if not ok:
            errors.append((idx, "no_convergence"))
            continue
        if np.any(theta < model["jmin"]) or np.any(theta > model["jmax"]):
            errors.append((idx, "joint_limit"))
        margin = min((theta - model["jmin"]).min(), (model["jmax"] - theta).min())
        min_margin = min(min_margin, margin)
        if prev_theta is not None:
            step = float(np.max(_ang_diff(theta, prev_theta)))
            max_step = max(max_step, step)
            if step > continuity_rad:
                errors.append((idx, f"discontinuity {step:.3f} rad"))
        prev_theta = theta
        seed = theta
    return {
        "errors": errors,
        "max_step": max_step,
        "min_margin": (None if min_margin is np.inf else float(min_margin)),
        "n": len(frames),
    }
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd validation && python -m pytest test_choreo.py -k "validate" -v`
Expected: PASS (3 passed).

- [ ] **Step 5: Commit**

```bash
git add validation/choreo.py validation/test_choreo.py
git commit -m "feat(choreo): offline IK validation gate (reachability + continuity)"
```

---

## Task 7: Preset soundness test (CI proof of the choreography)

This is the spec's "prove the choreography in CI without a robot" requirement. It may reveal that the default `target`/amplitudes produce unreachable or jerky frames — if so, tune the constants in `Config` / the act builders until both presets pass, then keep this test as the regression guard.

**Files:**
- Test: `validation/test_choreo.py`
- Possibly modify: `validation/choreo.py` (tune `Config.target`, orbit/sweep amplitudes only if the gate fails)

- [ ] **Step 1: Write the failing test**

Append to `validation/test_choreo.py`:

```python
def test_default_presets_pass_validation():
    m = choreo.load_model()
    seq = choreo.build_sequence(choreo.DEFAULT_CONFIG)
    for act_name, frames in seq.items():
        report = choreo.validate_sequence(
            m, frames, continuity_rad=choreo.DEFAULT_CONFIG.continuity_rad)
        assert report["errors"] == [], (
            f"act '{act_name}' has bad frames: {report['errors'][:5]} "
            f"(max_step={report['max_step']:.3f}, min_margin={report['min_margin']})")
```

- [ ] **Step 2: Run test to verify current state**

Run: `cd validation && python -m pytest test_choreo.py -k "default_presets" -v`
Expected: Either PASS (presets already sound) or FAIL listing offending frames.

- [ ] **Step 3: If it fails, tune until it passes**

Adjust ONLY these knobs and re-run, smallest change first:
- `Config.target` — move closer to the validated workspace. The home tool tip is near `[0.188, 0, 0.243]` (home flange `[0.168,0,0.243]` + 2 cm along +X), so a target around `(0.20, 0.0, 0.18)` is a sane starting neighborhood.
- `build_aim_act`: reduce `radius` (0.06 → 0.04) / `size` (0.05 → 0.03), or move `center` stand-off.
- `build_pin_act`: reduce `half_angle_deg` (35 → 25) or `turns`.
- If a single isolated frame trips continuity, raise `Config.continuity_rad` (0.15 → 0.20) — but prefer geometry fixes first.

Re-run after each change:
Run: `cd validation && python -m pytest test_choreo.py -k "default_presets" -v`

- [ ] **Step 4: Run the full test file to confirm nothing regressed**

Run: `cd validation && python -m pytest test_choreo.py -v`
Expected: PASS (all tests).

- [ ] **Step 5: Commit**

```bash
git add validation/choreo.py validation/test_choreo.py
git commit -m "test(choreo): both presets pass the offline validation gate"
```

---

## Task 8: `$movec` line formatting + pacing

**Files:**
- Modify: `validation/choreo.py`
- Test: `validation/test_choreo.py`

- [ ] **Step 1: Write the failing test**

Append to `validation/test_choreo.py`:

```python
def test_format_movec_line():
    p = np.array([0.168, 0.0, 0.243])
    rpy = (-90.0, 0.0, -90.0)
    line = choreo.format_movec(p, rpy, 1000)
    assert line == "$movec 0.1680 0.0000 0.2430 -90.00 0.00 -90.00 1000"


def test_time_ms_scales_with_motion():
    home = (np.array([0.0, 0.0, 0.0]), np.eye(3))
    far = (np.array([0.10, 0.0, 0.0]), np.eye(3))
    t_small = choreo.time_ms_for(home, home, speed_mps=0.05)
    t_big = choreo.time_ms_for(home, far, speed_mps=0.05)
    assert t_big > t_small
    assert 40 <= t_small <= 2000 and 40 <= t_big <= 2000
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd validation && python -m pytest test_choreo.py -k "format_movec or time_ms" -v`
Expected: FAIL — `AttributeError: module 'choreo' has no attribute 'format_movec'`.

- [ ] **Step 3: Write minimal implementation**

Add to `validation/choreo.py`:

```python
# --- output formatting + pacing -------------------------------------------


def format_movec(p, rpy, time_ms):
    """Render one '$movec X Y Z ROLL PITCH YAW TIME_MS' console line."""
    return ("$movec "
            f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} "
            f"{rpy[0]:.2f} {rpy[1]:.2f} {rpy[2]:.2f} {int(time_ms)}")


def _rot_angle(Ra, Rb):
    """Geodesic angle (rad) between two rotation matrices."""
    Rrel = Ra.T @ Rb
    return np.arccos(np.clip((np.trace(Rrel) - 1.0) / 2.0, -1.0, 1.0))


def time_ms_for(prev_frame, cur_frame, speed_mps, min_ms=40, max_ms=2000,
                rot_speed_dps=60.0):
    """Per-segment duration from translation AND rotation, whichever is slower.

    Keeps the pinned-tip act (tiny translation, large rotation) watchable."""
    if prev_frame is None:
        return 1000
    pp, Rp = prev_frame
    pc, Rc = cur_frame
    t_pos = np.linalg.norm(pc - pp) / speed_mps * 1000.0
    t_rot = np.degrees(_rot_angle(Rp, Rc)) / rot_speed_dps * 1000.0
    return int(np.clip(max(t_pos, t_rot), min_ms, max_ms))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd validation && python -m pytest test_choreo.py -k "format_movec or time_ms" -v`
Expected: PASS (2 passed).

- [ ] **Step 5: Commit**

```bash
git add validation/choreo.py validation/test_choreo.py
git commit -m "feat(choreo): \$movec line formatting + translation/rotation pacing"
```

---

## Task 9: JSONL pose dump (reproducible takes)

**Files:**
- Modify: `validation/choreo.py`
- Test: `validation/test_choreo.py`

- [ ] **Step 1: Write the failing test**

Append to `validation/test_choreo.py`:

```python
import json


def test_frames_to_jsonl_roundtrip(tmp_path):
    target = np.array([0.20, 0.0, 0.18])
    frames = choreo.build_pin_act(target, L=0.02, n=5)
    out = tmp_path / "take.jsonl"
    choreo.dump_jsonl(frames, str(out), act="pin")
    lines = out.read_text().strip().splitlines()
    assert len(lines) == 5
    rec = json.loads(lines[0])
    assert rec["act"] == "pin"
    assert len(rec["xyz"]) == 3 and len(rec["rpy"]) == 3
    # xyz matches the frame's flange position
    np.testing.assert_allclose(rec["xyz"], frames[0][0], atol=1e-9)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd validation && python -m pytest test_choreo.py -k "jsonl" -v`
Expected: FAIL — `AttributeError: module 'choreo' has no attribute 'dump_jsonl'`.

- [ ] **Step 3: Write minimal implementation**

Add to `validation/choreo.py` (`import json` with the other top imports):

```python
import json

# --- reproducible take dump ------------------------------------------------


def dump_jsonl(frames, path, act):
    """Write one JSON object per frame: {act, xyz[3], rpy[3]} (rpy in degrees)."""
    with open(path, "w") as f:
        for p, R in frames:
            rpy = matrix_to_rpy(R)
            f.write(json.dumps({
                "act": act,
                "xyz": [float(v) for v in p],
                "rpy": [float(v) for v in rpy],
            }) + "\n")
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd validation && python -m pytest test_choreo.py -k "jsonl" -v`
Expected: PASS (1 passed).

- [ ] **Step 5: Commit**

```bash
git add validation/choreo.py validation/test_choreo.py
git commit -m "feat(choreo): JSONL pose dump for reproducible takes"
```

---

## Task 10: CLI + serial streamer

The serial I/O itself is not unit-tested (no hardware in CI); the testable seams (formatting, pacing, validation, dump) are already covered. This task wires them into `main()` with `--dry-run` and `--validate-only` so the whole pipeline runs without a device.

**Files:**
- Modify: `validation/choreo.py`
- Modify: `validation/requirements.txt`
- Test: manual `--validate-only` / `--dry-run` runs (commands below)

- [ ] **Step 1: Add pyserial to requirements**

Edit `validation/requirements.txt`, add after the `pyyaml` line:

```
pyserial>=3.5
```

- [ ] **Step 2: Implement the streamer + CLI**

Add to the bottom of `validation/choreo.py`:

```python
# --- serial streaming + CLI ------------------------------------------------


def stream_frames(ser, frames, speed_mps, act_label, dry_run=False):
    """Send each frame as a $movec line, pacing TIME_MS by motion. Returns
    the number of frames sent. `ser` may be None when dry_run is True."""
    prev = None
    sent = 0
    for p, R in frames:
        time_ms = time_ms_for(prev, (p, R), speed_mps)
        line = format_movec(p, matrix_to_rpy(R), time_ms)
        if dry_run:
            print(line)
        else:
            ser.write((line + "\n").encode())
            # pace the host so commands don't outrun the onboard motion
            import time
            time.sleep(time_ms / 1000.0)
        prev = (p, R)
        sent += 1
    print(f"[{act_label}] sent {sent} frames")
    return sent


def print_preflight(config):
    print("=== PRE-FLIGHT CHECKLIST ===")
    print("  [auto] '$power 1' will be sent before motion")
    print("  [you ] joint-map calibration flashed to the arm?")
    print("  [you ] firmware built UNICORE (CONFIG_SMP=n)?")
    print(f"  [gate] target {config.target} validated below")
    print("============================")


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description="IK reveal demo choreographer")
    ap.add_argument("--port", help="serial port, e.g. /dev/ttyUSB0")
    ap.add_argument("--dry-run", action="store_true",
                    help="print $movec lines instead of sending")
    ap.add_argument("--validate-only", action="store_true",
                    help="run the offline gate and report; no output stream")
    ap.add_argument("--target", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="override the invisible target point (m)")
    ap.add_argument("--speed", type=float, help="EE speed m/s (pacing)")
    ap.add_argument("--dump", help="write resolved poses to this .jsonl path")
    args = ap.parse_args(argv)

    config = Config()
    if args.target:
        config.target = tuple(args.target)
    if args.speed:
        config.speed_mps = args.speed

    model = load_model()
    seq = build_sequence(config)

    # Stage 3: validation gate — fail before any serial output.
    failed = False
    for act_name, frames in seq.items():
        report = validate_sequence(model, frames, config.continuity_rad)
        print(f"[{act_name}] frames={report['n']} max_step={report['max_step']:.3f} "
              f"rad min_margin={report['min_margin']} errors={len(report['errors'])}")
        if report["errors"]:
            failed = True
            for idx, reason in report["errors"][:10]:
                print(f"    frame {idx}: {reason}")
    if failed:
        print("VALIDATION FAILED — fix geometry/target before filming.")
        return 1

    if args.dump:
        for act_name, frames in seq.items():
            path = args.dump.replace(".jsonl", f".{act_name}.jsonl")
            dump_jsonl(frames, path, act_name)
            print(f"wrote {path}")

    if args.validate_only:
        print("Validation OK.")
        return 0

    print_preflight(config)

    if args.dry_run:
        ser = None
    else:
        if not args.port:
            print("ERROR: --port required unless --dry-run/--validate-only")
            return 2
        import serial
        ser = serial.Serial(args.port, 115200, timeout=0.1)
        ser.write(b"$power 1\n")     # precondition: servos must be powered
        import time
        time.sleep(0.5)

    stream_frames(ser, seq["aim"], config.speed_mps, "aim", args.dry_run)
    input(">>> Act 1 (aim) done. Reposition camera, press Enter for Act 2 (pin)...")
    stream_frames(ser, seq["pin"], config.speed_mps, "pin", args.dry_run)

    if ser is not None:
        ser.close()
    print("Done.")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
```

- [ ] **Step 3: Verify the offline pipeline runs end-to-end**

Run: `cd validation && python choreo.py --validate-only`
Expected: prints per-act `frames=… max_step=… errors=0` for both acts, then `Validation OK.`, exit 0.

Run: `cd validation && python choreo.py --dry-run 2>/dev/null | head -3`
Expected: first three `$movec …` lines print (the `input()` pause is reached after Act 1; piping to `head` is fine for a spot check).

- [ ] **Step 4: Confirm the full test suite still passes**

Run: `cd validation && python -m pytest test_choreo.py -v`
Expected: PASS (all tests from Tasks 1–9).

- [ ] **Step 5: Commit**

```bash
git add validation/choreo.py validation/requirements.txt
git commit -m "feat(choreo): CLI + serial streamer with validation gate and dry-run"
```

---

## Task 11: Operator README

**Files:**
- Create: `validation/README-choreo.md`

- [ ] **Step 1: Write the README**

Create `validation/README-choreo.md`:

```markdown
# IK Reveal Demo — choreo.py

Generates and streams the two-act "invisible point" demo that reveals the
inverse kinematics. No firmware changes; it streams `$movec` console commands.

## Install
    cd validation && pip install -r requirements.txt

## Prove the choreography offline (no robot)
    python choreo.py --validate-only
    python choreo.py --dry-run | less          # inspect the $movec lines

## Run it on hardware
    python choreo.py --port /dev/ttyUSB0

Act 1 (aim) runs, then it pauses for you to reposition the camera; press Enter
to run Act 2 (pinned tip).

## Pre-flight (the tool prints this; items marked [you] are manual)
1. `$power 1` — sent automatically (Feetech servos move on GOAL_SPEED; without
   power the computed speed is 0 and the arm won't move).
2. Joint-map calibration flashed — otherwise the arm points where the servos
   think, not where the math says.
3. Firmware built UNICORE (`CONFIG_SMP=n`) — SMP boot-loops on hardware.
4. Target inside the validated workspace — enforced by the validation gate.

## Tuning the look
Edit `Config` (target, speeds) or the amplitudes in `build_aim_act` /
`build_pin_act`. Re-run `--validate-only` after every change; the shipped
presets are regression-guarded by `test_choreo.py::test_default_presets_pass_validation`.

## Reproducible takes
    python choreo.py --validate-only --dump take.jsonl
writes `take.aim.jsonl` / `take.pin.jsonl` with the exact resolved poses.

## The target point is never physical
It is an invisible point in space; annotate it in the post edit (a drawn-in
circle). No laser or prop is mounted.
```

- [ ] **Step 2: Commit**

```bash
git add validation/README-choreo.md
git commit -m "docs(choreo): operator README + pre-flight checklist"
```

---

## Final verification

- [ ] Full suite green: `cd validation && python -m pytest -v` (existing FK/IK tests + all `test_choreo.py`).
- [ ] Offline pipeline: `cd validation && python choreo.py --validate-only` → exit 0, both acts `errors=0`.
- [ ] Dry run emits well-formed `$movec` lines.
- [ ] Hardware run is deferred to the actual shoot (requires the pre-flight items; not part of this plan's automated verification).
```
