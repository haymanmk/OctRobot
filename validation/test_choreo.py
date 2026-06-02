import json

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
    np.testing.assert_allclose(np.mean(pts, axis=0), center, atol=1e-10)


def test_spherical_sweep_unit_dirs_near_base():
    base = np.array([-1.0, 0.0, 0.0])
    dirs = choreo.spherical_sweep(base, half_angle_deg=30, turns=2.0, n=50)
    assert len(dirs) == 50
    for d in dirs:
        assert abs(np.linalg.norm(d) - 1.0) < 1e-9          # unit vectors
        assert np.degrees(np.arccos(np.clip(np.dot(d, base / np.linalg.norm(base)), -1, 1))) <= 30 + 1e-6
    np.testing.assert_allclose(dirs[0], base / np.linalg.norm(base), atol=1e-12)
    np.testing.assert_allclose(dirs[-1], base / np.linalg.norm(base), atol=1e-12)


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
    assert len(seq["aim"]) == choreo.DEFAULT_CONFIG.aim_n
    assert len(seq["pin"]) == choreo.DEFAULT_CONFIG.pin_n


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


def test_default_presets_pass_validation():
    m = choreo.load_model()
    seq = choreo.build_sequence(choreo.DEFAULT_CONFIG)
    for act_name, frames in seq.items():
        report = choreo.validate_sequence(
            m, frames, continuity_rad=choreo.DEFAULT_CONFIG.continuity_rad)
        assert report["errors"] == [], (
            f"act '{act_name}' has bad frames: {report['errors'][:5]} "
            f"(max_step={report['max_step']:.3f}, min_margin={report['min_margin']})")


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
    # an intermediate distance exercises the unclamped linear regime:
    # 0.02 m / 0.05 m/s = 0.4 s = 400 ms (rotation is zero here)
    mid = (np.array([0.02, 0.0, 0.0]), np.eye(3))
    assert choreo.time_ms_for(home, mid, speed_mps=0.05) == 400


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
    np.testing.assert_allclose(rec["rpy"], choreo.matrix_to_rpy(frames[0][1]), atol=1e-9)
