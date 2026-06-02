#!/usr/bin/env python3
"""choreo.py — IK reveal demo choreographer.

Generates, validates, and streams a pose sequence that reveals the inverse
kinematics: Act 1 aims a virtual tool axis at an invisible fixed point; Act 2
pins a virtual tool tip to that point while the arm reconfigures around it.

Pure functions are importable for tests; main() drives the serial stream.
"""

import json
import os
from dataclasses import dataclass

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


# --- act builders + presets ------------------------------------------------


@dataclass
class Config:
    target: tuple = (0.18, 0.0, 0.24)   # invisible fixed point, base frame (m)
    tool_length: float = TOOL_LENGTH
    aim_n: int = 120
    pin_n: int = 120
    speed_mps: float = 0.05             # EE speed -> per-segment TIME_MS
    continuity_rad: float = 0.15        # max joint step between frames (rad)


DEFAULT_CONFIG = Config()


def build_aim_act(target, n=120):
    """Act 1: flange orbits + traces a figure-8 while the tool axis aims at T."""
    target = np.asarray(target, dtype=float)
    center = target + np.array([0.0, 0.0, -0.08])   # stand off below the target
    frames = []
    prev = None
    half = n // 2
    for p in orbit(center, radius=0.035, n=half, normal=(1, 0, 0)):
        R = orientation_from_axis(target - p, prev)
        prev = R
        frames.append((p, R))
    for p in figure_eight(center, size=0.03, n=n - half, normal=(1, 0, 0)):
        R = orientation_from_axis(target - p, prev)
        prev = R
        frames.append((p, R))
    return frames


def build_pin_act(target, L=TOOL_LENGTH, n=120):
    """Act 2: tool tip pinned at T while the tool axis sweeps a cone."""
    target = np.asarray(target, dtype=float)
    base_dir = np.array([0.0, 0.0, 1.0])   # nominal tool axis (points up at target)
    frames = []
    prev = None
    for d in spherical_sweep(base_dir, half_angle_deg=22, turns=1.5, n=n):
        R = orientation_from_axis(d, prev)
        prev = R
        p, _ = pin_pose(target, R, L)
        frames.append((p, R))
    return frames


def build_sequence(config=None):
    """Return {'aim': [...frames...], 'pin': [...frames...]} for the two acts."""
    if config is None:
        config = Config()
    return {
        "aim": build_aim_act(config.target, n=config.aim_n),
        "pin": build_pin_act(config.target, L=config.tool_length, n=config.pin_n),
    }


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
        "min_margin": (None if not np.isfinite(min_margin) else float(min_margin)),
        "n": len(frames),
    }


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
    t_pos = (np.linalg.norm(pc - pp) / speed_mps * 1000.0) if speed_mps > 0 else 0.0
    t_rot = np.degrees(_rot_angle(Rp, Rc)) / rot_speed_dps * 1000.0
    return int(np.round(np.clip(max(t_pos, t_rot), min_ms, max_ms)))


# --- reproducible take dump ------------------------------------------------


def dump_jsonl(frames, path, act):
    """Write one JSON object per frame: {act, xyz[3], rpy[3]} (rpy in degrees, ZYX)."""
    with open(path, "w") as f:
        for p, R in frames:
            rpy = matrix_to_rpy(R)
            f.write(json.dumps({
                "act": act,
                "xyz": [float(v) for v in p],
                "rpy": [float(v) for v in rpy],
            }) + "\n")


# --- serial streaming + CLI ------------------------------------------------


def stream_frames(ser, frames, speed_mps, act_label, dry_run=False):
    """Send each frame as a $movec line, pacing TIME_MS by motion. Returns
    the number of frames sent. `ser` may be None when dry_run is True."""
    import time
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
    if args.speed is not None:
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
        base = args.dump[:-6] if args.dump.endswith(".jsonl") else args.dump
        for act_name, frames in seq.items():
            path = f"{base}.{act_name}.jsonl"
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
