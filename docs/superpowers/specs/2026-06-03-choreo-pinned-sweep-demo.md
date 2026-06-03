# Choreographer: single pinned-sweep "reveal" demo

## Goal

Replace the two-act choreographer demo (`aim` + `pin`) with one intentional take:
the virtual tool tip is pinned at the EE home point, the EE retreats one tool
length to establish the pin, then the tool axis sweeps horizontally while the arm
visibly reconfigures around the fixed tip. Must pass the offline joint-limit gate
with the real per-joint limits (J5 = ±115° is the binding one).

## Frame conventions

- Base frame, metres. Home point **H = (0.168, 0, 0.243)** — the EE home position.
- Home orientation **R_home = [[0,0,1],[-1,0,0],[0,-1,0]]**; tool axis (local +Z)
  = `R_home[:,2] = (1,0,0) = +X` (forward, horizontal).
- Tool length **L = 0.01 m** (virtual tip is L ahead of the flange along tool +Z).

## Motion (one continuous take)

Three phases, concatenated into a single act `"reveal"`.

**A. Approach (~15 frames)** — establish the pin.
- Flange interpolates linearly from `H` (home EE) to `H − L·(+X) = (0.158, 0, 0.243)`.
- Orientation fixed at `R_home`.
- Effect: the virtual tip moves from 0.178 to **H** and is pinned at the end.

**B. Horizontal sweep (~120 frames)** — the reveal.
- Azimuth schedule `az(t) = θ·sin(2π·t)`, `t ∈ [0,1]`, `θ = 45°`
  → `0 → +45° → 0 → −45° → 0`.
- Tool axis `a(az) = Rz(az)·X = (cos az, sin az, 0)` (stays horizontal).
- Orientation `R(t) = orientation_from_axis(a(az), prev_R)`, seeded with
  `prev_R = R_home` at `t=0` so roll joins the approach smoothly and stays steady
  (existing `_align_roll` continuity).
- Flange `p(t) = pin_pose(H, R(t), L) = H − L·a(az)
  = (0.168 − 0.01·cos az, −0.01·sin az, 0.243)` — a 1 cm horizontal arc behind the
  pinned tip.

**C. Return (~15 frames)** — clean loop.
- Reverse of A: flange from `(0.158,0,0.243)` back to `H`, orientation `R_home`.
- Ends exactly at the home pose.

Junctions A→B and B→C both sit at flange `(0.158,0,0.243)` with orientation
≈ `R_home`, so the take is continuous (subject to the `continuity_rad` gate).

## Code changes (`validation/choreo.py`)

- New `build_pinned_sweep_act(target, L, half_angle_deg, n_approach, n_sweep,
  n_return)` producing the three phases above. Reuses `pin_pose`,
  `orientation_from_axis`, `_align_roll`, `pose_matrix`.
- `build_sequence()` returns a single act `{"reveal": [...]}` (was `{aim, pin}`).
- Retire `build_aim_act`; remove `orbit` / `figure_eight` if unused after that.
  Keep `spherical_sweep`? No — the new sweep is azimuth-only; remove if unused.
- `Config`: `target` default → `(0.168, 0.0, 0.243)`; add `sweep_deg: float = 45.0`;
  drop `aim_n`; keep `tool_length`, `speed_mps`, `continuity_rad`; add per-phase
  frame counts (or derive from a single `n`).

## Validation & testing

- `validate_sequence` must report `errors=0` for the `reveal` act against the real
  per-joint limits (esp. J5 ±115°). If ±45° trips a wrist/base joint, reduce
  `sweep_deg` (fall back 45 → 35 → 30) and document the chosen value.
- Update the choreo test (currently "both presets pass the gate") to the single
  `reveal` act and assert it passes the gate and stays under `continuity_rad`.
- `python choreo.py --validate-only` and `--dry-run` (with `--dump`) for manual
  inspection of the JSONL pose stream before any hardware run.

## Out of scope

- Firmware changes; joint_map calibration; speed/timing tuning beyond what the
  existing `time_ms_for` already does.

## Risks

- ±45° horizontal azimuth may push the base (J1) or a wrist joint near a limit;
  mitigated by the validation gate + the documented `sweep_deg` fallback.

## Implementation note — wrist-singularity fix (added during build)

A *perfectly level* sweep (tool axis = +X at az=0) sits at the J5=0 wrist
gimbal-lock (J4 ∥ J6). The validation gate caught this: limits were fine (~38°
margin) but IK branch-flipped J4/J6 by ~100° at every az=0 crossing
(`max_step` 1.8 rad). Fix: a constant **`tilt_deg = 25°` downward tool tilt** so
J5 stays bent throughout; the approach ramps into the tilt (pure J5 pitch) and
the return ramps out, ending level at home. Result: `max_step` 0.092 rad,
`min_margin` 0.659 rad, gate passes. The pure-horizontal sweep in the sections
above is therefore tilted 25° down in the shipped build.
