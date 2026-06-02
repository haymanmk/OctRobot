# IK Reveal Demo — "The Invisible Point"

**Date:** 2026-06-02
**Status:** Design approved, pending implementation plan
**Author:** brainstormed with Claude

## Purpose

Film a demo that reveals the inverse-kinematics feature in a way that is
*legibly IK* — not just servos turning, but the controller visibly *solving*
for joint angles to satisfy a spatial constraint. A virtual tool tip interacts
with a single invisible fixed point in space while the rest of the arm
reconfigures.

The footage is the deliverable. The software exists to produce a repeatable,
de-risked take.

## Narrative — two acts, one target

A single fixed target point `T` in the workspace, never marked physically;
the target is drawn in as an on-screen annotation in post.

- **Act 1 — Aiming.** The virtual tool axis always points *at* `T` while the
  end-effector moves through space at varying positions and distances — like a
  spotlight tracking a subject. Poses converge toward an unmarked point.
- **Act 2 — Pinned tip (remote center of motion).** The virtual tool tip stays
  glued *to* `T` while the whole arm sweeps and contorts around it. The flange
  rides a sphere of radius `L` around the tip; the tip stays dead still. This is
  the "wow" shot — the strongest "IK is solving" signal.

Act 1 reads from a distance; Act 2 delivers the payoff. Cut between them.

## Scope

**In scope:** a host-side choreography tool that generates, validates offline,
and streams a pose sequence to the existing firmware.

**Out of scope / explicitly NOT building:**
- No firmware changes. The existing `$movec X Y Z ROLL PITCH YAW [TIME_MS]`
  console command (`app/src/comms/uart_console.c:370`) and body-frame IK solver
  (`app/src/kinematics/inverse_kinematics_poe.c`) are sufficient.
- No firmware "track a point" mode. All aim/pin math is computed host-side; the
  firmware just solves each full 6-DOF pose it is handed.
- No onboard/untethered playback. Laptop stays tethered over serial during the
  shoot (decided over the precompute-to-NVS alternative for speed of iteration).
- No physical laser pointer or target prop. The point stays invisible and is
  annotated in the post edit.

## Architecture

A single new host tool, `validation/choreo.py`, placed alongside the offline
math it reuses (`robot_config.yaml`, `modern_robotics`). Four internal stages:

```
robot_config.yaml ─► [1 path gen] ─► [2 pose solve] ─► [3 validate gate] ─► [4 stream]
                      parametric       aim / pin         FK+IK offline        $movec lines
                      keyframes        constraint        reachable+smooth     over serial
```

Stages 1–3 also dump the resolved sequence to a `*.jsonl` pose file, so a take
can be re-run byte-identical without recomputation (and so a known-good take is
archived).

### Why this approach (Approach A)

Chosen over a minimal hardcoded script (B) and jog-to-keyframe capture (C). The
parametric generator with an *offline validation gate* is barely more code than
B, reuses the already-validated `modern_robotics` math, and the validation pass
is what turns "hope the take works" into "the take works" — unreachable poses
and IK branch-flips are caught at the desk, not on camera. The home-orientation
gotcha (identity orientation → `CMOVE_NO_SOLUTION`) is exactly the class of
failure this gate prevents.

## Component detail

### Tool model

The virtual tool is a length `L = 0.02 m` (2 cm) along the flange's local **+Z**
axis. At the home configuration, local +Z points along the base **+X** axis
(home orientation RPY = -90, 0, -90). Tip position in the base frame:

```
p_tip = p_flange + R_flange · [0, 0, L]
```

`L` is a tunable constant at the top of the file; default 0.02.

### Stage 1 — Path generation (parametric)

A top-of-file config (dataclass) declares:
- `target` `T` — the fixed point, in base-frame metres.
- `tool_length` `L` — default 0.02.
- a list of **acts**, each a named parametric primitive with parameters
  (amplitude, turns, sample count, EE speed).

Primitives: `orbit`, `nod`, `figure_eight` (for Act 1 flange paths) and
`spherical_sweep`, `nod` (for Act 2 orientation sweeps). Two presets shipped:
- **Act 1 (aim):** orbit + figure-eight whose flange positions pass through a
  range of viewing angles on `T`.
- **Act 2 (pin):** spherical sweep + nod of orientation around the fixed tip.

### Stage 2 — Pose solve (the two constraints)

- **Aim:** for each generated flange position `p`, build `R` so that local +Z =
  `normalize(T − p)`. The remaining roll-about-axis DOF is chosen to minimize
  the change from the previous pose's roll (continuity), rather than a fixed
  global up-vector — this avoids gimbal surprises as the axis swings.
- **Pin:** fix tip at `T`; for each swept orientation `R`, solve
  `p = T − R · [0, 0, L]`. As `R` sweeps, `p` rides a sphere of radius `L`
  around `T`; the tip stays at `T`.

Output per frame: `(p, RPY)`.

### Stage 3 — Validation gate (the de-risk)

For every generated pose, offline, using `robot_config.yaml`:
1. Run `modern_robotics.IKinBody`, seeded from the **previous pose's** joint
   solution → must converge.
2. Resulting joint angles must lie within `joint_limits` from the config.
3. Joint-space delta from the previous frame must be below a continuity
   threshold → rejects IK branch-flips that would look like a jerk on camera.

Any rejected pose **fails the build**, reporting the offending act + frame index
and the reason (no convergence / limit / discontinuity). On success, prints a
summary per act: minimum reach margin and maximum per-frame joint step.

### Stage 4 — Streaming & pacing

- Streams plain text lines `$movec X Y Z ROLL PITCH YAW TIME_MS\n` over serial
  to the console path (NOT host_test.py's binary 0xAA packets).
- `TIME_MS` per segment is derived from a target EE speed so motion looks even.
- Sends `$power 1` first (firmware precondition — see below).
- Pauses between acts for camera repositioning (operator presses a key to
  continue).
- Flags: `--port`, `--dry-run` (print lines, no serial), `--validate-only`
  (run stages 1–3 and report, no hardware).

## Pre-flight checklist (printed by the tool on start)

These are hardware preconditions the demo silently depends on:

1. **`$power 1` sent** — Feetech timed-move drives on GOAL_SPEED; without
   power-on the computed speed is 0 and the arm will not move. The tool sends
   this automatically; checklist confirms it.
2. **Joint-map calibration flashed** — if the joint map is off, the arm points
   where the servos think, not where the math says. Manual operator confirm.
3. **Unicore build** — `CONFIG_SMP=y` boot-loops on hardware; firmware must run
   unicore.
4. **Target `T` inside validated workspace** — verified by the Stage 3 gate.

## Error handling

- **Unreachable / discontinuous pose:** build fails offline (Stage 3) before any
  serial output. Reported with frame index and reason.
- **Serial write failure mid-stream:** abort, report last successfully sent
  frame index so the take can be resumed or restarted.
- **Firmware `movec failed` warning:** the gate should make this impossible for
  validated sequences; if it occurs it indicates calibration/workspace drift —
  surfaced to the operator, take aborted.

## Testing

- `choreo.py --validate-only` runs offline against `robot_config.yaml` with no
  hardware.
- A pytest (alongside the existing `validation/test_*_crossval.py`) asserts both
  shipped presets pass the full Stage 3 gate: every pose reachable, within
  limits, and below the continuity threshold. This proves the choreography is
  sound in CI without a robot attached.

## Open parameters (tunable, not blocking)

- Per-act amplitudes / sample counts — tuned by eye during dry runs.
- Continuity threshold (rad per frame) — start conservative, relax if it rejects
  visually-fine motion.
- Target EE speed → `TIME_MS` mapping.
