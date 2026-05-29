# Cartesian Move-to-Pose — Design Spec

**Date:** 2026-05-29
**Phase:** 6 (Motion control — first increment)
**Status:** Draft for review (brainstorming output; not yet approved)

## Goal

Add a host command that moves the OctroBot arm to a commanded Cartesian
end-effector **pose** (position + orientation) by running the existing IK
solver and sending the resulting joint angles to the servos. This is the first
Phase 6 increment: it turns the offline-validated IK (see
`2026-05-29-inverse-kinematics-design.md`) into actual arm motion.

## Non-Goals (this round)

- No MCU-side trajectory interpolation or 1 ms PID loop. The Feetech servos
  close their own position loop; the MCU only computes goal angles and lets the
  servos move there with onboard smoothing.
- No binary `CMD_MOVE_CARTESIAN` (0x02) protocol work. The active interface is
  the text console; the binary path stays an inactive placeholder.
- No NVS persistence of joint calibration this round (constants in code; tune
  during bring-up). NVS calibration can come later.

## Decisions

Settled during brainstorming:

1. **Deliverable:** Cartesian move-to-pose, end-to-end, smallest first step.
2. **Full 6-DOF pose**, orientation as roll/pitch/yaw.
3. **Motion:** servo onboard smoothing (`feetech_servo_sync_write_angles` with a
   move time), no MCU trajectory loop.
4. **Interface:** new text console command `$movec` in `uart_console.c` (the
   interface `main()` actually initializes today).
5. **IK seed:** current joint angles read off the servos.
6. **Joint mapping:** per-joint sign + offset table, defaulting to identity,
   tuned during bring-up (see §Joint mapping).

### Open points to confirm on review

- **(a) RPY convention:** proposed **ZYX intrinsic**, `R = Rz(yaw)·Ry(pitch)·Rx(roll)`.
- **(b) Units:** position **x y z in meters** (model units, e.g. ~0.2);
  orientation **roll pitch yaw in degrees** (matches the degree-based `$set`).
- **(c) Move time:** optional trailing `time_ms` argument, default **1000 ms**.
- **(d) Servo-read failure handling:** if reading current positions for the IK
  seed fails, **fall back to an all-zeros seed and still attempt the move**
  (alternative: refuse). Proposed: fall back + warn.

## Architecture / file structure

Three small, independently testable units plus a console hook. The pure-math and
mapping pieces are unit-testable on native_sim; only the servo write needs
hardware.

- **`app/include/kinematics_math.h` / `app/src/kinematics/kinematics_math.c`**
  (extend): add `mat3x3_t mat3x3_from_rpy(float roll, float pitch, float yaw)`
  — pure rotation utility, ZYX intrinsic, inputs in **radians**.
- **`app/include/joint_map.h` / `app/src/controller/joint_map.c`** (new): the
  sign+offset calibration table plus two converters:
  - `void joint_map_model_to_servo(const float theta_rad[6], float servo_deg[6])`
    — `servo_deg[i] = sign[i] * rad_to_deg(theta_rad[i]) + offset_deg[i]`.
  - `void joint_map_servo_to_model(const float servo_deg[6], float theta_rad[6])`
    — inverse: `theta_rad[i] = deg_to_rad((servo_deg[i] - offset_deg[i]) * sign[i])`
    (valid because `sign[i] ∈ {+1,-1}`, so dividing by sign == multiplying).
  - Defaults: `sign[6]` all `+1`, `offset_deg[6]` all `0`.
- **`app/include/cartesian_move.h` / `app/src/controller/cartesian_move.c`**
  (new): orchestration, split into a servo-independent core and a hardware
  wrapper:
  - Core (unit-testable):
    `cmove_status_t cartesian_pose_to_joints(const poe_robot_model_t *model,
    float x, float y, float z, float roll_deg, float pitch_deg, float yaw_deg,
    const float seed_theta[6], float out_theta[6])` — RPY→`T_target`→IK,
    returns the mapped status.
  - Wrapper (hardware):
    `cmove_status_t cartesian_move_to_pose(float x, float y, float z,
    float roll_deg, float pitch_deg, float yaw_deg, uint16_t move_time_ms)` —
    e-stop check, read servos→seed, call core, map→servo degrees, range-check,
    `sync_write_angles`.
  - Status enum:
    `CMOVE_OK, CMOVE_NO_SOLUTION, CMOVE_OUT_OF_LIMITS, CMOVE_ESTOP,
    CMOVE_SERVO_ERR, CMOVE_BAD_ARGS`.
- **`app/src/comms/uart_console.c`** (extend): parse `$movec` and call
  `cartesian_move_to_pose`, report the result on the console.

Both new `.c` files must be registered in `app/CMakeLists.txt`, and the
testable ones (`kinematics_math.c` already present; add `joint_map.c` and
`cartesian_move.c`) in `tests/kinematics/CMakeLists.txt` (or a new controller
test target).

## Command and data flow

`$movec x y z roll pitch yaw [time_ms]`

1. Console parses 6 or 7 floats. Wrong count → `CMOVE_BAD_ARGS`.
2. If emergency stop is active → `CMOVE_ESTOP`, refuse (no motion).
3. Read current servo positions (degrees) → `joint_map_servo_to_model` → IK
   seed (model radians). On read failure → all-zeros seed + warn (per open
   point (d)).
4. `R = mat3x3_from_rpy(deg_to_rad(roll), deg_to_rad(pitch), deg_to_rad(yaw))`;
   `p = (x, y, z)`; build `T_target` from `R, p`.
5. `inverse_kinematics_compute(model, &T_target, seed, out_theta, NULL, NULL)`.
6. On `IK_SUCCESS`: `joint_map_model_to_servo(out_theta, servo_deg)`;
   range-check each `servo_deg[i]` against ±180° (reject whole move if any is
   out of servo range → `CMOVE_OUT_OF_LIMITS`); then
   `feetech_servo_sync_write_angles(ids, servo_deg, 6)` with the move time
   applied (`set_goal_time` or speed). Map `IK_OUT_OF_LIMITS` /
   `IK_NO_CONVERGENCE` / `IK_INVALID_INPUT` to the matching `CMOVE_*` error and
   do **not** move.
7. Report the outcome string on the console.

## Joint mapping (calibration)

The kinematic model's joint zero (`θ=0` ⇒ end-effector at home config `M`) is a
different reference from each servo's center (tick 2048 = 0°), and the model's
positive rotation (screw-axis sign) may oppose a servo's positive direction. The
sign+offset table bridges both: `sign[i]` (±1) fixes direction, `offset_deg[i]`
fixes the zero-point difference. It assumes a linear 1:1 angular relationship
(no gear-ratio scaling between model and servo units) — true here. Defaults are
identity; the twelve numbers are tuned during bring-up: command the model home
pose (all θ=0), read where each servo sits to get offsets, jog each joint to
find sign flips.

## Error handling

- E-stop active → refuse, `CMOVE_ESTOP`.
- IK failure categories surfaced distinctly (`CMOVE_NO_SOLUTION`,
  `CMOVE_OUT_OF_LIMITS`).
- Per-servo ±180° range check before writing; reject the whole move if any
  joint is out of servo range.
- Servo communication error on read/write → `CMOVE_SERVO_ERR`.
- Argument validation → `CMOVE_BAD_ARGS`.
- No motion is commanded on any failure path.
- No logging on the hot path that would interfere with the UART0 binary
  protocol (consistent with the project's WARNING log level).

## Testing

### Native simulator (no hardware)
- `mat3x3_from_rpy`: zero → identity; single-axis rotations (e.g. yaw 90° about
  Z); cross-checked against `modern_robotics` in a Python test for several RPY
  triples.
- `joint_map`: forward∘inverse round-trip returns the input, for **identity**
  and for a **non-identity** sign/offset table (test via a seam that lets the
  test set the table, or a second internal table — keep it simple).
- `cartesian_pose_to_joints`: pick known joint angles θ → FK → extract
  position + RPY from the resulting pose → feed back through the function →
  assert FK of the result matches the original pose within tolerance (pose
  round-trip, since IK solutions are non-unique).

### Hardware
- `make bfm`, send `$movec x y z roll pitch yaw`, observe the arm reach the
  pose, confirm with `$read`. The servo write path is validated here, not in
  unit tests.

## Acceptance criteria

- `$movec` with a reachable pose drives the arm there on hardware; `$read`
  confirms the joint angles.
- Unreachable / out-of-limits / e-stop / bad-args cases report the correct
  `CMOVE_*` status and command no motion.
- Native unit tests pass (`make test-native`); Python RPY cross-check passes
  (`make test-py`).
- IK and FK behavior unchanged; existing tests still pass.
