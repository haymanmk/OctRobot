# Cartesian Move-to-Pose ($movec)

Move the end-effector to a Cartesian pose. The firmware runs body-frame IK and
sends the resulting joint angles to the servos with an onboard move time.

## Command

```
$movec X Y Z ROLL PITCH YAW [TIME_MS]
```

- `X Y Z` — target position in **meters** (model frame; the home position is
  roughly `0.168 0 0.243`).
- `ROLL PITCH YAW` — target orientation in **degrees**, ZYX intrinsic
  (`R = Rz(yaw)·Ry(pitch)·Rx(roll)`).
- `TIME_MS` — optional servo move duration in ms (default **1000**, range
  1–60000).

Example (move slightly forward over 2 s):

```
$movec 0.20 0.00 0.243 -90 0 -180 2000
```

## Joint calibration (REQUIRED before trusting motion)

The kinematic model's joint zero is not the servo center. Until calibrated, the
`joint_map` table is identity (`sign=+1, offset=0`), so the arm will very likely
go to the wrong pose. Calibrate by:

1. Command the model home pose so the arm is in the kinematic home
   configuration, and note each servo's reported angle (`$read`) — those become
   `offset_deg[i]`.
2. Jog each joint in the model's positive direction; if the servo angle
   decreases, that joint needs `sign[i] = -1`.
3. Put the twelve numbers into `s_sign` / `s_offset_deg` in
   `app/src/controller/joint_map.c` (or call `joint_map_set_calibration`).

## Statuses

`$movec` reports a non-zero result on failure: no IK solution, out of limits,
emergency stop active, servo error, or bad arguments (see `cmove_status_t` in
`app/include/cartesian_pose.h`).

## Safety

Press the e-stop button (GPIO 39) to disable torque. `$movec` refuses to start a
move while the e-stop button is held.

## How it works

`$movec` (uart_console.c) → `cartesian_move_to_pose` (cartesian_move.c):
1. Refuse if the e-stop button is pressed.
2. Read current servo angles → `joint_map_servo_to_model` → IK seed.
3. `cartesian_pose_to_joints` (cartesian_pose.c): RPY+position → SE(3) target →
   `inverse_kinematics_compute` → model-frame joint angles.
4. `joint_map_model_to_servo` → servo degrees; range-check ±180°.
5. `feetech_servo_sync_write_angles_timed` — one atomic packet, all joints
   interpolate over `TIME_MS` and finish together.
