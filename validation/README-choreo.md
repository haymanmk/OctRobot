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
