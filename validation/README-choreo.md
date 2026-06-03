# IK Reveal Demo — choreo.py

Generates and streams a single "reveal" take: the virtual tool tip is pinned at
the EE home point `(0.168, 0, 0.243)`, the EE retreats one tool length while
pitching down ~25°, then the tool axis pans left/right (±45°) so the arm
reconfigures around the fixed tip, and finally returns to the level home pose.
No firmware changes; it streams `$movec` console commands.

The constant downward tilt keeps the wrist off the J5=0 gimbal-lock — a perfectly
level sweep would cross that singularity and flip J4/J6 ~100°.

## Install
    cd validation && pip install -r requirements.txt

## Prove the choreography offline (no robot)
    python choreo.py --validate-only
    python choreo.py --dry-run | less          # inspect the $movec lines

## Run it on hardware
    python choreo.py --port /dev/ttyUSB0

The single take streams end-to-end (approach → sweep → return); no pause.

## Pre-flight (the tool prints this; items marked [you] are manual)
1. `$power 1` — sent automatically (Feetech servos move on GOAL_SPEED; without
   power the computed speed is 0 and the arm won't move).
2. Joint-map calibration flashed — otherwise the arm points where the servos
   think, not where the math says.
3. Firmware built UNICORE (`CONFIG_SMP=n`) — SMP boot-loops on hardware.
4. Target inside the validated workspace — enforced by the validation gate.

## Tuning the look
Edit `Config`: `target`, `sweep_deg` (pan half-angle), `tilt_deg` (downward tool
tilt — lower it toward the singularity at your own risk), `speed_mps`, frame
counts. Re-run `--validate-only` after every change; the shipped preset is
regression-guarded by `test_choreo.py::test_default_presets_pass_validation`.

## Reproducible takes
    python choreo.py --validate-only --dump take.jsonl
writes `take.reveal.jsonl` with the exact resolved poses.

## The target point is never physical
It is an invisible point in space; annotate it in the post edit (a drawn-in
circle). No laser or prop is mounted.
