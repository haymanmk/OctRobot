# `$movec` motion-complete handshake

## Goal

Replace the host's fixed `time.sleep(time_ms * 1.3)` pacing with a real
motion-complete handshake: the firmware replies `$movec ok` once the servos have
actually stopped, so the choreographer advances as fast as the hardware allows
and never under/over-waits.

## Protocol

Over UART0 (the console channel), in response to a `$movec ...` line:

- Success: `$movec ok\n` — sent **after** motion completes.
- Failure: `$movec err <status>\n` — sent instead, where `<status>` is the
  `cmove_status_t` (1=no-solution, 2=out-of-limits, 3=e-stop, 4=servo-err,
  5=bad-args) or the handler's errno path.

Both lines are emitted via `uart_console_reply()` (direct `uart_poll_out`), so
they are clean lines distinct from `LOG_*` output. The host ignores any line that
does not start with `$movec ok` / `$movec err`.

## Firmware

**New driver helper** (`feetech_servo.c` / `.h`):
```
int feetech_servo_wait_until_stopped(const uint8_t *ids, uint8_t count,
                                     uint16_t timeout_ms);
```
Poll each servo's `MOVING` register (0x42, already read by
`feetech_servo_read_state`) every ~20 ms until all report stopped, or
`timeout_ms` elapses. Returns 0 if all stopped, non-zero on timeout (caller still
proceeds — the timeout only bounds a flaky MOVING flag).

**`$movec` handler** (`uart_console.c`): after `cartesian_move_to_pose(... ,
move_time_ms)` returns:
- `CMOVE_OK` → `feetech_servo_wait_until_stopped(ids, 6, move_time_ms + 500)`
  then `uart_console_reply("$movec ok\n")`.
- otherwise → `uart_console_reply("$movec err %d\n", status)`.

The console thread is sequential, so blocking it for the move is fine for the
choreographer's one-at-a-time streaming. The e-stop button is a GPIO interrupt
(not on this thread), so it still fires during the wait.

## Host (`choreo.py`)

- New pure helper `classify_reply(line) -> "ok" | "err" | None` — matches the two
  tokens, returns `None` for log/other lines (unit-testable, no serial).
- `stream_frames`: for each frame, write the `$movec` line, then read serial
  lines, feeding each to `classify_reply`, until:
  - `"ok"` → advance to next frame;
  - `"err"` → print the line and **abort** the take (return early);
  - hard timeout `move_time_ms * 2 + 1000 ms` with no decisive reply → print a
    warning and **abort** (don't hang).
- Remove the `time.sleep` pacing. `time_ms_for` still computes the `TIME_MS`
  field of the `$movec` line (the move-duration hint the servos interpolate over).
- `--dry-run` (no serial) keeps printing lines only; no handshake.

## Testing

- Host: unit-test `classify_reply` (ok/err/log/garbage); test `stream_frames`
  with a fake serial object that returns scripted lines (ok stream advances; an
  err line aborts; silence triggers the timeout-abort).
- Firmware: manual hardware check — stream the reveal take and confirm the arm
  advances on each `$movec ok` with no fixed delay.

## Out of scope

- Binary packet protocol; other console commands; the IK/pacing math.

## Risks

- **MOVING flag reliability:** some Feetech units report a flaky flag → mitigated
  by the `move_time_ms + 500` timeout (reply `ok` anyway).
- **UART0 log interleaving:** the handshake line shares the bus with `LOG_*`
  output; mitigated by the host token match. Heavy per-frame logging is tolerable
  at 115200 for ~150 frames but could be lowered later if it bottlenecks.
- **Reflash required** for the firmware side to take effect.
