# AGENTS.md

Guidance for AI coding agents working in this repository. Agent-agnostic; tool-specific notes (e.g. Claude Code) live in their own files (`CLAUDE.md`).

## Project Snapshot

OctroBot — 6-DOF robot arm firmware on M5Stack Atom Lite (ESP32-PICO-D4), Zephyr RTOS v3.6.0. POE forward kinematics; Feetech SCS/STS bus servos over half-duplex UART @ 1 Mbps.

Current focus: **Phase 4** (kinematics — FK done, IK pending). See `## Phase Roadmap` in `CLAUDE.md` for the full picture.

## Setup

```bash
make setup    # one-time: downloads Zephyr v3.6.0 to ~/zephyrproject
```

The Makefile auto-sources `~/zephyrproject/.venv`. Override with `ZEPHYR_BASE=...` for a global Zephyr install.

## Common Commands

| Command          | Purpose                                                |
| ---------------- | ------------------------------------------------------ |
| `make build`     | Compile for `m5stack_atom_lite/esp32/procpu`           |
| `make flash`     | Flash via esptool.py                                   |
| `make monitor`   | Serial console (`screen /dev/ttyUSB0 115200`)          |
| `make bfm`       | build + flash + monitor (primary dev loop)             |
| `make test`      | Native sim tests + Python FK cross-validation          |
| `make test-native` | Kinematics math tests (host machine)                 |
| `make test-py`   | Pytest FK cross-check vs `modern_robotics`             |
| `make clean`     | Wipe build artifacts                                   |

Python validation (one-time):
```bash
cd validation && pip install -r requirements.txt
```

## Code Map

```
app/src/
├── hal/          # half-duplex UART (GPIO 21 TX, 25 RX), e-stop (GPIO 39), NVS
├── drivers/      # Feetech wire protocol + servo API
├── kinematics/   # matrix/vector ops, matrix exp, POE model, FK
├── comms/        # binary packet protocol (0xAA CMD LEN PAYLOAD CRC8), USB/UART
├── controller/   # servo_control.c (Phase 6 placeholder)
└── trajectory/   # Phase 5 — not started
```

Out-of-tree:
- `tests/kinematics/` — Zephyr native_sim test suite for kinematics math
- `validation/` — Python cross-validation harness (FK vs `modern_robotics`)
- `host_test.py` — interactive CLI that speaks the binary packet protocol
- `boards/` — board overlays
- `docs/` — design notes (incl. `FK_TEST_README.md`)

## Conventions

- **Float precision:** all kinematics is `float32`. FPU is enabled (`CONFIG_FPU=y`). Don't introduce `double` in hot paths.
- **POE formulation:** follows Lynch & Park *Modern Robotics*. Screw axes (ξ₁–ξ₆) and home matrix M live in NVS with CRC32; factory defaults load on corruption.
- **Half-duplex UART:** TX line is pulled high for receive. Direction control lives in `hal/half_duplex_uart.c` — keep it there.
- **Logging:** level is `WARNING` because UART0 carries the binary protocol. Do **not** raise verbosity without isolating the protocol channel.
- **SMP:** enabled (`CONFIG_SMP=y`) but no dual-core code yet — keep new code SMP-safe.

## Testing Expectations

Before declaring kinematics work done:
1. `make test-native` passes (host-side math).
2. `make test-py` passes (cross-validation against `modern_robotics`).
3. If touching servo or HAL code, validate on hardware via `host_test.py` (`ping`, `jog`, `read`, `test`).

Hardware is required for Phase 3+ validation — UI/firmware claims without a hardware run should say so explicitly.

## Gotchas

- **Don't print to stdout/UART0** in firmware code paths — it corrupts the host packet protocol.
- **Don't bump the Zephyr version** casually; board overlays and HAL drivers are pinned to v3.6.0 semantics.
- **NVS keys are versioned;** when changing the geometry struct layout, bump the schema and add a migration path or factory-reset fallback.
- **`build/` and `build-test/`** are throwaway — never commit them, never edit files inside.

## How to Verify a Change

| Change touches…                  | Minimum verification                                      |
| -------------------------------- | --------------------------------------------------------- |
| `kinematics/`                    | `make test-native && make test-py`                        |
| `drivers/` or `hal/`             | `make build`, then hardware smoke via `host_test.py`      |
| `comms/` packet protocol         | Round-trip a `ping` from `host_test.py`                   |
| Build system / Kconfig / overlays | `make clean && make build`                               |
| Docs / comments only             | `make build` (sanity) — no runtime test required          |

## Where to Look for More Context

- `README.md` — user-facing project intro
- `QUICKSTART.md` — get-running-fast guide
- `CLAUDE.md` — Claude Code-specific phrasing of this same material
- `docs/` — deeper design notes, including FK test methodology
- `west.yml` — Zephyr manifest pin
