# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

OctroBot is a 6-DOF robot arm firmware for the M5Stack Atom Lite (ESP32-PICO-D4) running Zephyr RTOS v3.6.0. It uses Product of Exponentials (POE) forward kinematics and communicates with Feetech SCS/STS serial bus servos over half-duplex UART at 1 Mbps.

## Build Commands

```bash
make setup        # Initialize West workspace (downloads Zephyr v3.6.0 to ~/zephyrproject)
make build        # Compile firmware for m5stack_atom_lite/esp32/procpu
make flash        # Flash to device via esptool.py
make monitor      # Open serial console (screen /dev/ttyUSB0 115200)
make bfm          # build + flash + monitor (main dev cycle)
make clean        # Clean build artifacts
make test         # Run both native simulator tests and Python validation
make test-native  # Native simulator tests (kinematics math)
make test-py      # Python pytest FK cross-validation against modern_robotics
```

The Makefile auto-sources `~/zephyrproject/.venv`. Set `ZEPHYR_BASE` to override with a global Zephyr install.

For Python validation:
```bash
cd validation && pip install -r requirements.txt
cd validation && python -m pytest -v
```

## Architecture

The firmware is organized in phase-based layers, bottom-up:

```
Host Protocol (USB CDC-ACM binary packets)
    ↓
Motion Controller  (Phase 6 - partial placeholder)
    ↓
Kinematics Layer   (Phase 4 - POE FK + body-frame IK complete; offline-validated)
    ↓
Servo Driver       (Phase 3 - complete; Feetech SYNC_WRITE for 6 joints)
    ↓
HAL Layer          (Phase 2 - complete; half-duplex UART, GPIO, NVS flash)
    ↓
Zephyr RTOS / ESP32-PICO-D4
```

**Source layout** (`app/src/`):
- `hal/` — half-duplex UART (GPIO 21 TX, GPIO 25 RX), GPIO emergency stop (GPIO 39), NVS flash storage
- `drivers/` — Feetech protocol (low-level register read/write) and servo API (high-level)
- `kinematics/` — matrix/vector ops, matrix exponentials, robot geometry (POE model + flash persistence), FK computation
- `comms/` — binary packet protocol (0xAA CMD LEN PAYLOAD CRC8), USB/UART command handlers, console parsing
- `controller/` — servo_control.c placeholder (Phase 6)
- `trajectory/` — Phase 5, not yet started

**Key design choices:**
- POE kinematics uses Lynch & Park *Modern Robotics* formulation. Screw axes (ξ₁–ξ₆) and home configuration matrix M are stored in NVS flash with CRC32 validation; factory defaults load if flash is corrupted.
- FPU is enabled (`CONFIG_FPU=y`) — all kinematics uses float32.
- SMP is enabled (`CONFIG_SMP=y`) for future dual-core use.
- Half-duplex UART requires direction control (TX line pulled high for receive). Direction is managed in `hal/half_duplex_uart.c`.
- Demo recording stores up to 50 waypoints in NVS for playback.

## Host Test Script

`host_test.py` is an interactive Python CLI that communicates with the device using the same binary packet protocol. Commands: `ping`, `jog`, `set`, `read`, `loop`, `test`, `record`, `waypoint`, `play`, `clear`. Use this for Phase 3b hardware validation without a host application.

## Phase Roadmap

| Phase | Status | Focus |
|-------|--------|-------|
| 1–3b  | ✅ Done | Scaffold, HAL, servo driver, USB manual control |
| 4     | ⏳ In Progress | FK complete; IK solver complete + offline-validated; IK not yet wired to motion control |
| 5     | ❌ Not started | Trajectory planner (joint-space interpolation) |
| 6     | ❌ Partial | Motion controller (1 ms loop, PID) |
| 7–8   | ❌ Not started | Full host protocol, system integration |

## Logging

Log level is set to WARNING to avoid interfering with the binary packet protocol on UART0. Do not increase verbosity without accounting for protocol interference.
