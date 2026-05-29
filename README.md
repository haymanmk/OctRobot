# OctroBot Robot Arm Firmware

6-DOF robot arm firmware for M5Stack Atom Lite (ESP32) running Zephyr RTOS.

## Features

- **Platform**: ESP32-PICO-D4 (dual-core @ 240MHz, 520KB SRAM, 4MB Flash)
- **RTOS**: Zephyr RTOS v3.6.0
- **Servo Control**: Feetech Serial Bus Servos (SCS/STS protocol, 1 Mbps half-duplex UART)
- **Communication**: USB CDC-ACM (native USB) for host commands, CH340 for debug console
- **Protocol**: Binary packet format with CRC8 error detection
- **Architecture**: Layered C firmware (HAL → Drivers → Kinematics → Controller → Command Interface)

## Project Structure

```
octrobot/
├── app/                      # Application source code
│   ├── src/
│   │   ├── main.c           # Main entry point
│   │   ├── hal/             # Hardware abstraction layer
│   │   ├── drivers/         # Servo drivers
│   │   ├── kinematics/      # POE forward kinematics
│   │   ├── trajectory/      # Motion planning (Phase 5)
│   │   ├── controller/      # Motion controller (Phase 6)
│   │   └── comms/           # Host protocol
│   ├── include/             # Public headers
│   ├── CMakeLists.txt
│   └── prj.conf
├── boards/                   # Custom board definitions
│   └── m5stack_atom_lite/   # M5Stack Atom Lite board
├── tests/kinematics/         # Zephyr native-sim unit tests
│   ├── src/                 # Test sources (math, matrix exp, FK, ground-truth)
│   └── include/             # Auto-generated fk_test_vectors.h
├── validation/               # Python FK cross-validation
│   ├── robot_config.yaml    # POE model (screw axes + home config M)
│   ├── gen_test_vectors.py  # Generates ground-truth from modern_robotics
│   ├── fk_test_vectors.json # Ground-truth FK results (float64)
│   └── test_fk_crossval.py  # pytest cross-validation suite
├── west.yml                 # West manifest (Zephyr workspace)
└── README.md
```

## Getting Started

### Prerequisites

- Python 3.8+
- Existing Zephyr installation with virtual environment at `~/zephyrproject/.venv`
  - OR `ZEPHYR_BASE` environment variable pointing to your Zephyr installation
- Zephyr SDK or toolchain for ESP32/Xtensa
- esptool.py for flashing

### Initial Setup

This project can use either:
- **Option A:** Existing Zephyr installation at `~/zephyrproject/.venv` (managed by west)
- **Option B:** Global Zephyr installation via `ZEPHYR_BASE` environment variable

**Using Option A (local west workspace):**

1. **Run the setup script** (first time only):

```bash
cd /home/hayman/Workspace/octrobot
./setup.sh
```

This will:
- Verify the Zephyr virtual environment exists
- Initialize the West workspace
- Download Zephyr and dependencies
- Install required Python packages

**Using Option B (ZEPHYR_BASE):**

If you have `ZEPHYR_BASE` set, the setup script will automatically detect it and skip workspace initialization:

```bash
export ZEPHYR_BASE=/path/to/zephyr  # Already set in your environment
cd /home/hayman/Workspace/octrobot
./setup.sh  # Will skip west init/update
```

2. **Activate the virtual environment** (required for each terminal session):

```bash
source ~/zephyrproject/.venv/bin/activate
```

Or use the Makefile targets which handle this automatically:

```bash
make build    # Automatically activates venv and builds
make flash    # Automatically activates venv and flashes
```

### Building

```bash
# Manual method (requires active virtual environment)
source ~/zephyrproject/.venv/bin/activate
west build -b m5stack_atom_lite/esp32/procpu app

# Or use Makefile (handles venv automatically)
make build
```

### Dual-Core Target Selection

ESP32 is dual-core, so Zephyr board targets require a core qualifier:

- `m5stack_atom_lite/esp32/procpu` (recommended default for this project)
- `m5stack_atom_lite/esp32/appcpu` (advanced/AMP use)

For this firmware stack, use `PROCPU` unless you are intentionally building an AMP setup.

Examples:

```bash
# Default (recommended)
west build -b m5stack_atom_lite/esp32/procpu app

# Alternate core (advanced)
west build -b m5stack_atom_lite/esp32/appcpu app
```

### Flashing

Connect the M5Stack Atom Lite via USB-C and flash:

```bash
west flash
```

### Monitoring Console Output

```bash
west espressif monitor
# or
screen /dev/ttyUSB0 115200
```

## Quick Start: Testing Phase 3b Manual Control

After flashing the firmware, you can immediately test manual control:

1. **Connect via USB CDC-ACM:**
   ```bash
   python3 host_test.py /dev/ttyACM0
   ```

2. **Try basic commands:**
   ```
   > ping              # Verify all 6 servos respond
   > jog 0 10.0        # Move joint 0 by +10 degrees
   > set 0 1 2 3 4 5   # Set all joints to specific angles
   > read              # Read current joint positions
   > loop 100          # Start 100Hz position streaming
   > stop              # Stop all motion
   ```

3. **Full testing guide:**
   - See [docs/phase3b_validation.md](docs/phase3b_validation.md) for complete test procedures
   - See [docs/PHASE3B_SUMMARY.md](docs/PHASE3B_SUMMARY.md) for implementation details
   - See [docs/FK_TEST_README.md](docs/FK_TEST_README.md) for FK cross-validation details

## Testing

### Run all tests

```bash
make test          # native-sim unit tests + Python cross-validation
make test-native   # Zephyr native-sim unit tests only
make test-py       # Python pytest cross-validation only
```

### Test layers

| Layer | Tool | What it checks |
|---|---|---|
| **C unit tests** | Zephyr native-sim (`make test-native`) | Math ops, matrix exponential, FK properties, FK vs ground truth |
| **Python cross-validation** | pytest (`make test-py`) | FK via `modern_robotics`, SE(3) validity, config-derived position assertions |

### Updating the robot model

When you change `validation/robot_config.yaml` (e.g., after hardware calibration), regenerate the ground-truth vectors before rebuilding:

```bash
cd validation
python gen_test_vectors.py   # writes fk_test_vectors.json + tests/kinematics/include/fk_test_vectors.h
cd ..
make test
```

See [docs/FK_TEST_README.md](docs/FK_TEST_README.md) for full details on the cross-validation pipeline.

## Development Phases

- **Phase 1**: ✅ Project scaffold & toolchain setup
- **Phase 2**: ✅ HAL layer (UART, GPIO, timers)
- **Phase 3**: ✅ Feetech servo driver
- **Phase 3b**: ✅ USB CDC-ACM manual control interface (early validation)
- **Phase 4**: ⏳ Kinematics — POE FK complete (Mecharm 270 Pi); body-frame IK solver complete (offline-validated); not yet wired to motion control
- **Phase 5**: ❌ Trajectory planner
- **Phase 6**: ❌ Motion controller
- **Phase 7**: ❌ Full host command protocol (trajectory commands)
- **Phase 8**: ❌ Integration & testing

### Phase 3b: Manual Control & Validation

Phase 3b implements an **early validation interface** before complex kinematics. This allows testing servo communication and basic motion control with real hardware.

**Features:**
- UART-based binary packet protocol (CRC8 error detection)
- Manual joint control commands (jog, direct positioning)
- Position streaming (100Hz continuous readback)
- Demo recording/playback (up to 50 waypoints per demo)
- Python interactive CLI for testing
- Emergency stop command

**Important Note:**
Phase 3b uses UART0 for both debug logs and command packets. Debug logging is reduced to WARNING level to minimize interference. For production use, consider disabling logging (`CONFIG_LOG=n` in prj.conf) or using a separate UART for commands.

**Documentation:**
- [Phase 3b Implementation Summary](docs/PHASE3B_SUMMARY.md) - Complete feature overview, design notes, code statistics
- [Phase 3b Validation Guide](docs/phase3b_validation.md) - Hardware setup, test procedures, troubleshooting

**Why Phase 3b?**
- Validates servo communication before implementing complex math
- Enables manual exploration of workspace and joint limits
- Provides tools for recording test trajectories
- Catches mechanical/electrical issues early
- Educational: demonstrates packet protocol design and USB CDC-ACM

## Hardware Pinout

| Function | GPIO | Notes |
|----------|------|-------|
| Servo Bus TX | 21 | To UART-to-serial converter |
| Servo Bus RX | 25 | From UART-to-serial converter |
| Status LED | 27 | WS2812B RGB LED |
| Emergency Stop | 39 | Button (active-low) |
| USB Console | UART0 | CH340 USB-UART bridge |

## Architecture

```
┌─────────────────────────────────────┐
│   Host USB CDC-ACM Protocol         │ ← Binary packet (CRC8) over native USB
├─────────────────────────────────────┤
│      Motion Controller              │ ← 1ms control loop (Phase 6)
├─────────────────────────────────────┤
│     Trajectory Planner              │ ← Joint-space interpolation (Phase 5)
├─────────────────────────────────────┤
│   Kinematics (FK/IK)                │ ← POE formula, Mecharm 270 Pi (Phase 4)
├─────────────────────────────────────┤
│   Servo Driver (Feetech SCS/STS)   │ ← SYNC_WRITE for 6 joints ✅
├─────────────────────────────────────┤
│   HAL (UART, GPIO, Timers)          │ ← Zephyr device drivers ✅
├─────────────────────────────────────┤
│   Zephyr RTOS (ESP32)               │
└─────────────────────────────────────┘
```

**Current Status (Phase 4 in progress):**
- ✅ HAL layer complete
- ✅ Feetech servo driver complete
- ✅ USB CDC-ACM packet protocol complete
- ✅ Manual control commands (jog, direct set, read state, demo recording)
- ✅ Forward kinematics — POE formulation, Mecharm 270 Pi, cross-validated against `modern_robotics`
- ✅ Inverse kinematics — body-frame IK solver complete (offline-validated); not yet wired to motion control
- ⏸️ Trajectory planning and motion controller pending

## License

Apache-2.0

## Future Plans

- ROS2 integration (host-side bridge node)
- MoveIt2 integration for advanced planning
- Web-based control interface
- OTA firmware updates

---

**Status**: Phase 4 in progress — POE FK complete (Mecharm 270 Pi); body-frame IK solver complete (offline-validated); not yet wired to motion control

See [docs/HAL_LAYER.md](docs/HAL_LAYER.md) for HAL documentation.
See [docs/SERVO_DRIVER.md](docs/SERVO_DRIVER.md) for servo driver documentation.
