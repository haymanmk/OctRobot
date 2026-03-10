# OctroBot Robot Arm Firmware

6-DOF robot arm firmware for M5Stack Atom Lite (ESP32) running Zephyr RTOS.

## Features

- **Platform**: ESP32-PICO-D4 (dual-core @ 240MHz, 520KB SRAM, 4MB Flash)
- **RTOS**: Zephyr RTOS v3.6.0
- **Servo Control**: Feetech Serial Bus Servos (SCS/STS protocol)
- **Communication**: UART console via CH340 USB-UART bridge
- **Architecture**: Layered C firmware (HAL → Drivers → Kinematics → Controller → Command Interface)

## Project Structure

```
octrobot/
├── app/                      # Application source code
│   ├── src/
│   │   ├── main.c           # Main entry point
│   │   ├── hal/             # Hardware abstraction layer
│   │   ├── drivers/         # Servo drivers
│   │   ├── kinematics/      # Forward/inverse kinematics
│   │   ├── trajectory/      # Motion planning
│   │   ├── controller/      # Motion controller
│   │   └── comms/           # Host protocol
│   ├── include/             # Public headers
│   ├── CMakeLists.txt
│   └── prj.conf
├── boards/                   # Custom board definitions
│   └── m5stack_atom_lite/   # M5Stack Atom Lite board
├── west.yml                 # West manifest (Zephyr workspace)
└── README.md

```

## Getting Started

### Prerequisites

- Python 3.8+
- West (Zephyr's meta-tool): `pip install west`
- Zephyr SDK or toolchain for ESP32/Xtensa
- esptool.py for flashing

### Initial Setup

1. **Initialize the West workspace** (first time only):

```bash
cd /home/hayman/Workspace/octrobot
west init -l .
west update
```

This will download Zephyr and its dependencies into `.west/modules/`.

2. **Install Zephyr SDK** (if not already installed):

Follow the [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html).

3. **Install Python dependencies**:

```bash
pip install -r ~/.west/modules/zephyr/scripts/requirements.txt
```

### Building

```bash
west build -b m5stack_atom_lite/esp32/procpu app
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

## Development Phases

- **Phase 1**: ✅ Project scaffold & toolchain setup
- **Phase 2**: ✅ HAL layer (UART, GPIO, timers)
- **Phase 3**: ✅ Feetech servo driver
- **Phase 4**: Kinematics (FK/IK)
- **Phase 5**: Trajectory planner
- **Phase 6**: Motion controller
- **Phase 7**: Host command protocol (UART serial)
- **Phase 8**: Integration & testing

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
│     Host Command Interface          │ ← UART Serial Protocol
├─────────────────────────────────────┤
│      Motion Controller              │ ← 1ms control loop
├─────────────────────────────────────┤
│     Trajectory Planner              │ ← Joint-space interpolation
├─────────────────────────────────────┤
│   Kinematics (FK/IK)                │ ← DH parameters
├─────────────────────────────────────┤
│   Servo Driver (Feetech SCS/STS)   │ ← SYNC_WRITE for 6 joints
├─────────────────────────────────────┤
│   HAL (UART, GPIO, Timers)          │ ← Zephyr device drivers
├─────────────────────────────────────┤
│   Zephyr RTOS (ESP32)               │
└─────────────────────────────────────┘
```

## License

Apache-2.0

## Future Plans

- ROS2 integration (host-side bridge node)
- MoveIt2 integration for advanced planning
- Web-based control interface
- OTA firmware updates

---

**Status**: Phase 3 complete - Servo driver functional

See [docs/HAL_LAYER.md](docs/HAL_LAYER.md) for HAL documentation.
See [docs/SERVO_DRIVER.md](docs/SERVO_DRIVER.md) for servo driver documentation.
