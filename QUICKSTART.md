# Quick Start Guide

## Phase 1 Complete ✅

Your OctroBot firmware project scaffold is ready!

## What Was Created

### Core Files
- `west.yml` - West manifest defining Zephyr v3.6.0 as dependency
- `app/CMakeLists.txt` - Build configuration
- `app/prj.conf` - Zephyr kernel configuration
- `app/src/main.c` - Hello World with UART console

### Board Definition
- `boards/m5stack_atom_lite/` - Complete custom board support
  - Device tree (`.dts`)
  - Pin control configuration
  - Kconfig files
  - Build scripts

### Documentation
- `README.md` - Project overview and development guide
- `setup.sh` - Automated setup script

## Next Steps

### Step 1: Install West and Initialize Workspace

```bash
# Install West
pip3 install --user west

# Add to PATH (if needed)
export PATH="$HOME/.local/bin:$PATH"

# Initialize workspace
cd /home/hayman/Workspace/octrobot
west init -l .
west update
```

The `west update` command will download Zephyr RTOS and all dependencies into a hidden `.west/` directory.

### Step 2: Install Zephyr SDK

Download and install the Zephyr SDK with Xtensa (ESP32) toolchain:

```bash
# Download SDK (adjust version as needed)
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.5/zephyr-sdk-0.16.5_linux-x86_64.tar.xz

# Extract
tar xvf zephyr-sdk-0.16.5_linux-x86_64.tar.xz

# Run setup (installs to ~/zephyr-sdk-0.16.5)
cd zephyr-sdk-0.16.5
./setup.sh -t xtensa-espressif_esp32_zephyr-elf
```

### Step 3: Install Python Dependencies

```bash
pip3 install -r ~/.zephyr/modules/zephyr/scripts/requirements.txt
```

Or use the provided setup script:

```bash
./setup.sh
```

### Step 4: Build

```bash
west build -b m5stack_atom_lite/esp32/procpu app
```

Expected output location: `build/zephyr/zephyr.bin`

### Step 5: Flash

Connect M5Stack Atom Lite via USB-C:

```bash
west flash
```

Or manually with esptool:

```bash
esptool.py --chip esp32 --port /dev/ttyUSB0 write_flash 0x10000 build/zephyr/zephyr.bin
```

### Step 6: Monitor Console

```bash
# Using screen
screen /dev/ttyUSB0 115200

# Or using minicom
minicom -D /dev/ttyUSB0 -b 115200

# Or West's built-in monitor
west espressif monitor
```

Expected output:
```
===========================================
OctroBot Robot Arm Firmware v0.1.0
===========================================
MCU: ESP32-PICO-D4 (M5Stack Atom Lite)
RTOS: Zephyr RTOS v3.6.0
===========================================
Console: UART0 via CH340 USB-UART bridge
System initialization complete
6-DOF robot arm ready
Heartbeat: 0 seconds
Heartbeat: 5 seconds
...
```

## Troubleshooting

### West not found
Add `~/.local/bin` to PATH:
```bash
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc
source ~/.zshrc
```

### Build fails with "board not found"
Ensure `BOARD_ROOT` points to your project:
```bash
export BOARD_ROOT=/home/hayman/Workspace/octrobot
```

Or set in CMakeLists.txt (already done).

### ESP32 toolchain not found
Install Zephyr SDK with ESP32 support or set `ESPRESSIF_TOOLCHAIN_PATH`.

### Flash fails
- Check USB cable (must support data)
- Verify port: `ls /dev/ttyUSB*` or `/dev/ttyACM*`
- Try holding BOOT button during flash
- Install udev rules for USB permissions

### No USB console output
- Ensure only one process owns the port: `lsof /dev/ttyUSB0`
- Close VS Code serial monitor, `screen`, `minicom`, or any stale monitor before opening a new one
- Use stable by-id path when monitoring:
  - `west espressif monitor --port /dev/serial/by-id/usb-Hades2001_M5stack_81524BF67C-if00-port0`
- If you still see only bootloader warnings about unsupported chip revision on older branches, set:
  - `CONFIG_ESP32_USE_UNSUPPORTED_REVISION=y`

## What's Next? (Phase 2)

After verifying the hello world works:

1. Implement HAL layer (`app/src/hal/half_duplex_uart.c`)
2. Map UART1 to Feetech servo bus
3. Add GPIO control for TX/RX direction switching
4. Implement timer utilities for precise control loop timing

See [plan-octrobotRobotArmFirmwareStack.prompt.md](.github/prompts/plan-octrobotRobotArmFirmwareStack.prompt.md) for detailed phase breakdown.

## Project Philosophy

This is an **out-of-tree Zephyr application** where:
- Your application code stays in `octrobot/` (your Git repo)
- Zephyr kernel is pulled as a dependency via West
- Custom board definition lives with your project
- You control Zephyr version via `west.yml`

This is the recommended approach for production firmware, not Zephyr kernel development.

---

**Ready to code!** 🚀
