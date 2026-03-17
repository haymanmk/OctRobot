# Phase 3b - Early Manual Control Validation

## Overview

Phase 3b implements manual control and demo recording features to validate hardware and servo communication before proceeding to kinematics and trajectory planning.

## What's Implemented

### Firmware (MCU)
- ✅ USB CDC-ACM communication
- ✅ Binary packet protocol with CRC8 error detection
- ✅ Manual control commands:
  - JOG_JOINT - Incremental joint movements
  - SET_JOINT_DIRECT - Direct position control
  - READ_STATE - Query joint angles and servo status
  - START/STOP_READ_LOOP - Continuous 100Hz streaming
  - SINGLE_JOINT_TEST - Oscillate joint for testing
  - STOP - Emergency stop
- ✅ Demo recording system:
  - START_DEMO_RECORDING - Begin recording
  - ADD_WAYPOINT - Capture current position
  - FINISH_DEMO_RECORDING - Save to RAM
  - PLAY_DEMO - Playback recorded sequence
  - CLEAR_DEMO - Erase demo

### Host (PC)
- ✅ Python3 interactive CLI
- ✅ Packet protocol implementation
- ✅ All command interfaces
- ✅ Status parsing and display

## Hardware Setup

### M5Stack Atom Lite Connections
```
GPIO 21 (TX) ──→ Feetech Servo Bus TX
GPIO 25 (RX) ←── Feetech Servo Bus RX
GPIO 39 (BTN) ──→ Emergency Stop Button
USB-C ──────────→ Host PC (for CDC-ACM + power)
```

### Servo Configuration
- **IDs**: 1-6 (configure using Feetech software)
- **Baud Rate**: 1 Mbps
- **Power**: 6-12V external supply
- **Protocol**: SCS/STS serial bus

## Building and Flashing

### Prerequisites
```bash
# Install Zephyr SDK and dependencies
# Follow: https://docs.zephyrproject.org/latest/develop/getting_started/

# Install Python dependencies for host script
pip3 install pyserial
```

### Build Firmware
```bash
cd /home/hayman/Workspace/octrobot

# Option A: If using ZEPHYR_BASE environment variable
# (setup.sh will detect and skip workspace initialization)
./setup.sh

# Option B: Activate Zephyr virtual environment manually
source ~/zephyrproject/.venv/bin/activate

# Build
west build -b m5stack_atom_lite/esp32/procpu app

# Or use Makefile (handles venv automatically)
make build
```

### Flash to MCU
```bash
# With venv active:
west flash

# Or use Makefile:
make flash
```

### Monitor Console Output
```bash
# Via USB-UART (CH340 bridge on Atom Lite)
screen /dev/ttyUSB0 115200

# Or use minicom
minicom -D /dev/ttyUSB0 -b 115200
```

## Testing

### 1. Verify Firmware Boot
After flashing, check console output:
```
===========================================
OctroBot Robot Arm Firmware v0.3.0
===========================================
System initialization complete
Testing Servo Communication
===========================================
Detected 6/6 servos
```

### 2. Connect Host Test Script
In a new terminal:
```bash
cd /home/hayman/Workspace/octrobot
python3 host_test.py /dev/ttyACM0
```

Replace `/dev/ttyACM0` with your USB CDC-ACM device path.

### 3. Run Validation Tests

#### Test 1: Ping Servos
```
octrobot> read
```
Should display all 6 joint angles, temperatures, and voltages.

#### Test 2: Jog Joints
```
octrobot> jog 1 1 5    # Jog joint 1 forward by 5°
octrobot> jog 1 -1 5   # Jog joint 1 backward by 5°
octrobot> jog 2 1 10   # Jog joint 2 forward by 10°
```

#### Test 3: Direct Position Control
```
octrobot> set 1 0.0     # Set joint 1 to 0 radians
octrobot> set 2 0.5     # Set joint 2 to 0.5 radians
octrobot> set 3 -0.5    # Set joint 3 to -0.5 radians
```

#### Test 4: Continuous Streaming
```
octrobot> loop
# Watch 100Hz position updates, press Ctrl+C to stop
```

#### Test 5: Single Joint Test
```
octrobot> test 1 -0.5 0.5 5
# Oscillates joint 1 between -0.5 and 0.5 rad for 5 cycles
```

#### Test 6: Demo Recording
```
octrobot> record 0              # Start recording to slot 0
octrobot> jog 1 1 10            # Move to position 1
octrobot> waypoint 2000         # Add waypoint, wait 2s
octrobot> jog 2 1 10            # Move to position 2
octrobot> waypoint 1000         # Add waypoint, wait 1s
octrobot> jog 1 -1 10           # Move to position 3
octrobot> waypoint 2000         # Add waypoint, wait 2s
octrobot> finish                # Save demo
octrobot> play 0                # Playback demo
```

#### Test 7: Emergency Stop
```
octrobot> stop
# All servos should disable torque immediately
```

Or press the physical button (GPIO 39) on the MCU.

## Packet Protocol Details

### Format
```
[0xAA] [CMD] [LEN] [PAYLOAD...] [CRC8]
```

- **0xAA**: Start marker
- **CMD**: Command byte (see commands below)
- **LEN**: Payload length (0-128)
- **PAYLOAD**: Command-specific data
- **CRC8**: Dallas/Maxim polynomial (0x31)

### Command Reference

| CMD  | Name | Payload | Description |
|------|------|---------|-------------|
| 0x20 | JOG_JOINT | `[id, dir, deg]` | Jog joint incrementally |
| 0x21 | SET_JOINT_DIRECT | `[id, angle(f32)]` | Direct position |
| 0x03 | READ_STATE | - | Request status |
| 0x22 | START_READ_LOOP | - | Start 100Hz streaming |
| 0x23 | STOP_READ_LOOP | - | Stop streaming |
| 0x24 | SINGLE_JOINT_TEST | `[id, start(f32), end(f32), cycles]` | Test joint |
| 0x04 | STOP | - | Emergency stop |
| 0x31 | START_DEMO_RECORDING | `[demo_id]` | Begin recording |
| 0x32 | ADD_WAYPOINT | `[delay_ms(u32)]` | Capture waypoint |
| 0x33 | FINISH_DEMO_RECORDING | - | Save demo |
| 0x30 | PLAY_DEMO | `[demo_id]` | Play demo |
| 0x34 | CLEAR_DEMO | `[demo_id]` | Clear demo |
| 0xF0 | STATUS_REPORT | MCU→Host | Joint state |
| 0xF2 | DEMO_RECORDING_STATUS | MCU→Host | Recording progress |

## Troubleshooting

### No Servos Detected
- Check power supply (6-12V, sufficient current)
- Verify UART connections (GPIO 21 TX, GPIO 25 RX)
- Confirm servo IDs are 1-6
- Set servo baud rate to 1 Mbps using Feetech software

### USB CDC Not Working
- Check USB cable (must support data, not charge-only)
- Try different USB port
- Check `dmesg` for device enumeration
- Verify `CONFIG_USB_CDC_ACM=y` in prj.conf

### CRC Errors
- Check for electrical noise on USB cable
- Try shorter USB cable
- Verify serial port isn't being used by another program

### "Invalid Packet" or "No Response Received"
**Cause:** Debug log messages interfering with packet protocol on console UART.

**Symptoms:**
- `host_test.py` reports "Warning: Received XXXX bytes but no valid packet"
- Hex dump shows ANSI escape codes and log messages mixed with data

**Solution:**
The firmware shares UART0 for both:
- Console debug logs
- Host command/response packets

To fix this, log levels have been reduced to WARNING by default. If you still see interference:

1. **Disable all logging (cleanest)** - Edit `app/prj.conf`:
   ```
   CONFIG_LOG=n
   # CONFIG_LOG_DEFAULT_LEVEL=3
   # CONFIG_LOG_BACKEND_UART=n
   ```

2. **Reduce individual module log levels** - Edit module files:
   ```c
   LOG_MODULE_REGISTER(module_name, LOG_LEVEL_ERR);  // Only errors
   ```

3. **For production:** Use separate UART for commands (requires hardware mod)

**Note:** Heartbeat logs occur every 10 seconds to minimize interference.

### Servo Not Moving
- Check torque enable (emer stop disables torque)
- Verify joint limits aren't exceeded
- Check servo temperature (auto-stops above 80°C)
- Test with Feetech software to confirm servo health

## Next Steps

After successful Phase 3b validation:
1. **Phase 4** - Implement kinematics (FK/IK)
2. **Phase 5** - Implement trajectory planning
3. **Phase 6** - Implement motion controller thread
4. **Phase 7** - Full host communication (add trajectory commands)
5. **Phase 8** - Integration testing

## File Structure

```
octrobot/
├── app/
│   ├── src/
│   │   ├── main.c                     # Main application
│   │   ├── hal/                       # Hardware abstraction
│   │   │   ├── half_duplex_uart.c
│   │   │   └── hal_gpio.c
│   │   ├── drivers/                   # Servo driver
│   │   │   ├── feetech_protocol.c
│   │   │   └── feetech_servo.c
│   │   └── comms/                     # Host communication (NEW - Phase 3b)
│   │       ├── packet_protocol.c
│   │       └── host_comms.c
│   ├── include/
│   │   ├── packet_protocol.h          # NEW - Phase 3b
│   │   └── host_comms.h               # NEW - Phase 3b
│   ├── prj.conf                       # USB CDC enabled
│   └── CMakeLists.txt
├── host_test.py                       # NEW - Python test script
└── docs/
    └── phase3b_validation.md          # This file
```

## Educational Notes

### Why CRC8?
CRC8 with Dallas/Maxim polynomial detects:
- All single-bit errors
- All double-bit errors  
- All burst errors ≤8 bits
- Most longer burst errors

This is sufficient for USB/UART where bit error rates are low (typically <10^-9).

### Why Blocking Parser in Phase 3b?
The Phase 3b parser is intentionally simple (blocking, no threads) to:
- Minimize complexity during hardware validation
- Make debugging easier
- Isolate servo issues from threading issues

Phase 7 will upgrade to a threaded, non-blocking implementation with message queues.

### Why Demo Recording in RAM?
Phase 3b stores demos in RAM (not flash) to:
- Simplify testing
- Avoid flash wear during development
- Defer flash driver complexity to later phase

Flash storage will be added when demos are finalized.

---

**Status**: ✅ Phase 3b Complete - Ready for Phase 4 (Kinematics)
