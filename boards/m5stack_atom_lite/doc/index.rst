# M5Stack Atom Lite Board Definition

## Overview

The M5Stack Atom Lite is a compact ESP32-based development board featuring:

- **SoC**: ESP32-PICO-D4
  - Dual-core Xtensa LX6 @ 240MHz
  - 520KB SRAM
  - 4MB Flash (integrated)
- **Connectivity**: 
  - USB-C port (CH340C USB-UART bridge)
  - Wi-Fi 802.11 b/g/n
  - Bluetooth v4.2 BR/EDR and BLE
- **I/O**:
  - Custom UART on GPIO 21 (TX), GPIO 25 (RX) for servo bus
  - WS2812B RGB LED (GPIO 27)
  - Button (GPIO 39)
- **Power**: 5V via USB-C

## Pin Mapping

| Function | GPIO | Notes |
|----------|------|-------|
| UART1 TX (Servo Bus) | 21 | To UART-to-serial converter |
| UART1 RX (Servo Bus) | 25 | From UART-to-serial converter |
| WS2812B RGB LED | 27 | Status indicator |
| User Button | 39 | Emergency stop, active-low with pull-up |
| USB Console | UART0 | CH340 USB-UART virtual COM port |

## Programming

Flash using esptool.py:

```bash
west build -b m5stack_atom_lite/esp32/procpu app
west flash
```

## References

- [M5Stack Atom Lite Documentation](https://docs.m5stack.com/en/core/atom_lite)
- [ESP32-PICO-D4 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-pico-d4_datasheet_en.pdf)
