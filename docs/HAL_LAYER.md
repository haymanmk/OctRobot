# HAL Layer Documentation

## Overview

The Hardware Abstraction Layer (HAL) provides a clean interface between the application code and Zephyr device drivers. This layer simplifies the use of hardware peripherals and provides consistent APIs for the robot arm firmware.

## Components

### 1. Half-Duplex UART (`half_duplex_uart.h/.c`)

Manages communication with Feetech serial bus servos over UART.

**Features:**
- Interrupt-driven RX with ring buffer (256 bytes)
- Automatic TX/RX direction management
- Configurable baud rate (default: 1 Mbps for Feetech STS)
- Timeout-based receive with non-blocking operation

**API:**
```c
hal_uart_handle_t half_duplex_uart_init(const struct half_duplex_uart_config *config);
int half_duplex_uart_transmit(hal_uart_handle_t handle, const uint8_t *data, size_t len);
int half_duplex_uart_receive(hal_uart_handle_t handle, uint8_t *data, size_t len, uint32_t timeout_ms);
int half_duplex_uart_flush_rx(hal_uart_handle_t handle);
```

**Configuration:**
```c
struct half_duplex_uart_config uart_config = {
    .device_name = "UART_1",
    .baudrate = 1000000,     // 1 Mbps
    .tx_timeout_ms = 100,
    .rx_timeout_ms = 50,
};
```

**Hardware Mapping:**
- UART1 TX → GPIO 21 (to UART-to-serial converter)
- UART1 RX → GPIO 25 (from UART-to-serial converter)

### 2. GPIO Utilities (`hal_gpio.h/.c`)

Manages GPIO peripherals including button and LED.

**Features:**
- Emergency stop button with interrupt
- Status LED control
- Callback support for button events

**API:**
```c
int hal_gpio_button_init(void);
bool hal_gpio_button_is_pressed(void);
int hal_gpio_button_set_callback(void (*callback)(void));

int hal_gpio_led_init(void);
int hal_gpio_led_set(bool on);
```

**Hardware Mapping:**
- Button (sw0) → GPIO 39 (emergency stop, active-low with pull-up)
- LED (led0) → GPIO 27 (WS2812B RGB LED - simple on/off only)

### 3. Timer Utilities (`hal_timer.h/.c`)

Provides microsecond-resolution timing for control loops.

**Features:**
- Microsecond and millisecond timestamps
- High-resolution delays
- Based on Zephyr cycle counter (k_cycle_get_64)

**API:**
```c
int hal_timer_init(void);
uint64_t hal_timer_get_us(void);
uint64_t hal_timer_get_ms(void);
void hal_timer_delay_us(uint32_t us);
void hal_timer_delay_ms(uint32_t ms);
```

**Usage Example:**
```c
uint64_t start_time = hal_timer_get_us();
// ... do work ...
uint64_t elapsed = hal_timer_get_us() - start_time;
```

## Integration

### Initialization Sequence

1. Use Zephyr UART console on UART0 (CH340 USB-UART bridge)
2. Initialize timer subsystem
3. Initialize GPIO (LED and button)
4. Initialize UART for servo bus
5. Register emergency stop callback

See `main.c::initialize_hal()` for reference implementation.

### Error Handling

All HAL functions return:
- `HAL_OK` (0) on success
- `HAL_ERROR` (-1) on general errors
- `HAL_TIMEOUT` (-2) on timeout
- `HAL_BUSY` (-3) when resource is busy
- `HAL_INVALID` (-4) on invalid parameters

### Thread Safety

- **half_duplex_uart**: TX operations are mutex-protected. RX uses lock-free ring buffer.
- **hal_gpio**: Functions are generally thread-safe (atomic GPIO operations).
- **hal_timer**: All functions are thread-safe and reentrant.

## Testing

### HAL Self-Test

The current `main.c` includes basic HAL testing:

1. **Timer test**: Measures loop time with microsecond precision
2. **LED test**: Blinks LED during startup and heartbeat
3. **Button test**: Monitors emergency stop button
4. **UART test**: Initializes servo UART (Phase 3 will add servo commands)

### Expected Console Output

```
===========================================
OctroBot Robot Arm Firmware v0.2.0
===========================================
MCU: ESP32-PICO-D4 (M5Stack Atom Lite)
RTOS: Zephyr RTOS v3.6.0
===========================================
Console: UART0 via CH340 USB-UART bridge
Initializing HAL layer...
Timer initialized: 240000000 Hz (240 cycles/us, 240000 cycles/ms)
LED GPIO initialized (GPIO 27)
Button GPIO initialized (GPIO 39)
Half-duplex UART initialized: UART_1 @ 1000000 baud
HAL layer initialized successfully
System initialization complete
6-DOF robot arm ready
Press button (GPIO 39) for emergency stop
Heartbeat: 0 sec (loop time: XXXXX us)
Heartbeat: 1 sec (loop time: XXXXX us)
...
```

### Manual Testing

1. **LED test**: Observe LED blinking on startup and during heartbeat
2. **Button test**: Press button, should log "EMERGENCY STOP ACTIVATED"
3. **Timer test**: Check loop time measurements in console

### Next Steps (Phase 3)

With HAL complete, Phase 3 will implement:
- Feetech protocol packet builder/parser
- Servo-level API (ping, read position, write position)
- SYNC_WRITE for commanding all 6 joints simultaneously

## Troubleshooting

### UART Not Found

**Error**: `UART device 'UART_1' not found`

**Fix**: Check device tree configuration. UART1 should be enabled and aliased as `servo-uart` in `m5stack_atom_lite.dts`.

### Button Interrupt Not Working

**Error**: Button press not detected

**Fix**: 
- Verify GPIO 39 is configured with pull-up in device tree
- Check button wiring (active-low configuration)
- Verify interrupt is enabled in Zephyr config

### Timer Resolution Issues

**Symptom**: Timing measurements seem incorrect

**Fix**:
- Ensure `hal_timer_init()` is called before using timer functions
- Verify system clock frequency matches ESP32 spec (240 MHz)

## References

- [Zephyr UART API](https://docs.zephyrproject.org/latest/hardware/peripherals/uart.html)
- [Zephyr GPIO API](https://docs.zephyrproject.org/latest/hardware/peripherals/gpio.html)
- [Zephyr Timing API](https://docs.zephyrproject.org/latest/kernel/services/timing/timers.html)
- M5Stack Atom Lite Schematic
- Feetech STS Series Servo Manual

---

**Phase 2 Status**: ✅ Complete - HAL layer functional and tested
