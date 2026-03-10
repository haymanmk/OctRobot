# HAL Quick Reference

## Initialization Order

```c
hal_timer_init();                          // 1. Timer first
hal_gpio_led_init();                       // 2. GPIO
hal_gpio_button_init();
hal_gpio_button_set_callback(my_callback);

struct half_duplex_uart_config cfg = {    // 3. UART last
    .device_name = "UART_1",
    .baudrate = 1000000,
    .tx_timeout_ms = 100,
    .rx_timeout_ms = 50,
};
hal_uart_handle_t uart = half_duplex_uart_init(&cfg);
```

## Common Patterns

### UART Communication (Feetech Servo Protocol)

```c
// Send command packet
uint8_t cmd_packet[] = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x2A, 0x64, 0xCA};
int ret = half_duplex_uart_transmit(uart, cmd_packet, sizeof(cmd_packet));

// Wait for response
uint8_t response[8];
ret = half_duplex_uart_receive(uart, response, 8, 50); // 50ms timeout

// Flush old data
half_duplex_uart_flush_rx(uart);
```

### Timing Control Loops

```c
// Option 1: Measure loop time
uint64_t start = hal_timer_get_us();
// ... do work ...
uint64_t elapsed = hal_timer_get_us() - start;

// Option 2: Fixed-rate loop
while (1) {
    uint64_t loop_start = hal_timer_get_us();
    
    // ... control loop work ...
    
    uint64_t work_time = hal_timer_get_us() - loop_start;
    if (work_time < 1000) { // 1ms loop
        hal_timer_delay_us(1000 - work_time);
    }
}
```

### GPIO Status Indication

```c
// Heartbeat LED
static bool led_state = false;
void heartbeat_tick(void) {
    led_state = !led_state;
    hal_gpio_led_set(led_state);
}

// Emergency stop
void emergency_stop_handler(void) {
    // Blink LED rapidly
    for (int i = 0; i < 10; i++) {
        hal_gpio_led_set(true);
        hal_timer_delay_ms(100);
        hal_gpio_led_set(false);
        hal_timer_delay_ms(100);
    }
}
```

## Error Handling

```c
int ret = hal_some_function();

if (ret == HAL_OK) {
    // Success
} else if (ret == HAL_TIMEOUT) {
    LOG_WRN("Operation timed out");
} else if (ret == HAL_INVALID) {
    LOG_ERR("Invalid parameters");
} else {
    LOG_ERR("Operation failed: %d", ret);
}
```

## Thread Safety Notes

- **UART TX**: Mutex-protected, safe from multiple threads
- **UART RX**: Lock-free ring buffer, safe for ISR + reads
- **GPIO**: Atomic hardware operations, generally safe
- **Timer**: All functions are reentrant and thread-safe

## Performance Tips

1. **Avoid `hal_timer_delay_us()` in threads**: Use `k_usleep()` instead to yield

2. **Batch UART operations**: Send all servo commands in SYNC_WRITE, not individual writes

3. **Pre-allocate buffers**: Avoid malloc in ISR or real-time loops

4. **Check RX available before blocking receive**:
   ```c
   int avail = half_duplex_uart_rx_available(uart);
   if (avail >= expected_bytes) {
       half_duplex_uart_receive(uart, buffer, expected_bytes, 0);
   }
   ```

## Common Issues

### UART RX Buffer Overflow
**Symptom**: Missing servo responses  
**Fix**: Increase `RX_RING_BUFFER_SIZE` or call `half_duplex_uart_receive()` more frequently

### Timer Wrapping
**Symptom**: Incorrect timing after ~71 minutes (32-bit cycle counter)  
**Fix**: Use 64-bit `k_cycle_get_64()` (already done) or handle wrap-around

### Emergency Stop Not Firing
**Symptom**: Button press not detected  
**Fix**: Check pull-up configuration in DTS, verify GPIO 39 wiring

### LED Not Visible
**Note**: WS2812B needs special RGB driver for colors. Current implementation only controls power pin (limited visibility).  
**Enhancement**: Add WS2812 LED strip driver in future phase

## Hardware Connections

```
M5Stack Atom Lite:
┌─────────────────┐
│    ESP32-PICO   │
│                 │
│  GPIO 26 ──────┼─→ UART TX (Yellow - Grove)
│  GPIO 32 ──────┼─→ UART RX (White - Grove)
│  GPIO 27 ──────┼─→ WS2812B LED
│  GPIO 39 ──────┼─→ Button (Pull-up, active-low)
│                 │
│  USB-C ────────┼─→ Console & Power
└─────────────────┘
```

## Logging

```c
LOG_MODULE_REGISTER(my_module, LOG_LEVEL_DBG);

LOG_DBG("Debug: value=%d", value);
LOG_INF("Info: system ready");
LOG_WRN("Warning: servo timeout");
LOG_ERR("Error: communication failed");
```

---

**Quick Start**: See `app/src/main.c::initialize_hal()` for reference implementation
