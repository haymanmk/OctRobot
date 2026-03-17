# HAL Quick Reference

## Initialization Order

```c
hal_gpio_button_init();
hal_gpio_button_set_callback(my_callback);

struct half_duplex_uart_config cfg = {    // 2. UART last
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
// Measure elapsed time
int64_t start = k_uptime_get(); // milliseconds
// ... do work ...
int64_t elapsed_ms = k_uptime_get() - start;

// Fixed-rate loop (yield-friendly)
while (1) {
    int64_t loop_start = k_uptime_get();

    // ... control loop work ...

    int64_t work_ms = k_uptime_get() - loop_start;
    if (work_ms < 1) {
        k_msleep(1 - work_ms); // yield remaining time
    }
}
```

### GPIO Emergency Stop

```c
// Emergency stop callback
void emergency_stop_handler(void) {
    LOG_ERR("Emergency stop triggered");
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

## Performance Tips

1. **Batch UART operations**: Send all servo commands in SYNC_WRITE, not individual writes

2. **Pre-allocate buffers**: Avoid malloc in ISR or real-time loops

3. **Check RX available before blocking receive**:
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

## Hardware Connections

```
M5Stack Atom Lite:
┌─────────────────┐
│    ESP32-PICO   │
│                 │
│  GPIO 21 ──────┼─→ UART TX (to serial converter)
│  GPIO 25 ──────┼─→ UART RX (from serial converter)
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
