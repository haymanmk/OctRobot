# Feetech Servo Driver Documentation

## Overview

The Feetech servo driver provides a complete implementation of the SCS/STS protocol for controlling Feetech serial bus servos. It consists of two layers:

1. **Protocol Layer** (`feetech_protocol.h/.c`) - Low-level packet building/parsing
2. **Servo API** (`feetech_servo.h/.c`) - High-level servo control functions

## Protocol Layer

### Packet Format

```
[0xFF] [0xFF] [ID] [LENGTH] [INSTRUCTION] [PARAMS...] [CHECKSUM]
```

- **Header**: Two 0xFF bytes
- **ID**: Servo ID (1-253) or 0xFE for broadcast
- **LENGTH**: Instruction + Parameters + Checksum (1 byte)
- **INSTRUCTION**: Command byte
- **PARAMS**: Variable-length parameter data
- **CHECKSUM**: `~(ID + LENGTH + INSTRUCTION + PARAMS) & 0xFF`

### Instructions

| Instruction | Code | Description |
|-------------|------|-------------|
| PING | 0x01 | Check if servo is responding |
| READ | 0x02 | Read data from servo registers |
| WRITE | 0x03 | Write data to servo registers |
| REG_WRITE | 0x04 | Register write (execute with ACTION) |
| ACTION | 0x05 | Execute registered writes |
| SYNC_WRITE | 0x83 | Write to multiple servos simultaneously |

### Key Functions

```c
/* Calculate checksum */
uint8_t feetech_calculate_checksum(uint8_t id, uint8_t length, 
                                    uint8_t instruction,
                                    const uint8_t *params, 
                                    uint8_t param_len);

/* Build packet for transmission */
int feetech_build_packet(const struct feetech_packet *packet, 
                         uint8_t *buffer, size_t buffer_size);

/* Parse received response */
int feetech_parse_packet(const uint8_t *buffer, size_t buffer_len,
                         struct feetech_packet *packet);

/* Protocol operations */
int feetech_ping(hal_uart_handle_t uart, uint8_t id);
int feetech_read(hal_uart_handle_t uart, uint8_t id, uint8_t reg_addr,
                 uint8_t length, uint8_t *data);
int feetech_write(hal_uart_handle_t uart, uint8_t id, uint8_t reg_addr,
                  const uint8_t *data, uint8_t length);
int feetech_sync_write(hal_uart_handle_t uart, uint8_t reg_addr, 
                       uint8_t data_len, const uint8_t *servo_ids,
                       const uint8_t *servo_data, uint8_t servo_count);
```

## Servo API

### Initialization

```c
/* Initialize servo driver with UART handle */
int feetech_servo_init(hal_uart_handle_t uart);

/* Test servo connectivity */
int feetech_servo_ping(uint8_t id);
```

### Position Control

```c
/* Set goal position (raw servo ticks: 0-4095) */
int feetech_servo_set_goal_position(uint8_t id, uint16_t position);

/* Set goal position (radians) */
int feetech_servo_set_goal_angle(uint8_t id, float angle);

/* Read current position */
int feetech_servo_read_position(uint8_t id, uint16_t *position);
int feetech_servo_read_angle(uint8_t id, float *angle);
```

**Position Range:**
- Raw ticks: 0 - 4095
- Center: 2048
- Resolution: ~0.088° per tick (360° / 4096)
- Conversion: `rad = (pos - 2048) * (2π / 4096)`

### Motion Parameters

```c
/* Set movement speed */
int feetech_servo_set_goal_speed(uint8_t id, uint16_t speed);

/* Set movement time (trajectory duration) */
int feetech_servo_set_goal_time(uint8_t id, uint16_t time_ms);

/* Set acceleration */
int feetech_servo_set_acceleration(uint8_t id, uint8_t acceleration);

/* Enable/disable torque */
int feetech_servo_set_torque_enable(uint8_t id, bool enable);
```

### Status Reading

```c
/* Read individual status values */
int feetech_servo_read_load(uint8_t id, uint16_t *load);
int feetech_servo_read_temperature(uint8_t id, uint8_t *temperature);
int feetech_servo_read_voltage(uint8_t id, uint8_t *voltage);

/* Read complete servo state */
struct feetech_servo_state {
    uint16_t present_position;
    uint16_t present_speed;
    uint16_t present_load;
    uint8_t present_voltage;    /* 0.1V units (e.g., 120 = 12.0V) */
    uint8_t present_temperature; /* °C */
    bool is_moving;
    uint8_t error;
};

int feetech_servo_read_state(uint8_t id, struct feetech_servo_state *state);
```

### Synchronized Operations

**Critical for coordinated robot motion!**

```c
/* Sync write positions to all joints (raw ticks) */
int feetech_servo_sync_write_positions(const uint8_t *ids, 
                                        const uint16_t *positions,
                                        uint8_t count);

/* Sync write positions (radians) */
int feetech_servo_sync_write_angles(const uint8_t *ids, 
                                     const float *angles,
                                     uint8_t count);

/* Read multiple servo positions */
int feetech_servo_read_multi_positions(const uint8_t *ids, 
                                        uint16_t *positions,
                                        uint8_t count);
```

**Why SYNC_WRITE is important:**
- Sends goal positions to all 6 servos in a single packet
- Ensures all servos start moving at the same time
- Critical for smooth, coordinated robot arm motion
- Much faster than individual writes (6ms vs 36ms for 6 servos)

## Common Register Addresses

| Register | Address | Size | Description |
|----------|---------|------|-------------|
| TORQUE_ENABLE | 0x28 | 1 | Enable (1) or disable (0) torque |
| ACCELERATION | 0x29 | 1 | Acceleration value (0-254) |
| GOAL_POSITION | 0x2A | 2 | Target position (little-endian) |
| GOAL_TIME | 0x2C | 2 | Movement duration (ms) |
| GOAL_SPEED | 0x2E | 2 | Target speed |
| PRESENT_POSITION | 0x38 | 2 | Current position |
| PRESENT_SPEED | 0x3A | 2 | Current speed |
| PRESENT_LOAD | 0x3C | 2 | Current load (torque) |
| PRESENT_VOLTAGE | 0x3E | 1 | Voltage (0.1V units) |
| PRESENT_TEMPERATURE | 0x3F | 1 | Temperature (°C) |
| MOVING | 0x42 | 1 | Movement status |

## Usage Examples

### Example 1: Ping All Servos

```c
uint8_t servo_ids[] = {1, 2, 3, 4, 5, 6};
for (uint8_t i = 0; i < 6; i++) {
    if (feetech_servo_ping(servo_ids[i]) == HAL_OK) {
        LOG_INF("Servo %d: OK", servo_ids[i]);
    } else {
        LOG_WRN("Servo %d: No response", servo_ids[i]);
    }
}
```

### Example 2: Move Single Servo

```c
/* Move servo 1 to center position over 1 second */
feetech_servo_set_torque_enable(1, true);
feetech_servo_set_goal_time(1, 1000);
feetech_servo_set_goal_position(1, FEETECH_POS_CENTER);

/* Or using radians */
feetech_servo_set_goal_angle(1, 0.0f); /* 0 radians = center */
```

### Example 3: Coordinated Multi-Joint Motion

```c
uint8_t ids[] = {1, 2, 3, 4, 5, 6};
float angles[] = {0.0f, 0.5f, -0.5f, 0.0f, 0.3f, -0.3f};

/* All servos receive commands simultaneously */
feetech_servo_sync_write_angles(ids, angles, 6);
```

### Example 4: Read Servo Status

```c
struct feetech_servo_state state;
if (feetech_servo_read_state(1, &state) == HAL_OK) {
    LOG_INF("Servo 1:");
    LOG_INF("  Position: %d (%.2f rad)", 
            state.present_position,
            FEETECH_POS_TO_RAD(state.present_position));
    LOG_INF("  Temperature: %d°C", state.present_temperature);
    LOG_INF("  Voltage: %d.%dV", 
            state.present_voltage / 10,
            state.present_voltage % 10);
    LOG_INF("  Load: %d", state.present_load);
}
```

### Example 5: Emergency Stop

```c
void emergency_stop(void) {
    uint8_t servo_ids[] = {1, 2, 3, 4, 5, 6};
    
    /* Disable torque on all servos */
    for (uint8_t i = 0; i < 6; i++) {
        feetech_servo_set_torque_enable(servo_ids[i], false);
    }
}
```

## Testing

### Servo Test Function

The `main.c` includes a comprehensive servo test function that:

1. Pings all 6 servos to check connectivity
2. Reports how many servos are detected
3. Reads initial positions from all servos
4. Reads status (temperature, voltage, load) from all servos

**Expected Output:**
```
===========================================
Testing Servo Communication
===========================================
Pinging servos...
  Servo 1: OK
  Servo 2: OK
  Servo 3: OK
  Servo 4: OK
  Servo 5: OK
  Servo 6: OK
Detected 6/6 servos
Reading initial servo positions...
  Servo 1: position=2048 (0.00 rad)
  Servo 2: position=2100 (0.08 rad)
  ...
Reading servo status...
  Servo 1: temp=35°C, voltage=12.0V, load=0
  ...
===========================================
Servo test complete
===========================================
```

### Hardware Setup for Testing

1. **Power**: Connect 6-12V power supply to servos
2. **Communication**: Connect UART to servo bus via UART-to-serial converter
   - GPIO 21 (TX) → Converter TX
   - GPIO 25 (RX) → Converter RX
   - GND → Common ground
3. **Servo IDs**: Configure servos with IDs 1-6
4. **Baud Rate**: Set servos to 1 Mbps (1000000 baud)

### Troubleshooting

**No servos detected:**
- Check power supply voltage (6-12V)
- Verify UART wiring (TX/RX may be swapped)
- Confirm servo IDs are set to 1-6
- Check baud rate matches (1 Mbps default)
- Test with single servo first

**Checksum errors:**
- Interference on servo bus (check wiring quality)
- Baud rate mismatch
- RX buffer overflow (increase delay between commands)

**Timeout errors:**
- Servo may be in error state (check LED)
- Wrong servo ID
- Servo not powered
- UART RX not connected

## Performance Notes

### Communication Timing

- **Ping**: ~5ms per servo
- **Single write**: ~6ms per servo
- **Sync write (6 servos)**: ~6ms total (much faster!)
- **Read position**: ~8ms per servo
- **Read full state**: ~40ms per servo (6 register reads)

### Best Practices

1. **Use SYNC_WRITE for coordinated motion** - Always use sync write when commanding multiple joints
2. **Batch reads** - Group status reads during idle time, not in control loop
3. **Monitor temperature** - Log warnings above 65°C, stop above 75°C
4. **Check voltage** - Warn if voltage drops below 6V or exceeds 12V
5. **Limit load** - Monitor load values to detect collisions or binding

## Error Handling

### Error Flags (in response packets)

```c
#define FEETECH_ERROR_VOLTAGE       (1 << 0)  /* Input voltage error */
#define FEETECH_ERROR_ANGLE_LIMIT   (1 << 1)  /* Angle limit error */
#define FEETECH_ERROR_OVERHEAT      (1 << 2)  /* Overheating */
#define FEETECH_ERROR_RANGE         (1 << 3)  /* Out of range */
#define FEETECH_ERROR_CHECKSUM      (1 << 4)  /* Checksum error */
#define FEETECH_ERROR_OVERLOAD      (1 << 5)  /* Overload */
#define FEETECH_ERROR_INSTRUCTION   (1 << 6)  /* Invalid instruction */
```

### Return Codes

All servo functions return:
- `HAL_OK` (0) on success
- `HAL_ERROR` (-1) on communication errors
- `HAL_TIMEOUT` (-2) on timeout
- `HAL_INVALID` (-4) on invalid parameters

## Next Steps (Phase 4)

With the servo driver complete, Phase 4 will implement:

- Forward kinematics (joint angles → end-effector pose)
- Inverse kinematics (end-effector pose → joint angles)
- DH parameter definitions for 6-DOF arm
- Joint limit enforcement
- Lightweight matrix math library

---

**Phase 3 Status**: ✅ Complete - Servo driver tested and functional
