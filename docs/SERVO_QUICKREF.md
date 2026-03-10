# Servo Driver Quick Reference

## Initialize Driver

```c
#include "feetech_servo.h"

hal_uart_handle_t uart = /* ... from HAL initialization ... */;
feetech_servo_init(uart);
```

## Common Operations

### Check Connectivity
```c
if (feetech_servo_ping(servo_id) == HAL_OK) {
    LOG_INF("Servo %d: Online", servo_id);
}
```

### Enable/Disable Torque
```c
feetech_servo_set_torque_enable(servo_id, true);   // Enable
feetech_servo_set_torque_enable(servo_id, false);  // Disable (free-move)
```

### Move Single Servo

**Using raw ticks (0-4095):**
```c
feetech_servo_set_goal_position(1, 2048);  // Center position
```

**Using radians:**
```c
feetech_servo_set_goal_angle(1, 0.0f);     // Center position
feetech_servo_set_goal_angle(1, 1.57f);    // π/2 radians (90°)
feetech_servo_set_goal_angle(1, -1.57f);   // -π/2 radians (-90°)
```

**With speed and time:**
```c
feetech_servo_set_goal_time(1, 1000);      // 1 second duration
feetech_servo_set_goal_speed(1, 2000);     // Max speed
feetech_servo_set_goal_angle(1, 0.5f);     // Move to 0.5 radians
```

### Move Multiple Servos (SYNCHRONIZED)

**Critical for coordinated motion!**

```c
// Define servo IDs and target angles
uint8_t ids[] = {1, 2, 3, 4, 5, 6};
float angles[] = {0.0f, 0.5f, -0.5f, 0.0f, 0.3f, -0.3f};

// All servos move together
feetech_servo_sync_write_angles(ids, angles, 6);
```

**Using positions (ticks):**
```c
uint16_t positions[] = {2048, 2500, 1500, 2048, 2300, 1700};
feetech_servo_sync_write_positions(ids, positions, 6);
```

### Read Servo State

**Single value:**
```c
uint16_t pos;
feetech_servo_read_position(1, &pos);

float angle;
feetech_servo_read_angle(1, &angle);

uint8_t temp;
feetech_servo_read_temperature(1, &temp);
```

**Complete state:**
```c
struct feetech_servo_state state;
if (feetech_servo_read_state(1, &state) == HAL_OK) {
    LOG_INF("Pos: %d, Temp: %d°C, Voltage: %d.%dV", 
            state.present_position,
            state.present_temperature,
            state.present_voltage / 10,
            state.present_voltage % 10);
}
```

## Position Conversions

```c
/* Ticks to radians */
float angle = FEETECH_POS_TO_RAD(position);

/* Radians to ticks */
uint16_t position = FEETECH_RAD_TO_POS(angle);
```

**Position Range:**
- Ticks: 0 - 4095
- Center: 2048
- Range: ±π radians (±180°)
- Resolution: 0.088° per tick

## Emergency Stop Pattern

```c
void emergency_stop(void) {
    uint8_t ids[] = {1, 2, 3, 4, 5, 6};
    
    for (uint8_t i = 0; i < 6; i++) {
        feetech_servo_set_torque_enable(ids[i], false);
    }
}
```

## Safety Monitoring

```c
void monitor_servos(void) {
    for (uint8_t id = 1; id <= 6; id++) {
        struct feetech_servo_state state;
        
        if (feetech_servo_read_state(id, &state) == HAL_OK) {
            /* Check temperature */
            if (state.present_temperature > 65) {
                LOG_WRN("Servo %d hot: %d°C", id, state.present_temperature);
            }
            
            /* Check voltage */
            if (state.present_voltage < 60 || state.present_voltage > 120) {
                LOG_ERR("Servo %d voltage: %d.%dV", id,
                        state.present_voltage / 10,
                        state.present_voltage % 10);
            }
            
            /* Check load (collision detection) */
            if (state.present_load > 800) {
                LOG_WRN("Servo %d high load: %d", id, state.present_load);
            }
        }
    }
}
```

## Typical Control Loop

```c
void control_loop(void) {
    uint8_t ids[] = {1, 2, 3, 4, 5, 6};
    uint16_t current_positions[6];
    
    while (1) {
        /* Read all positions */
        feetech_servo_read_multi_positions(ids, current_positions, 6);
        
        /* Compute new positions (from kinematics/trajectory planner) */
        float target_angles[6];
        compute_next_trajectory_point(target_angles);
        
        /* Send synchronized command */
        feetech_servo_sync_write_angles(ids, target_angles, 6);
        
        /* Wait for next control tick (e.g., 10ms) */
        k_msleep(10);
    }
}
```

## Error Codes

```c
int ret = feetech_servo_set_goal_position(1, 2048);

if (ret == HAL_OK) {
    // Success
} else if (ret == HAL_TIMEOUT) {
    LOG_ERR("Servo not responding");
} else if (ret == HAL_INVALID) {
    LOG_ERR("Invalid parameter");
} else {
    LOG_ERR("Communication error: %d", ret);
}
```

## Performance Tips

1. **Always use SYNC_WRITE for robot motion**
   - 6ms for 6 servos vs 36ms for individual writes

2. **Batch status reads during idle time**
   - Don't read status in real-time control loop
   - Read every 100ms or slower

3. **Pre-compute trajectories**
   - Don't compute angles in control loop
   - Use pre-computed trajectory points

4. **Cache servo configurations**
   - Don't re-write acceleration/speed every tick
   - Set once during initialization

## Common Pitfalls

❌ **DON'T:**
```c
// Individual writes - slow!
for (int i = 0; i < 6; i++) {
    feetech_servo_set_goal_angle(ids[i], angles[i]);
}
```

✅ **DO:**
```c
// Synchronized write - fast!
feetech_servo_sync_write_angles(ids, angles, 6);
```

---

❌ **DON'T:**
```c
// Reading in control loop - too slow!
while (1) {
    feetech_servo_read_temperature(1, &temp);
    feetech_servo_set_goal_angle(1, angle);
    k_msleep(10);
}
```

✅ **DO:**
```c
// Status monitoring in separate thread
while (1) {
    feetech_servo_read_temperature(1, &temp);
    k_msleep(100);  // Much slower rate
}
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No servo response | Check power, UART wiring, servo ID |
| Checksum errors | Check baud rate (1 Mbps), check wiring quality |
| Timeout | Increase timeout, check servo power, verify ID |
| Servo doesn't move | Enable torque first, check position limits |
| Voltage warnings | Check power supply, battery charge |
| High temperature | Reduce load, improve cooling, check for binding |

---

**See [SERVO_DRIVER.md](SERVO_DRIVER.md) for complete documentation**
