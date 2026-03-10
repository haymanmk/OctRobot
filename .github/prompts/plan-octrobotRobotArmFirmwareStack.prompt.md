# Plan: OctroBot Robot Arm Firmware Stack

## Platform Summary
- **MCU**: M5Stack Atom Lite (ESP32-PICO-D4, dual-core 240MHz, 520KB SRAM, 4MB Flash)
- **RTOS**: Zephyr RTOS
- **Servo**: Feetech Serial Bus Servos (half-duplex UART, SCS/STS protocol)
- **DOF**: 6-DOF robot arm
- **Language**: C (C17)
- **Rationale for C over C++**: Zephyr-native (no `CONFIG_CPP` needed), slightly leaner binary, no C++ runtime overhead, simpler Zephyr API integration (no `extern "C"` guards), better fit for a solo-maintained embedded codebase. C modules with opaque structs + function prefixes replace C++ classes with zero overhead.
- **Host connection**: USB-CDC serial (built-in on Atom Lite via USB-C)

---

## TL;DR

Build a layered C firmware stack on Zephyr RTOS targeting ESP32 (M5Stack Atom Lite). Layers: HAL → Servo Driver → Kinematics (FK/IK) → Trajectory Planner → Motion Controller → Host Command Interface. A clean host protocol over USB-CDC enables future ROS2 bridge without touching firmware.

---

## Phase 1 — Project Scaffold & Toolchain

1. Initialize a Zephyr west workspace (`west init / west update`)
2. Create custom board definition for M5Stack Atom Lite (based on `esp32_devkitc_wroom`, adjust GPIO/flash/pins in DTS overlay)
3. Establish project folder structure:
   ```
   octrobot/
   ├── app/
   │   ├── src/
   │   │   ├── main.c
   │   │   ├── hal/           # UART, GPIO, timer wrappers
   │   │   ├── drivers/       # Feetech servo driver
   │   │   ├── kinematics/    # FK, IK
   │   │   ├── trajectory/    # Trajectory planner
   │   │   ├── controller/    # Motion controller (joint PID)
   │   │   └── comms/         # Host protocol (USB-CDC)
   │   ├── include/
   │   ├── CMakeLists.txt
   │   └── prj.conf
   ├── boards/
   │   └── m5stack_atom_lite/
   │       ├── board.cmake
   │       ├── Kconfig.board
   │       └── m5stack_atom_lite.dts
   └── west.yml
   ```
4. Configure `prj.conf`: enable `CONFIG_UART_INTERRUPT_DRIVEN`, `CONFIG_USB_CDC_ACM` (no C++ config needed)
5. Verify a "hello world" builds and flashes (USB-CDC log output visible on host)

---

## Phase 2 — HAL Layer

6. Wrap Zephyr UART device API into a `half_duplex_uart` C module:
   - Handles TX/RX direction switching (GPIO direction pin or auto via single-wire UART)
   - Configurable baud rate (Feetech STS typically 1 Mbps)
   - Non-blocking TX with ISR-driven RX into a ring buffer
7. Map GPIO assignment for Atom Lite:
   - `UART1` (GPIO 26 TX / GPIO 32 RX) → servo bus (Grove port)
   - `USB_CDC` (built-in) → host comms
   - Reserve GPIO 39 (button) for emergency stop
8. Implement a microsecond-resolution timer utility using Zephyr `k_cycle_get_32()` for control loop timing

---

## Phase 3 — Feetech Serial Bus Servo Driver

9. Implement `feetech_bus` C module:
   - Packet builder/parser for Feetech SCS/STS protocol:
     `0xFF 0xFF [ID] [LEN] [INSTR] [PARAMS...] [CHECKSUM]`
   - Instructions: PING, READ, WRITE, SYNC_WRITE, REG_WRITE, ACTION
   - Checksum: `~(ID + LEN + INSTR + params) & 0xFF`
10. Implement `feetech_servo` module per joint:
    - `feetech_servo_set_goal_position(servo, pos)` — maps radians to servo ticks
    - `feetech_servo_set_goal_velocity(servo, vel)`
    - `feetech_servo_read_present_position(servo)` → returns radians
    - `feetech_servo_read_load()`, `feetech_servo_read_temperature()` for safety monitoring
11. Implement `feetech_bus_sync_write()` for simultaneously commanding all 6 joints in a single bus transaction (critical for coordinated motion)
12. Unit test: ping all 6 servo IDs, read/write position on each joint individually

---

## Phase 4 — Kinematics Module

13. Define robot geometry:
    - Store DH parameters (a, d, alpha, theta_offset) for all 6 joints as `const` arrays
    - Joint limits (min/max radians) per joint
14. Implement `forward_kinematics_compute(joint_angles[6], out_transform)`:
    - Chain 6 homogeneous transform matrices using DH convention
    - Returns end-effector pose (position + rotation matrix)
15. Implement `inverse_kinematics` — recommended approach: **analytical IK for 6-DOF spherical wrist** (joints 4-5-6 intersect at a point):
    - Decouple: solve wrist center position (joints 1-3 geometrically), then solve wrist orientation (joints 4-6 analytically)
    - Return up to 8 solution candidates; select closest to current configuration
    - Return error code if target is unreachable
16. Math dependencies: implement lightweight `mat4x4`, `vec3`, `quat` in `kinematics/math.h/.c` — **no external linear algebra library** to keep the footprint minimal
17. Unit test kinematics offline on host PC (same C source files, compiled with gcc, no Zephyr dependency)

---

## Phase 5 — Trajectory Planner

18. Implement `trajectory_point_t` struct: `{joint_angles[6], timestamp_ms}`
19. Implement `joint_trajectory` module — trapezoidal velocity profile in joint space:
    - Input: start angles, goal angles, max velocity, max acceleration (per joint)
    - Output: pre-computed array of `trajectory_point_t` at fixed 10ms interpolation steps
    - Enforces joint velocity and acceleration limits
20. Implement `cartesian_trajectory` (optional, Phase 5b):
    - Linear interpolation in Cartesian space, IK solved at each waypoint
    - Useful for straight-line end-effector paths
21. Trajectory buffer: ring buffer stored in RAM, capacity ~200 points (~2s of motion at 10ms step)

---

## Phase 6 — Motion Controller

22. Implement `motion_controller` Zephyr thread (highest priority, 1ms period via `k_timer`):
    - Reads current joint positions via `SyncRead` (or staggered individual reads)
    - Fetches next `trajectory_point_t` from trajectory buffer
    - Computes joint position error
    - Sends `SyncWrite` goal positions to servo bus
    - Feetech STS servos have internal PID — outer loop sends position setpoints only (no torque control needed unless servos support it)
23. Implement emergency stop: GPIO 39 (button) ISR disables motion, sends hold-position command to all servos
24. Watchdog: if host comms silent for >500ms during teleoperation, auto-stop

---

## Phase 7 — Host Command Protocol (USB-CDC)

25. Define binary packet protocol over USB-CDC:
    ```
    [0xAA] [CMD] [PAYLOAD_LEN(1B)] [PAYLOAD...] [CRC8]
    ```
    Commands:
    - `0x01` MOVE_JOINTS — target joint angles (6× float32) + duration_ms
    - `0x02` MOVE_CARTESIAN — target pose (x,y,z,rx,ry,rz) + duration_ms
    - `0x03` READ_STATE — request current joint angles + end-effector pose
    - `0x04` STOP — immediate halt
    - `0x05` HOME — move to defined home position
    - `0x10` SET_PARAMS — update speed/accel limits
    - `0xF0` STATUS_REPORT — MCU→host: joint angles, temperatures, error flags
26. Implement `host_comms` Zephyr thread (lower priority):
    - Parses incoming packets, dispatches to controller/kinematics
    - Sends periodic `STATUS_REPORT` at 50Hz
27. CRC8 for packet integrity (Dallas/Maxim polynomial)

---

## Phase 8 — Integration & Testing

28. Integration test: send `MOVE_JOINTS` from host Python script → verify all 6 joints move to target
29. Integration test: send `MOVE_CARTESIAN` → verify IK solution is valid and arm reaches pose
30. Tune trajectory parameters (max vel/accel) for smooth, safe motion
31. Stress test: continuous motion commands for 30 minutes, monitor servo temperatures

---

## Future: ROS2 Bridge (Host Side Only, No Firmware Changes)

- Write a ROS2 node (`octrobot_driver`) that:
  - Opens the USB-CDC serial port
  - Subscribes to `/joint_trajectory_controller/command` (standard `trajectory_msgs`)
  - Translates to `MOVE_JOINTS` packets → MCU
  - Publishes `/joint_states` from `STATUS_REPORT` packets
- MoveIt2 integration via `ros2_control` hardware interface
- RViz2 visualization using URDF built from DH parameters

---

## Key Files (at completion)

- `app/src/hal/half_duplex_uart.h/.c`
- `app/src/drivers/feetech_bus.h/.c`
- `app/src/drivers/feetech_servo.h/.c`
- `app/src/kinematics/math.h/.c`
- `app/src/kinematics/forward_kinematics.h/.c`
- `app/src/kinematics/inverse_kinematics.h/.c`
- `app/src/trajectory/joint_trajectory.h/.c`
- `app/src/controller/motion_controller.h/.c`
- `app/src/comms/host_comms.h/.c`
- `boards/m5stack_atom_lite/m5stack_atom_lite.dts`
- `app/prj.conf`
- `app/CMakeLists.txt`

---

## Decisions & Scope

- **In scope**: firmware only — no simulation, no ROS2 in this phase
- **Out of scope**: dynamics (gravity compensation, torque control) — deferred to later phase
- **Servo inner loop**: Feetech STS series handles PID internally; outer loop sends position targets only
- **IK method**: analytical spherical-wrist decoupling (fast, deterministic, no iteration — suitable for MCU)
- **Math library**: hand-rolled minimal math (no Eigen), keeping binary size small under Zephyr
- **Kinematics test**: same C source files compile on host PC for offline unit testing
