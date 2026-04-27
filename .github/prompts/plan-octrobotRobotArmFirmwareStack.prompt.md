# Plan: OctroBot Robot Arm Firmware Stack

## Platform Summary
- **MCU**: M5Stack Atom Lite (ESP32-PICO-D4, dual-core 240MHz, 520KB SRAM, 4MB Flash)
- **RTOS**: Zephyr RTOS
- **Servo**: Feetech Serial Bus Servos (half-duplex UART, SCS/STS protocol)
- **DOF**: 6-DOF robot arm
- **Language**: C (C17)
- **Rationale for C over C++**: Zephyr-native (no `CONFIG_CPP` needed), slightly leaner binary, no C++ runtime overhead, simpler Zephyr API integration (no `extern "C"` guards), better fit for a solo-maintained embedded codebase. C modules with opaque structs + function prefixes replace C++ classes with zero overhead.
- **Host connection**: UART serial via CH340 USB-UART bridge

---

## TL;DR

Build a layered C firmware stack on Zephyr RTOS targeting dual-core ESP32 (M5Stack Atom Lite). Layers: HAL → Servo Driver → Kinematics (FK/IK using POE) → Trajectory Planner → Motion Controller → Host Command Interface. Dual-core architecture: PRO_CPU (Core 0) handles servo I/O + safety; APP_CPU (Core 1) handles FK/IK computation. SMP with selective thread pinning provides isolation + load balancing. A clean host protocol over UART serial enables future ROS2 bridge without touching firmware.

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
    │   │   └── comms/         # Host protocol (UART serial)
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
4. Configure `prj.conf`: enable `CONFIG_UART_INTERRUPT_DRIVEN`, `CONFIG_SMP=y` (dual-core), `CONFIG_SCHED_CPU_MASK=y` (thread pinning), `CONFIG_FPU=y` (hardware floating point), `CONFIG_MSGQ=y` (message queues for IPC)
5. Verify a "hello world" builds and flashes (UART log output visible on host)

---

## Phase 2 — HAL Layer

6. Wrap Zephyr UART device API into a `half_duplex_uart` C module:
   - Handles TX/RX direction switching (GPIO direction pin or auto via single-wire UART)
   - Configurable baud rate (Feetech STS typically 1 Mbps)
   - Non-blocking TX with ISR-driven RX into a ring buffer
7. Map GPIO assignment for Atom Lite:
   - `UART1` (GPIO 21 TX / GPIO 25 RX) → servo bus (via UART-to-serial converter)
    - `UART0` (via CH340 bridge) → host comms
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

## Phase 3b — Early Manual Control Validation (Before Kinematics)

**Purpose**: Test servo control, jogging, and demo recording early to validate hardware/driver before investing time in kinematics/trajectory. This provides confidence that joint-level control works correctly.

13. Implement minimal USB-CDC packet parser (subset of Phase 7):
    - Basic packet format: `[0xAA] [CMD] [PAYLOAD_LEN] [PAYLOAD...] [CRC8]`
    - Implement only the commands below (defer trajectory and Cartesian commands)
    - Simple blocking parser (no threading yet — sufficient for early testing)
    
14. Implement manual control commands:
    - `0x20` JOG_JOINT — `[joint_id, direction, step_size_deg]` — incremental movement
    - `0x21` SET_JOINT_DIRECT — `[joint_id, angle_rad]` — direct position command (bypass trajectory)
    - `0x03` READ_STATE — return current joint angles only (6× float32)
    - `0x22` START_READ_LOOP — stream joint positions at 100Hz
    - `0x23` STOP_READ_LOOP — stop streaming
    - `0x24` SINGLE_JOINT_TEST — `[joint_id, start, end, cycles]` — oscillate joint
    - `0x04` STOP — emergency stop (hold current position)
    
15. Implement demo recording for early validation:
    - `0x31` START_DEMO_RECORDING — `[demo_id]`
    - `0x32` ADD_WAYPOINT — `[delay_ms]` — capture current joint angles
    - `0x33` FINISH_DEMO_RECORDING — save to flash
    - `0x30` PLAY_DEMO — `[demo_id]` — playback recorded waypoints (simple point-to-point, no trajectory smoothing yet)
    - `0x34` CLEAR_DEMO — `[demo_id]` — erase demo
    
16. Write simple host Python script for testing:
    - Open USB-CDC port, send/receive packets
    - Interactive CLI: `jog 0 +5`, `set 1 0.5`, `record_demo`, `play_demo`
    
17. Validation tests:
    - **Test 1**: Ping all 6 servos, verify all respond
    - **Test 2**: Jog each joint individually (±1°, ±5°, ±10°)
    - **Test 3**: Set all 6 joints to zero position via sync write
    - **Test 4**: Stream positions for 10 seconds, verify no packet loss or corrupt data
    - **Test 5**: Oscillate joint 0 between -30° and +30° for 10 cycles
    - **Test 6**: Record 5-waypoint demo, playback, verify arm follows path
    - **Test 7**: Emergency stop during motion, verify immediate halt
    
18. Document any issues found:
    - Servo mechanical limits (may differ from spec)
    - Communication timing issues (adjust baud rate or delays if needed)
    - Flash write reliability
    
**Outcome**: Hardware validated, driver proven, safe to proceed to Phases 4-6 knowing joint control works.

---

## Phase 4 — Kinematics Module (POE with Dual-Core)

19. Implement matrix exponential library (`matrix_exp.c`):
    - `skew_symmetric(ω)` → 3×3 skew-symmetric matrix from angular velocity vector
    - `matrix_exp_se3(ξ, θ)` → 4×4 homogeneous transform via Rodrigues' formula
    - Rodrigues: exp([ω]θ) = I + sin(θ)[ω] + (1-cos(θ))[ω]²
20. Define robot geometry using POE (`robot_geometry.c`):
    - Store screw axes ξ₁...ξ₆ as (ω, v) pairs (6×6 array, 144 bytes)
    - Store home configuration M (4×4 matrix, end-effector pose at θ=0)
    - Store joint limits (min/max radians) per joint
    - Flash NVS storage for user calibration/overrides
21. Implement forward kinematics (`forward_kinematics_poe.c`):
    - FK(θ) = exp([ξ₁]θ₁) · exp([ξ₂]θ₂) · ... · exp([ξ₆]θ₆) · M
    - Chain 6 matrix exponentials in sequence
    - Returns end-effector pose (4×4 homogeneous transform)
22. Implement FK worker thread (pinned to APP_CPU Core 1):
    - Consumes FK requests from message queue (non-blocking for servo control)
    - Computes FK in 2-5ms, posts result back via message queue
    - Priority: lower than servo thread, higher than IK thread
23. Implement spatial Jacobian (`jacobian.c`):
    - J(θ) = [Ad(exp([ξ₁]θ₁)...exp([ξ₁₋₁]θ₁₋₁)) · ξᵢ] for each joint i
    - Used for IK solver (Newton-Raphson iteration)
24. Implement inverse kinematics solver (`inverse_kinematics_poe.c`):
    - Numerical IK via Newton-Raphson: θₖ₊₁ = θₖ + J⁺(θₖ)·(log(T_target · FK(θₖ)⁻¹))ᵛ
    - Damped pseudoinverse J⁺ = Jᵀ(JJᵀ + λ²I)⁻¹ for stability near singularities
    - Max 20 iterations, ε = 1mm position + 0.01 rad orientation tolerance
    - Returns closest solution or error code if unreachable
25. Implement IK worker thread (pinned to APP_CPU Core 1):
    - Consumes IK requests from message queue
    - Computes IK in 10-50ms depending on iterations, posts result back
    - Priority: lower than FK thread (FK needed for control loop, IK for planning)
26. Math dependencies: implement `mat4x4`, `vec3`, `se3_log`, `pseudoinverse` in `kinematics/math.h/.c` — **no external library** to keep footprint minimal
27. Unit test kinematics offline on host PC (same C source files, compiled with gcc, no Zephyr dependency)

---

## Phase 5 — Trajectory Planner

24. Implement `trajectory_point_t` struct: `{joint_angles[6], timestamp_ms}`
25. Implement `joint_trajectory` module — trapezoidal velocity profile in joint space:
    - Input: start angles, goal angles, max velocity, max acceleration (per joint)
    - Output: pre-computed array of `trajectory_point_t` at fixed 10ms interpolation steps
    - Enforces joint velocity and acceleration limits
26. Implement `cartesian_trajectory` (optional, Phase 5b):
    - Linear interpolation in Cartesian space, IK solved at each waypoint
    - Useful for straight-line end-effector paths
27. Trajectory buffer: ring buffer stored in RAM, capacity ~200 points (~2s of motion at 10ms step)

---

## Phase 6 — Motion Controller

28. Implement `motion_controller` Zephyr thread (highest priority, 1ms period via `k_timer`):
    - Reads current joint positions via `SyncRead` (or staggered individual reads)
    - Fetches next `trajectory_point_t` from trajectory buffer
    - Computes joint position error
    - Sends `SyncWrite` goal positions to servo bus
    - Feetech STS servos have internal PID — outer loop sends position setpoints only (no torque control needed unless servos support it)
29. Implement emergency stop: GPIO 39 (button) ISR disables motion, sends hold-position command to all servos
30. Watchdog: if host comms silent for >500ms during teleoperation, auto-stop

**Dual-core thread assignment**: Motion controller + servo I/O + safety monitoring pinned to PRO_CPU (Core 0) for real-time guarantees. FK/IK worker threads pinned to APP_CPU (Core 1) for parallel computation.

---

## Phase 7 — Host Command Protocol (USB-CDC) & Manual Modes

31. Define binary packet protocol over USB-CDC:
    ```
    [0xAA] [CMD] [PAYLOAD_LEN(1B)] [PAYLOAD...] [CRC8]
    ```
    **Core Commands:**
    - `0x01` MOVE_JOINTS — target joint angles (6× float32) + duration_ms
    - `0x02` MOVE_CARTESIAN — target pose (x,y,z,rx,ry,rz) + duration_ms
    - `0x03` READ_STATE — request current joint angles + end-effector pose
    - `0x04` STOP — immediate halt
    - `0x05` HOME — move to defined home position
    - `0x10` SET_PARAMS — update speed/accel limits
    
    **Manual/Debug Mode Commands:**
    - `0x20` JOG_JOINT — `[joint_id, direction, step_size_deg]` — incremental joint movement (±1°, ±5°, ±10°)
    - `0x21` SET_JOINT_DIRECT — `[joint_id, position]` — bypass trajectory planner, immediate servo command
    - `0x22` START_READ_LOOP — stream all joint positions at 100Hz (debugging/monitoring)
    - `0x23` STOP_READ_LOOP — stop continuous streaming
    - `0x24` SINGLE_JOINT_TEST — `[joint_id, start_angle, end_angle, cycles]` — oscillate joint for testing
    
    **Demo Recording & Playback Commands:**
    - `0x30` PLAY_DEMO — `[demo_id]` — play recorded demo sequence from flash
    - `0x31` START_DEMO_RECORDING — `[demo_id]` — begin recording waypoints to specified demo slot (0-2)
    - `0x32` ADD_WAYPOINT — `[delay_ms]` — capture current joint angles + time delay to next waypoint
    - `0x33` FINISH_DEMO_RECORDING — save current demo to flash, mark as complete
    - `0x34` CLEAR_DEMO — `[demo_id]` — erase stored demo from flash
    
    **Status Reports:**
    - `0xF0` STATUS_REPORT — MCU→host: joint angles, temperatures, error flags (periodic at 50Hz)
    - `0xF2` DEMO_RECORDING_STATUS — MCU→host: demo_id, waypoint count, bytes remaining
    
32. Implement `host_comms` Zephyr thread (lower priority):
    - Parses incoming packets, dispatches to controller/kinematics
    - Sends periodic `STATUS_REPORT` at 50Hz (or 100Hz in read loop mode)
    - Command dispatcher with mode state machine (normal / jogging / test / demo / recording)
33. CRC8 for packet integrity (Dallas/Maxim polynomial)
34. Implement demo recording system (host-commanded):
    - User jogs arm to desired positions using `JOG_JOINT` or `SET_JOINT_DIRECT`
    - Host sends `START_DEMO_RECORDING [demo_id]` to begin capture
    - At each waypoint, host sends `ADD_WAYPOINT [delay_ms]` to record current joint angles + time to next waypoint
    - Host sends `FINISH_DEMO_RECORDING` to save to flash
    - Store up to 3 demo sequences in flash (each max 50 waypoints @ 24 bytes = 1.2KB per demo, 3.6KB total)
    - Flash layout: demo header (ID, waypoint_count, CRC) + waypoint array (6× float32 + uint32 delay_ms per waypoint)

---

## Phase 8 — Integration & Testing

### Testing Architecture: Hybrid Three-Layer Approach

Testing uses three complementary layers:

1. **Python cross-validation** (`validation/`): Generate ground truth FK/IK outputs using the `modern-robotics` Python library (from Lynch & Park's *Modern Robotics* textbook). Compare against C implementation outputs to catch math errors.
2. **Zephyr Ztest unit tests** (`tests/`): Automated C unit tests using Zephyr's `CONFIG_ZTEST` framework. Run on `native_posix` (fast, no hardware) or on-target ESP32 for hardware-specific validation.
3. **Host integration tests** (`host_test.py`): End-to-end tests over serial (servo control, streaming, demos — already exist from Phase 3b).

### Directory Structure

```
octrobot/
├── tests/                          # Zephyr Ztest unit tests
│   └── kinematics/
│       ├── CMakeLists.txt          # Pulls in app/src/kinematics/ sources
│       ├── prj.conf                # CONFIG_ZTEST=y, CONFIG_FPU=y
│       ├── testcase.yaml           # Test metadata for twister
│       └── src/
│           ├── test_matrix_exp.c
│           ├── test_forward_kinematics.c
│           └── test_kinematics_math.c
└── validation/                     # Python cross-validation
    ├── requirements.txt            # modern-robotics, numpy, pytest
    ├── robot_config.yaml           # POE parameters (mirrors robot_geometry.c)
    ├── verify_fk.py                # Generate expected FK outputs, compare with C
    └── conftest.py                 # Shared pytest fixtures
```

### Layer 1: Python Cross-Validation (`validation/`)

Uses the `modern-robotics` library as canonical reference:
- `mr.FKinSpace(M, Slist, thetalist)` for FK ground truth
- `mr.JacobianSpace(Slist, thetalist)` for Jacobian ground truth
- `mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)` for IK ground truth
- `mr.MatrixExp6(se3mat)` for individual matrix exponential validation

Workflow:
1. Define POE parameters in `robot_config.yaml` (must match `robot_geometry.c`)
2. Generate test cases (zero config, single-joint, random configs)
3. Compute expected transforms via `modern-robotics`
4. Export as JSON test vectors (`test_data/fk_expected.json`)
5. Compare against C output (serial log capture or compiled host binary)

Run: `cd validation/ && python -m pytest` or `python verify_fk.py`

### Layer 2: Zephyr Ztest Unit Tests (`tests/`)

Ztest configuration:
- `CONFIG_ZTEST=y`, `CONFIG_ZTEST_NEW_API=y`
- `CONFIG_FPU=y`, `CONFIG_NEWLIB_LIBC=y` (for math functions)
- `CONFIG_MAIN_STACK_SIZE=4096`, `CONFIG_ZTEST_STACK_SIZE=4096`
- Test sources include actual kinematics `.c` files from `app/src/kinematics/`

Test cases (embed expected values from Python cross-validation):
- **Matrix exponential**: zero angle → identity, 90° Z rotation, 180° rotation, pure translation
- **Skew-symmetric**: known vector → expected 3×3 matrix
- **FK zero config**: all θ=0 → output must equal home transform M
- **FK single joint**: rotate joint 1 by 30° → compare position/rotation with `mr.FKinSpace`
- **FK multi-joint**: known 6-DOF config → compare full 4×4 transform
- **FK round-trip consistency**: FK at θ, extract position, verify deterministic
- **Performance**: 100 FK iterations < 500ms total (< 5ms each)

Run modes:
- `west build -b native_posix tests/kinematics/ && west build -t run` (fast, no hardware)
- `west build -b m5stack_atom_lite/esp32/procpu tests/kinematics/ && west flash` (on-target)
- `west twister -T tests/` (batch runner across platforms)

### Layer 3: Host Integration Tests (`host_test.py`)

Already exists from Phase 3b. Extend for kinematics:
- Send `$poe_fk_test 0 0 0 0 0 0` → compare output with Python expected value
- Send `$poe_validate` → verify all screws are valid unit vectors
- Servo sync write → FK of current positions → verify physical consistency

### Testing Workflow

1. **Development**: Write C code → run Python `verify_fk.py` to generate expected values → embed in Ztest
2. **Fast iteration**: Run Ztest on `native_posix` (sub-second feedback, no flashing)
3. **Hardware validation**: Run Ztest on ESP32 target (catches FPU precision, stack, timing issues)
4. **Regression**: `west twister -T tests/` on each code change
5. **Cross-validation**: Periodically re-run Python suite against serial output for end-to-end check

### Integration Tests

35. Integration test: send `MOVE_JOINTS` from host Python script → verify all 6 joints move to target
36. Integration test: send `MOVE_CARTESIAN` → verify IK solution is valid and arm reaches pose
37. Tune trajectory parameters (max vel/accel) for smooth, safe motion
38. Stress test: continuous motion commands for 30 minutes, monitor servo temperatures

---

## Future: ROS2 Bridge (Host Side Only, No Firmware Changes)

- Write a ROS2 node (`octrobot_driver`) that:
  - Opens the USB-CDC serial port
  - Subscribes to `/joint_trajectory_controller/command` (standard `trajectory_msgs`)
  - Translates to `MOVE_JOINTS` packets → MCU
  - Publishes `/joint_states` from `STATUS_REPORT` packets
- MoveIt2 integration via `ros2_control` hardware interface
- RViz2 visualization using URDF built from POE screw axes and home configuration

---

## Key Files (at completion)

- `app/src/hal/half_duplex_uart.h/.c`
- `app/src/drivers/feetech_bus.h/.c`
- `app/src/drivers/feetech_servo.h/.c`
- `app/src/kinematics/math.h/.c`
- `app/src/kinematics/matrix_exp.h/.c` — POE matrix exponentials (Rodrigues formula)
- `app/src/kinematics/robot_geometry.h/.c` — POE screw axes, home config, limits
- `app/src/kinematics/forward_kinematics_poe.h/.c` — FK via POE
- `app/src/kinematics/inverse_kinematics_poe.h/.c` — Numerical IK (Newton-Raphson)
- `app/src/kinematics/jacobian.h/.c` — Spatial Jacobian for IK solver
- `app/src/kinematics/calibration.h/.c` — UART commands for POE parameter setup
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
- **IK method**: numerical Newton-Raphson (10-50ms on ESP32 @ 240MHz, non-blocking via worker thread on APP_CPU)
- **Kinematics formulation**: POE (Product of Exponentials) with screw theory — singularity-free, better calibration than DH
- **Dual-core strategy**: SMP with selective thread pinning (CONFIG_SMP=y, CONFIG_SCHED_CPU_MASK=y) — critical threads pinned for isolation, flexible threads float for load balancing
- **Math library**: hand-rolled minimal math (no Eigen), keeping binary size small under Zephyr
- **Kinematics test**: same C source files compile on host PC for offline unit testing
