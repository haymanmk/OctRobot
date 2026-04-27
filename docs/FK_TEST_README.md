# Forward Kinematics Test

## What's Implemented

The FK test validates the POE (Product of Exponentials) kinematics implementation with 5 test cases:

### Test Cases

1. **Zero Configuration** - Home position θ=[0,0,0,0,0,0]
   - Verifies FK matches home configuration matrix M
   
2. **Single Joint Movement** - Base rotation 30°
   - Tests basic rotation, end-effector moves in XY plane
   
3. **Multiple Joint Movements** - Complex pose
   - θ=[45°,30°,-20°,0°,15°,0°]
   - Validates full transformation chain
   - Displays position + rotation matrix
   
4. **Partial FK** - Intermediate link positions
   - Computes position of joint 2 only
   - Useful for collision detection/visualization
   
5. **Performance Measurement** - 100 iterations
   - Measures FK computation time in microseconds
   - Expected: 2000-5000 μs (2-5 ms) per FK call
   - ESP32 @ 240MHz with hardware FPU

## How to Run

### Build and Flash

```bash
cd /home/hayman/Workspace/octrobot
make build
make flash
```

### Monitor Output

```bash
make monitor
# Or use any serial terminal at 115200 baud
screen /dev/ttyUSB0 115200
```

## Expected Output

```
===========================================
Testing Forward Kinematics (POE)
===========================================
Robot model loaded:
  - Screw axes: 6 joints
  - Home config: M matrix
  - Joint limits configured

Test 1: Zero configuration (home position)
  FK([0,0,0,0,0,0]):
    Position: [0.5000, 0.0000, 0.1000] m
    (Should match home configuration M)

Test 2: Small joint movements
  FK([30°,0,0,0,0,0]):
    Position: [0.4330, 0.2500, 0.1000] m
    (Base rotated 30°, end-effector should move in XY plane)

Test 3: Multiple joint movements
  FK([45°,30°,-20°,0°,15°,0°]):
    Position: [0.xxxx, 0.xxxx, 0.xxxx] m
    Rotation matrix:
      [x.xxx, x.xxx, x.xxx]
      [x.xxx, x.xxx, x.xxx]
      [x.xxx, x.xxx, x.xxx]

Test 4: Partial FK (joint 2 position)
  FK_partial([45°,30°,-20°,...], end_idx=2):
    Joint 2 position: [0.xxxx, 0.xxxx, 0.xxxx] m
    (Position after joints 0-2)

Test 5: Performance measurement
  FK computation time:
    100 iterations: XXXXX cycles
    Average: XXXX cycles/FK (XXXX.XX μs)
    Expected: 2000-5000 μs (2-5 ms)
    ✓ Performance is good!

===========================================
Forward Kinematics test complete
===========================================
```

## Implementation Files

- **Math Library**: `app/src/kinematics/kinematics_math.c` (~700 lines)
  - Vec3, Mat3x3, Mat4x4 operations
  - Skew-symmetric matrices
  - SE(3) logarithm
  - Adjoint transformations
  
- **Matrix Exponentials**: `app/src/kinematics/matrix_exp.c`
  - Rodrigues' formula
  - exp([ω]θ) for SO(3) and SE(3)
  
- **Robot Geometry**: `app/src/kinematics/robot_geometry.c`
  - POE model storage (6 screw axes + home config)
  - Flash persistence with CRC32
  - Factory defaults (placeholder - needs calibration!)
  
- **Forward Kinematics**: `app/src/kinematics/forward_kinematics_poe.c`
  - FK(θ) = exp([ξ₁]θ₁)·exp([ξ₂]θ₂)·...·exp([ξ₆]θ₆)·M

## Memory Usage

Current firmware size:
- **FLASH**: 213,908 bytes (5.1% of 4MB)
- **IRAM**: 49,920 bytes (21.8% of 224KB)
- **DRAM**: 22,648 bytes (11.5% of 192KB)

POE model size: 256 bytes (stored in NVS flash)

## Next Steps

1. **Calibrate Robot** - Replace factory default screw axes with actual measured values
2. **Test with Real Servos** - Read joint positions, compute FK, validate results
3. **Implement IK** - Newton-Raphson inverse kinematics solver
4. **Add Worker Threads** - Offload FK/IK to APP_CPU (Core 1)

## Notes

⚠️ **Factory defaults are PLACEHOLDERS!**
- Current screw axes are generic values for a typical 6-DOF arm
- You MUST calibrate your specific robot:
  - Measure joint axis directions (visual inspection or CAD)
  - Measure link lengths (calipers, ±1mm accuracy)
  - Define home position (what should θ=0 look like?)
  - Test joint limits (manual determination of safe ranges)

See `/memories/session/poe_kinematics_plan.md` for calibration procedures.
