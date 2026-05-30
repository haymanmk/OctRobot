# Forward Kinematics Cross-Validation

OctroBot's FK implementation (C, float32, Zephyr native-sim) is verified against
**modern_robotics** (Python, float64, Lynch & Park *Modern Robotics*) as the
authoritative reference.  The pipeline runs in two layers:

```
robot_config.yaml
      │
      ▼
gen_test_vectors.py ──► fk_test_vectors.json       (human-readable ground truth)
  (modern_robotics)  ──► fk_test_vectors.h          (embedded in C native tests)
                                 │
                           C native-sim
                      test_fk_ground_truth.c
                      ± 1e-4 tolerance
                      (float32 vs float64)
```

## Quick start

```bash
make test           # both layers in one shot
make test-native    # C native-sim only
make test-py        # Python pytest only
```

All 43 Python tests and 38 C tests pass green.

## Layer 1 — Python cross-validation (`make test-py`)

```
validation/
├── robot_config.yaml      # POE model: 6 screw axes + home config M
├── conftest.py            # pytest fixtures: slist, home_config
└── test_fk_crossval.py    # pytest suite
```

Uses `mr.FKinSpace(M, Slist, θ)` from modern_robotics to verify:

- `FK(zero) == M` (home configuration identity)
- SE(3) validity (`RᵀR = I`, `det(R) = 1`, bottom row `[0 0 0 1]`)
- Single-joint position assertions derived from the current `robot_config.yaml`
- Multi-joint and random configurations

Run directly:
```bash
cd validation && python -m pytest -v
```

## Layer 2 — C native-sim ground-truth tests (`make test-native`)

```
tests/kinematics/
├── CMakeLists.txt
├── include/
│   └── fk_test_vectors.h          # auto-generated — do not edit
└── src/
    ├── test_fk_ground_truth.c     # 35 ground-truth vectors vs forward_kinematics_compute()
    ├── test_forward_kinematics.c  # FK property tests (null inputs, SE3, determinism, …)
    ├── test_matrix_exp.c          # Matrix exponential unit tests
    ├── test_kinematics_math.c     # Vec/mat math unit tests
    └── hal_flash_stub.c           # NVS stub → forces factory defaults
```

`test_fk_ground_truth.c` iterates every entry in `fk_test_vectors.h`, calls
`forward_kinematics_compute()` with the stored `theta[6]`, and asserts each
element of the 4×4 result is within **1e-4** of the float64 reference.  This
tolerance covers float32 accumulated error across 6 chained matrix exponentials.

`test_zero_config_equals_M` additionally checks that the C factory defaults in
`robot_geometry_factory_defaults()` match the YAML — catching any
C ↔ YAML drift at test time.

## Updating after a model change

When `validation/robot_config.yaml` changes (hardware calibration, new robot),
regenerate the ground-truth header before rebuilding:

```bash
cd validation
python gen_test_vectors.py        # rewrites fk_test_vectors.json + fk_test_vectors.h
cd ..
make test                         # verify both layers
```

Then update `app/src/kinematics/robot_geometry_factory_defaults()` to match the
new YAML values.  `test_zero_config_equals_M` will catch any remaining mismatch.

## Test vectors

`gen_test_vectors.py` generates 35 configurations covering:

| Group | Count | Description |
|---|---|---|
| Zero config | 1 | `θ = [0,…,0]`, must equal M |
| Single joint ±30° | 12 | Each of 6 joints at +30° and −30° |
| Single joint edge | 3 | Joint 0 at +90°, −90°, +180° |
| Adjacent pairs | 5 | Pairs (0,1), (1,2), (2,3), (3,4), (4,5) |
| All joints | 2 | All at 30°; mixed [10°,−20°,30°,−40°,50°,−60°] |
| Random (fixed seed) | 12 | `rng.uniform(−π/2, π/2, 6)`, seeds 0–11 |

## Robot model — Mecharm 270 Pi

The current model is the **Mecharm 270 Pi** (6-DOF).

Screw axes and home config are defined in `validation/robot_config.yaml` and
mirrored in `app/src/kinematics/robot_geometry.c` (`robot_geometry_factory_defaults()`).

Home configuration at `θ = [0,…,0]`:

```
M = [[ 0,  0,  1, 0.168],
     [-1,  0,  0, 0.000],
     [ 0, -1,  0, 0.243],
     [ 0,  0,  0, 1.000]]
```

End-effector position at home: `[0.168, 0, 0.243]` m.

## File map

| File | Purpose |
|---|---|
| `validation/robot_config.yaml` | Single source of truth for POE model |
| `validation/gen_test_vectors.py` | Generates ground-truth from modern_robotics |
| `validation/fk_test_vectors.json` | Float64 ground-truth (human-readable) |
| `validation/test_fk_crossval.py` | Python pytest suite |
| `validation/conftest.py` | Pytest fixtures (slist, home_config) |
| `tests/kinematics/include/fk_test_vectors.h` | Auto-generated C header (do not edit) |
| `tests/kinematics/src/test_fk_ground_truth.c` | C ground-truth test suite |
| `app/src/kinematics/robot_geometry.c` | C factory defaults (must match YAML) |
| `app/src/kinematics/forward_kinematics_poe.c` | FK implementation under test |
