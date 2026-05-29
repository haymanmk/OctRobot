# Inverse Kinematics Solver — Design Spec

**Date:** 2026-05-29
**Phase:** 4 (Kinematics)
**Status:** Approved design, pending implementation plan

## Goal

Implement a numerical inverse kinematics (IK) solver for the OctroBot 6-DOF arm
using the body-frame Newton-Raphson method (Modern Robotics `IKinBody`), and
validate it offline against `modern_robotics` and via FK round-trip — with **no
servo motion** in this round. This mirrors how forward kinematics was built and
tested first.

## Non-Goals (this round)

- Driving the physical servos / motion-controller integration (later, Phase 6).
- Host-protocol command for Cartesian pose targets (later, Phase 7–8).
- Adaptive/singularity-aware damping (noted as future work below).

## Background: existing foundation

The math library already implements every primitive the solver needs, so this
round adds the Jacobian assembly and the iteration loop only:

- `mat4x4_log_se3` — pose error as a body twist `Vb` (`kinematics_math.c:322`)
- `mat4x4_adjoint` / `mat6x6_mul_vec6` — adjoint for Jacobian columns and body
  screw-axis derivation (`kinematics_math.c:411`, `:448`)
- `mat4x4_inverse_transform` — `T⁻¹` and `M⁻¹` (`kinematics_math.c:275`)
- `jacobian_damped_pinv` — damped least-squares pseudoinverse
  (`kinematics_math.c:540`)
- `robot_geometry_check_joint_limits` / `_clamp_joint_angles` /
  `wrap_to_pi` — limit handling and angle wrapping

FK (`forward_kinematics_compute`) uses the **space-frame** PoE form
`T = exp([S₁]θ₁)···exp([S₆]θ₆)·M`, so the stored `screw_axes` are **space**
screw axes (the `Slist`).

## New files (mirrors the FK layout)

- `app/include/inverse_kinematics_poe.h` — public API
- `app/src/kinematics/inverse_kinematics_poe.c` — body Jacobian + solver loop
- `tests/kinematics/src/test_inverse_kinematics.c` — native-sim tests
- `validation/` — extend with IK cross-validation against
  `modern_robotics.IKinBody`

No changes to existing FK/math source files are expected. A small addition to
`kinematics_math` is permitted only if testing reveals a genuine gap.

## Public API

```c
typedef enum {
    IK_SUCCESS = 0,      /* converged, all joints within limits   */
    IK_OUT_OF_LIMITS,    /* converged, but >=1 joint out of limits */
    IK_NO_CONVERGENCE,   /* hit max_iters without converging      */
    IK_INVALID_INPUT,    /* NULL arg or bad model                 */
} ik_status_t;

typedef struct {
    float eomg;       /* angular error tol (rad), default 1e-3 */
    float ev;         /* linear error tol (m),    default 1e-4 */
    int   max_iters;  /* default 20                            */
    float lambda;     /* DLS damping, default 0.0 (pure pinv)  */
} ik_params_t;

ik_params_t inverse_kinematics_default_params(void);

ik_status_t inverse_kinematics_compute(
    const poe_robot_model_t *model,
    const mat4x4_t          *T_target,        /* desired T_sd          */
    const float              seed[NUM_JOINTS],/* initial guess         */
    float                    out_angles[NUM_JOINTS],
    const ik_params_t       *params,          /* NULL -> defaults      */
    int                     *iters_used);     /* optional, may be NULL */
```

## Algorithm (IKinBody)

1. **Derive the body screw list once** from the stored space screw list:
   `B_i = Ad_{M⁻¹}·S_i`, using `mat4x4_inverse_transform(M)` then
   `mat4x4_adjoint` and `mat6x6_mul_vec6`. (Valid because
   `exp([S]θ)···M == M·exp([B]θ)···` iff `B_i = Ad_{M⁻¹}·S_i`.)
2. **Iterate** from `seed` (θ ← seed):
   - `T_sb = FK(θ)` — **reuse `forward_kinematics_compute`** (space form yields
     the same end-effector pose, so no separate body FK is needed).
   - `Vb = log(T_sb⁻¹·T_sd)` via `mat4x4_log_se3`.
   - **Converged** if `‖Vb.ω‖ < eomg` and `‖Vb.v‖ < ev`.
   - Assemble `Jb(θ)` (see below).
   - `Δθ = Jb⁺·Vb` via `jacobian_damped_pinv(J, J_pinv, lambda, NUM_JOINTS)`.
   - `θ ← θ + Δθ`.
3. **On exit:** wrap angles to [-π,π], write the final θ iterate to
   `out_angles` (always — not necessarily the lowest-error iterate, matching the
   reference),
   then check joint limits and return `IK_SUCCESS` / `IK_OUT_OF_LIMITS` /
   `IK_NO_CONVERGENCE`. The caller decides what to do — the solver does not
   clamp during iteration (keeps the math identical to the Python reference).

### Body Jacobian assembly

Standard Modern Robotics column-by-column construction:

- `Jb[:, n-1] = B_{n-1}`
- Accumulate `T = I`; for `i` from `n-2` down to `0`:
  `T = T · exp(-[B_{i+1}]·θ_{i+1})`, then `Jb[:, i] = Ad_T · B_i`.

Stored as `float J[6][6]` to feed `jacobian_damped_pinv` directly.

## Damping (lambda) and the pseudoinverse — rationale

Each step solves `Jb·Δθ = Vb`. `Jb` becomes **singular** at certain
configurations (wrist axes aligned, full extension), where a plain inverse
explodes and the solver diverges. The pseudoinverse is the least-squares
generalization; the implementation computes the right pseudoinverse
`J⁺ = Jᵀ(J·Jᵀ + λ²I)⁻¹` (`kinematics_math.c:540`).

- **λ = 0** → plain Moore-Penrose pseudoinverse. Exact, minimal-norm `Δθ` away
  from singularities; matches `modern_robotics.IKinBody` (`np.linalg.pinv`)
  bit-for-bit. Diverges near singularities.
- **λ > 0** → damped least squares: adds `λ²` to the diagonal before inverting
  (line 552), keeping `(JJᵀ + λ²I)` well-conditioned. Solves
  `min ‖J·Δθ − Vb‖² + λ²‖Δθ‖²` — bounded `Δθ` near singularities at the cost of
  a small steady-state bias. λ sets the singular-value crossover: directions
  with `σ ≫ λ` pass through (`σ/(σ²+λ²) ≈ 1/σ`); as `σ → 0`,
  `σ/(σ²+λ²) → 0`, so the solver gracefully abandons the unreachable direction
  instead of going berserk.

**Decision:** default `lambda = 0.0` this round so the solver reproduces
`IKinBody` exactly — any mismatch is then a real bug, not a damping artifact.
The field stays exposed so a small λ ≈ 0.01–0.05 can be enabled for hardware
safety later without code changes.

**Future work (not this round):** adaptive λ — near-zero normally, ramping up
only as a singularity is approached (detected via smallest singular value or
`det(JJᵀ)`). Needs singularity detection that does not yet exist.

## Error handling

- NULL `model`/`T_target`/`seed`/`out_angles`, or failed model validation
  → `IK_INVALID_INPUT`.
- `jacobian_damped_pinv` returning false → treated as `IK_NO_CONVERGENCE`
  (the final θ iterate is still written to `out_angles`).
- No dynamic allocation. No logging on the hot path (respects the UART0
  binary-protocol log constraint — log level stays at WARNING).
- Single-precision float throughout (FPU enabled).

## Testing

### Native simulator (`tests/kinematics/`)

- **Round-trip:** sample random θ within joint limits → `FK` → `T` →
  `IK(T, seed = θ + small perturbation)` → θ′ → assert `FK(θ′) ≈ T` (pose match,
  not joint match — IK solutions are not unique).
- **Edge cases:** identity/home target; near-singular start configuration;
  unreachable target → `IK_NO_CONVERGENCE`; seed already at solution →
  converges in 0–1 iterations; out-of-limits solution → `IK_OUT_OF_LIMITS`.

### Python cross-validation (`validation/`)

- Extend `gen_test_vectors.py` to emit IK targets and expected results.
- Run the same `T_target`/`seed` through `modern_robotics.IKinBody` with the
  matching `Blist`, `M`, `eomg`, `ev`.
- Assert both converge to **pose-equivalent** solutions (compare via FK, since
  joint solutions may differ) and that convergence/non-convergence agrees.

## Acceptance criteria

- All native-sim IK tests pass (`make test-native`).
- Python cross-validation passes (`make test-py`).
- λ = 0.0 default produces solutions pose-equivalent to `IKinBody` within
  tolerances on the shared test vectors.
- No changes to FK behavior; existing FK tests still pass.
