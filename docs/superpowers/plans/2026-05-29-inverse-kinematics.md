# Inverse Kinematics Solver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a body-frame Newton-Raphson inverse kinematics solver for the OctroBot 6-DOF arm and validate it offline (native-sim + Python) against `modern_robotics`, with no servo motion.

**Architecture:** New `inverse_kinematics_poe.{h,c}` in the kinematics layer. The solver reuses the existing `forward_kinematics_compute` for the pose at each iterate, derives the body screw list once via `B_i = Ad_{M⁻¹}·S_i`, assembles the body Jacobian column-by-column, and steps `θ += Jb⁺·Vb` using the existing `jacobian_damped_pinv`. Joint limits are checked only at exit (solve-free-then-flag). Validation mirrors FK: a Python generator embeds `modern_robotics`-derived targets into a C header consumed by a native ground-truth test.

**Tech Stack:** C (Zephyr Ztest, native_sim), float32; Python (`modern_robotics`, pytest); CMake; Make.

**Reference spec:** `docs/superpowers/specs/2026-05-29-inverse-kinematics-design.md`

---

## File Structure

- **Create** `app/include/inverse_kinematics_poe.h` — `ik_status_t`, `ik_params_t`, `inverse_kinematics_default_params`, `inverse_kinematics_compute`.
- **Create** `app/src/kinematics/inverse_kinematics_poe.c` — solver + internal body-screw-list / body-Jacobian helpers.
- **Create** `tests/kinematics/src/test_inverse_kinematics.c` — unit/behavior tests.
- **Create** `tests/kinematics/src/test_ik_ground_truth.c` — cross-validation against generated vectors.
- **Modify** `validation/gen_test_vectors.py` — also emit `tests/kinematics/include/ik_test_vectors.h`.
- **Create** `validation/test_ik_crossval.py` — Python IKinBody reference + `Blist` derivation checks.
- **Modify** `app/CMakeLists.txt` — register the new source for the firmware build.
- **Modify** `tests/kinematics/CMakeLists.txt` — register the new source + test files.

---

## Task 1: Scaffold the IK module (API, defaults, CMake wiring)

**Files:**
- Create: `app/include/inverse_kinematics_poe.h`
- Create: `app/src/kinematics/inverse_kinematics_poe.c`
- Create: `tests/kinematics/src/test_inverse_kinematics.c`
- Modify: `app/CMakeLists.txt`
- Modify: `tests/kinematics/CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/kinematics/src/test_inverse_kinematics.c`:

```c
/*
 * OctroBot - Inverse Kinematics Unit Tests
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include "kinematics_math.h"
#include "robot_geometry.h"
#include "forward_kinematics_poe.h"
#include "inverse_kinematics_poe.h"
#include <math.h>
#include <string.h>

#define IK_POSE_TOL 5e-3f

struct ik_fixture {
	poe_robot_model_t model;
};

static void *ik_setup(void)
{
	static struct ik_fixture fixture;
	fixture.model = robot_geometry_factory_defaults();
	return &fixture;
}

/* Build a 4x4 transform from identity rotation + translation (test helper). */
static mat4x4_t make_pose_identity_rot(float x, float y, float z)
{
	mat4x4_t T = mat4x4_identity();
	vec3_t t = vec3_create(x, y, z);
	mat4x4_set_translation(&T, &t);
	return T;
}

ZTEST_F(inverse_kinematics, test_default_params)
{
	ARG_UNUSED(fixture);
	ik_params_t p = inverse_kinematics_default_params();
	zassert_true(is_near_equal(p.eomg, 1e-3f, 1e-9f), "eomg default");
	zassert_true(is_near_equal(p.ev, 1e-4f, 1e-9f), "ev default");
	zassert_equal(p.max_iters, 20, "max_iters default");
	zassert_true(is_near_equal(p.lambda, 0.0f, 1e-9f), "lambda default");
}

ZTEST_F(inverse_kinematics, test_null_inputs_rejected)
{
	float seed[NUM_JOINTS] = {0};
	float out[NUM_JOINTS] = {0};
	mat4x4_t T = make_pose_identity_rot(0.2f, 0.0f, 0.2f);

	zassert_equal(inverse_kinematics_compute(NULL, &T, seed, out, NULL, NULL),
		      IK_INVALID_INPUT, "NULL model");
	zassert_equal(inverse_kinematics_compute(&fixture->model, NULL, seed, out, NULL, NULL),
		      IK_INVALID_INPUT, "NULL target");
	zassert_equal(inverse_kinematics_compute(&fixture->model, &T, NULL, out, NULL, NULL),
		      IK_INVALID_INPUT, "NULL seed");
	zassert_equal(inverse_kinematics_compute(&fixture->model, &T, seed, NULL, NULL, NULL),
		      IK_INVALID_INPUT, "NULL out");
}

ZTEST_SUITE(inverse_kinematics, NULL, ik_setup, NULL, NULL, NULL);
```

- [ ] **Step 2: Wire the new files into CMake**

In `tests/kinematics/CMakeLists.txt`, add the application source under test (in the "Application kinematics sources under test" `target_sources` block, after the `forward_kinematics_poe.c` line):

```cmake
  ${APP_SRC_DIR}/src/kinematics/inverse_kinematics_poe.c
```

And add the test file (in the "Test sources" `target_sources` block, after `test_fk_ground_truth.c`):

```cmake
  src/test_inverse_kinematics.c
```

In `app/CMakeLists.txt`, add to the "Kinematics (Phase 4 - POE implementation)" `target_sources` block (after `forward_kinematics_poe.c`):

```cmake
  src/kinematics/inverse_kinematics_poe.c
```

- [ ] **Step 3: Run test to verify it fails**

Run: `make test-native`
Expected: FAIL — build error, `inverse_kinematics_poe.h: No such file or directory`.

- [ ] **Step 4: Create the header**

Create `app/include/inverse_kinematics_poe.h`:

```c
/*
 * OctroBot Robot Arm Firmware - Inverse Kinematics (POE, body-frame)
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Body-frame Newton-Raphson IK solver (Modern Robotics IKinBody).
 * Offline-validated against the modern_robotics Python library.
 */

#ifndef INVERSE_KINEMATICS_POE_H
#define INVERSE_KINEMATICS_POE_H

#include "kinematics_math.h"
#include "robot_geometry.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Outcome of an IK solve. */
typedef enum {
	IK_SUCCESS = 0,    /* converged, all joints within limits      */
	IK_OUT_OF_LIMITS,  /* converged, but >=1 joint out of limits   */
	IK_NO_CONVERGENCE, /* hit max_iters without converging         */
	IK_INVALID_INPUT,  /* NULL argument                            */
} ik_status_t;

/** Solver tuning parameters. */
typedef struct {
	float eomg;      /* angular error tolerance (rad)   */
	float ev;        /* linear error tolerance (m)      */
	int   max_iters; /* maximum Newton-Raphson iterations */
	float lambda;    /* damped-least-squares factor (0 = pure pinv) */
} ik_params_t;

/**
 * Default parameters: eomg=1e-3, ev=1e-4, max_iters=20, lambda=0.0.
 * lambda=0.0 reproduces modern_robotics.IKinBody for clean cross-validation.
 */
ik_params_t inverse_kinematics_default_params(void);

/**
 * Solve IK for a desired end-effector pose.
 *
 * Inputs:
 *   - model:     robot geometry (space-frame screw axes + home config M)
 *   - T_target:  desired end-effector pose T_sd
 *   - seed:      initial joint-angle guess (radians)
 *   - params:    tuning parameters, or NULL for defaults
 * Outputs:
 *   - out_angles:  final joint angles, wrapped to [-pi, pi] (always written)
 *   - iters_used:  iterations performed (optional, may be NULL)
 *
 * Returns an ik_status_t. out_angles always holds the final iterate, even on
 * non-convergence.
 */
ik_status_t inverse_kinematics_compute(const poe_robot_model_t *model,
				       const mat4x4_t *T_target,
				       const float seed[NUM_JOINTS],
				       float out_angles[NUM_JOINTS],
				       const ik_params_t *params,
				       int *iters_used);

#ifdef __cplusplus
}
#endif

#endif /* INVERSE_KINEMATICS_POE_H */
```

- [ ] **Step 5: Create the source stub**

Create `app/src/kinematics/inverse_kinematics_poe.c` (stub — full solver lands in Task 2):

```c
/*
 * OctroBot Robot Arm Firmware - Inverse Kinematics Implementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "inverse_kinematics_poe.h"
#include "forward_kinematics_poe.h"
#include "matrix_exp.h"
#include <string.h>

ik_params_t inverse_kinematics_default_params(void)
{
	ik_params_t p = {
		.eomg = 1e-3f,
		.ev = 1e-4f,
		.max_iters = 20,
		.lambda = 0.0f,
	};
	return p;
}

ik_status_t inverse_kinematics_compute(const poe_robot_model_t *model,
				       const mat4x4_t *T_target,
				       const float seed[NUM_JOINTS],
				       float out_angles[NUM_JOINTS],
				       const ik_params_t *params,
				       int *iters_used)
{
	if (model == NULL || T_target == NULL || seed == NULL ||
	    out_angles == NULL) {
		return IK_INVALID_INPUT;
	}

	ARG_UNUSED(params);
	if (iters_used != NULL) {
		*iters_used = 0;
	}
	memcpy(out_angles, seed, sizeof(float) * NUM_JOINTS);
	return IK_NO_CONVERGENCE; /* replaced in Task 2 */
}
```

Note: `ARG_UNUSED` comes from Zephyr's `sys/util.h`, already transitively included via Ztest in tests; in the app source add `#include <zephyr/sys/util.h>` if the firmware build warns. To keep the stub self-contained, instead of `ARG_UNUSED(params);` you may write `(void)params;`.

- [ ] **Step 6: Run tests to verify they pass**

Run: `make test-native`
Expected: PASS — suite `inverse_kinematics` runs; `test_default_params` and `test_null_inputs_rejected` pass. Existing FK suites still pass.

- [ ] **Step 7: Commit**

```bash
git add app/include/inverse_kinematics_poe.h \
        app/src/kinematics/inverse_kinematics_poe.c \
        tests/kinematics/src/test_inverse_kinematics.c \
        app/CMakeLists.txt tests/kinematics/CMakeLists.txt
git commit -m "feat(ik): scaffold inverse kinematics module (API + defaults)"
```

---

## Task 2: Implement the body-frame Newton-Raphson solver

**Files:**
- Modify: `app/src/kinematics/inverse_kinematics_poe.c`
- Modify: `tests/kinematics/src/test_inverse_kinematics.c`

- [ ] **Step 1: Write the failing tests**

Append these tests to `tests/kinematics/src/test_inverse_kinematics.c`, immediately before the `ZTEST_SUITE(...)` line:

```c
/* IK at the home pose with zero seed converges back to ~zero angles. */
ZTEST_F(inverse_kinematics, test_home_target_zero_seed)
{
	float seed[NUM_JOINTS] = {0};
	float out[NUM_JOINTS] = {0};
	int iters = -1;
	mat4x4_t T_target = robot_geometry_get_home_config(&fixture->model);

	ik_status_t st = inverse_kinematics_compute(&fixture->model, &T_target,
						    seed, out, NULL, &iters);

	zassert_equal(st, IK_SUCCESS, "home target should solve, got %d", st);
	for (int i = 0; i < NUM_JOINTS; i++) {
		zassert_true(fabsf(out[i]) < 1e-3f,
			     "joint %d should be ~0, got %.6f", i, (double)out[i]);
	}
	zassert_true(iters >= 0, "iters should be reported");
}

/* Round-trip: FK(theta) -> T, then IK(T, perturbed seed) -> theta', FK(theta') ~= T. */
ZTEST_F(inverse_kinematics, test_round_trip_pose_match)
{
	const float theta_true[NUM_JOINTS] = {
		0.2f, -0.3f, 0.4f, -0.2f, 0.3f, -0.1f
	};
	const float seed[NUM_JOINTS] = {
		0.1f, -0.2f, 0.3f, -0.1f, 0.2f, 0.0f
	};
	mat4x4_t T_target;
	zassert_true(forward_kinematics_compute(&fixture->model, theta_true,
						&T_target),
		     "FK setup should succeed");

	float out[NUM_JOINTS] = {0};
	ik_status_t st = inverse_kinematics_compute(&fixture->model, &T_target,
						    seed, out, NULL, NULL);
	zassert_true(st == IK_SUCCESS || st == IK_OUT_OF_LIMITS,
		     "IK should converge, got %d", st);

	mat4x4_t T_check;
	zassert_true(forward_kinematics_compute(&fixture->model, out, &T_check),
		     "FK recheck should succeed");
	zassert_true(mat4x4_is_equal(&T_check, &T_target, IK_POSE_TOL),
		     "FK(IK(T)) should match T within %.0e", (double)IK_POSE_TOL);
}

/* A target far outside the workspace cannot converge. */
ZTEST_F(inverse_kinematics, test_unreachable_target)
{
	float seed[NUM_JOINTS] = {0};
	float out[NUM_JOINTS] = {0};
	mat4x4_t T_target = make_pose_identity_rot(5.0f, 5.0f, 5.0f);

	ik_status_t st = inverse_kinematics_compute(&fixture->model, &T_target,
						    seed, out, NULL, NULL);
	zassert_equal(st, IK_NO_CONVERGENCE,
		      "far target should not converge, got %d", st);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `make test-native`
Expected: FAIL — `test_home_target_zero_seed` and `test_round_trip_pose_match` fail (stub returns `IK_NO_CONVERGENCE` and copies the seed unchanged). `test_unreachable_target` may incidentally pass.

- [ ] **Step 3: Implement the full solver**

Replace the entire body of `app/src/kinematics/inverse_kinematics_poe.c` with:

```c
/*
 * OctroBot Robot Arm Firmware - Inverse Kinematics Implementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Body-frame Newton-Raphson IK (Modern Robotics IKinBody).
 *   T_sb  = FK(theta)            (space-frame FK; same end pose as body FK)
 *   Vb    = log(T_sb^-1 * T_sd)  (body twist error)
 *   theta = theta + Jb^+ * Vb    (damped least-squares step)
 */

#include "inverse_kinematics_poe.h"
#include "forward_kinematics_poe.h"
#include "matrix_exp.h"
#include <string.h>

/*
 * Body screw list: B_i = Ad_{M^-1} * S_i.
 * Valid because exp([S]th)...M == M*exp([B]th)...  iff  B_i = Ad_{M^-1} S_i.
 */
static void derive_body_screw_axes(const poe_robot_model_t *model,
				   vec6_t Blist[NUM_JOINTS])
{
	mat4x4_t M = robot_geometry_get_home_config(model);
	mat4x4_t M_inv = mat4x4_inverse_transform(&M);
	mat6x6_t AdMinv = mat4x4_adjoint(&M_inv);

	for (int i = 0; i < NUM_JOINTS; i++) {
		vec6_t Si;
		robot_geometry_get_screw_axis(model, i, &Si);
		Blist[i] = mat6x6_mul_vec6(&AdMinv, &Si);
	}
}

/*
 * Body Jacobian Jb(theta), stored as J[6][6] with rows [wx,wy,wz,vx,vy,vz].
 *   Jb[:, n-1] = B_{n-1}
 *   T = I; for i = n-2 .. 0:  T = T * exp(-[B_{i+1}] theta_{i+1});
 *                             Jb[:, i] = Ad_T * B_i
 */
static void body_jacobian(const vec6_t Blist[NUM_JOINTS],
			  const float theta[NUM_JOINTS],
			  float J[6][6])
{
	vec6_t col[NUM_JOINTS];

	col[NUM_JOINTS - 1] = Blist[NUM_JOINTS - 1];

	mat4x4_t T = mat4x4_identity();
	for (int i = NUM_JOINTS - 2; i >= 0; i--) {
		mat4x4_t E = matrix_exp_se3(&Blist[i + 1], -theta[i + 1]);
		T = mat4x4_mul(&T, &E);
		mat6x6_t Ad = mat4x4_adjoint(&T);
		col[i] = mat6x6_mul_vec6(&Ad, &Blist[i]);
	}

	for (int c = 0; c < NUM_JOINTS; c++) {
		J[0][c] = col[c].w.x;
		J[1][c] = col[c].w.y;
		J[2][c] = col[c].w.z;
		J[3][c] = col[c].v.x;
		J[4][c] = col[c].v.y;
		J[5][c] = col[c].v.z;
	}
}

ik_params_t inverse_kinematics_default_params(void)
{
	ik_params_t p = {
		.eomg = 1e-3f,
		.ev = 1e-4f,
		.max_iters = 20,
		.lambda = 0.0f,
	};
	return p;
}

ik_status_t inverse_kinematics_compute(const poe_robot_model_t *model,
				       const mat4x4_t *T_target,
				       const float seed[NUM_JOINTS],
				       float out_angles[NUM_JOINTS],
				       const ik_params_t *params,
				       int *iters_used)
{
	if (model == NULL || T_target == NULL || seed == NULL ||
	    out_angles == NULL) {
		return IK_INVALID_INPUT;
	}

	ik_params_t p = (params != NULL) ? *params
					 : inverse_kinematics_default_params();

	vec6_t Blist[NUM_JOINTS];
	derive_body_screw_axes(model, Blist);

	float theta[NUM_JOINTS];
	memcpy(theta, seed, sizeof(theta));

	bool converged = false;
	int iter = 0;

	for (iter = 0; iter < p.max_iters; iter++) {
		mat4x4_t T_sb;
		if (!forward_kinematics_compute(model, theta, &T_sb)) {
			break;
		}

		mat4x4_t T_sb_inv = mat4x4_inverse_transform(&T_sb);
		mat4x4_t T_bd = mat4x4_mul(&T_sb_inv, T_target);

		vec6_t Vb;
		if (!mat4x4_log_se3(&T_bd, &Vb)) {
			break;
		}

		if (vec3_norm(&Vb.w) < p.eomg && vec3_norm(&Vb.v) < p.ev) {
			converged = true;
			break;
		}

		float J[6][6];
		body_jacobian(Blist, theta, J);

		float Jpinv[6][6];
		if (!jacobian_damped_pinv(J, Jpinv, p.lambda, NUM_JOINTS)) {
			break;
		}

		const float Vb_arr[6] = {
			Vb.w.x, Vb.w.y, Vb.w.z, Vb.v.x, Vb.v.y, Vb.v.z
		};
		for (int i = 0; i < NUM_JOINTS; i++) {
			float dtheta = 0.0f;
			for (int k = 0; k < 6; k++) {
				dtheta += Jpinv[i][k] * Vb_arr[k];
			}
			theta[i] += dtheta;
		}
	}

	for (int i = 0; i < NUM_JOINTS; i++) {
		out_angles[i] = wrap_to_pi(theta[i]);
	}
	if (iters_used != NULL) {
		*iters_used = iter;
	}

	if (!converged) {
		return IK_NO_CONVERGENCE;
	}
	if (!robot_geometry_check_joint_limits(model, out_angles)) {
		return IK_OUT_OF_LIMITS;
	}
	return IK_SUCCESS;
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `make test-native`
Expected: PASS — all `inverse_kinematics` tests pass (home target → zeros, round-trip pose match, unreachable → no convergence). Existing FK/math suites still pass.

- [ ] **Step 5: Commit**

```bash
git add app/src/kinematics/inverse_kinematics_poe.c \
        tests/kinematics/src/test_inverse_kinematics.c
git commit -m "feat(ik): implement body-frame Newton-Raphson IK solver"
```

---

## Task 3: Native ground-truth cross-validation against modern_robotics

**Files:**
- Modify: `validation/gen_test_vectors.py`
- Create: `tests/kinematics/include/ik_test_vectors.h` (generated)
- Create: `tests/kinematics/src/test_ik_ground_truth.c`
- Modify: `tests/kinematics/CMakeLists.txt`

- [ ] **Step 1: Extend the vector generator**

In `validation/gen_test_vectors.py`, after the FK C-header write (after the final `print(f"Wrote {HEADER_OUT}")` near the end of the file), append:

```python
# ---------------------------------------------------------------------------
# IK ground-truth vectors (body-frame IKinBody)
# ---------------------------------------------------------------------------

IK_HEADER_OUT = os.path.join(REPO_ROOT, "tests", "kinematics", "include",
                             "ik_test_vectors.h")

# Body screw list: B_i = Ad_{M^-1} * S_i  (columns are body screw axes).
Blist = mr.Adjoint(mr.TransInv(M)) @ Slist  # (6, N)

IK_EOMG = 1e-3
IK_EV = 1e-4

ik_cases = []


def add_ik(theta_true, label, rng):
    theta_true = np.asarray(theta_true, dtype=float)
    T_target = mr.FKinSpace(M, Slist, theta_true)
    seed = theta_true + rng.uniform(-0.1, 0.1, size=N)
    theta_sol, ok = mr.IKinBody(Blist, M, T_target, seed.copy(),
                                IK_EOMG, IK_EV)
    ik_cases.append({"description": label,
                     "seed": list(seed),
                     "T_target": T_target.tolist(),
                     "theta_ref": list(theta_sol),
                     "converged": bool(ok)})


# Home + reproducible random reachable configs (seed perturbed from truth).
add_ik([0.0] * N, "ik_home", np.random.default_rng(100))
for seed_id in range(12):
    rng = np.random.default_rng(200 + seed_id)
    th = rng.uniform(-np.pi / 3, np.pi / 3, size=N)
    add_ik(list(th), f"ik_rng_seed{seed_id}", rng)

n_conv = sum(1 for c in ik_cases if c["converged"])
print(f"Generated {len(ik_cases)} IK vectors ({n_conv} converged).")

ik_lines = [
    "/* Auto-generated by validation/gen_test_vectors.py - do not edit */",
    f"/* Robot: mecharm_270_pi  |  Generated: {date.today()} */",
    "#ifndef IK_TEST_VECTORS_H",
    "#define IK_TEST_VECTORS_H",
    "",
    "#include <stddef.h>",
    "",
    f"#define NUM_IK_TEST_VECTORS {len(ik_cases)}",
    "",
    "typedef struct {",
    "    float seed[6];",
    "    float T_target[4][4];",
    "    float theta_ref[6];",
    "    int converged;",
    "    const char *description;",
    "} ik_test_vector_t;",
    "",
    "static const ik_test_vector_t IK_TEST_VECTORS[NUM_IK_TEST_VECTORS] = {",
]

for i, c in enumerate(ik_cases):
    ik_lines.append(f"    /* [{i}] {c['description']} */")
    ik_lines.append("    {")
    ik_lines.append(f"        .seed = {fmt_theta(c['seed'])},")
    ik_lines.append(f"        .T_target = {fmt_mat(np.array(c['T_target']))},")
    ik_lines.append(f"        .theta_ref = {fmt_theta(c['theta_ref'])},")
    ik_lines.append(f"        .converged = {1 if c['converged'] else 0},")
    ik_lines.append(f"        .description = \"{c['description']}\"")
    sep = "," if i < len(ik_cases) - 1 else ""
    ik_lines.append(f"    }}{sep}")

ik_lines += ["};", "", "#endif /* IK_TEST_VECTORS_H */", ""]

with open(IK_HEADER_OUT, "w") as f:
    f.write("\n".join(ik_lines))

print(f"Wrote {IK_HEADER_OUT}")
```

(Reuses the existing `fmt_float`, `fmt_theta`, `fmt_mat`, `Slist`, `M`, `N`, `date`, `np` defined earlier in the file.)

- [ ] **Step 2: Generate the IK header**

Run: `cd validation && python gen_test_vectors.py && cd ..`
Expected: prints `Generated 13 IK vectors (... converged).` and `Wrote .../tests/kinematics/include/ik_test_vectors.h`. Confirm the file exists:

Run: `ls tests/kinematics/include/ik_test_vectors.h`
Expected: the path prints (file exists).

- [ ] **Step 3: Write the ground-truth test**

Create `tests/kinematics/src/test_ik_ground_truth.c`:

```c
/*
 * OctroBot - IK Ground-Truth Cross-Validation Tests
 * SPDX-License-Identifier: Apache-2.0
 *
 * Each vector holds a seed, a target pose T_target produced by FK on a known
 * config, and whether modern_robotics.IKinBody converged from that seed.
 * For every vector where the reference converged, the C solver must converge
 * too and FK(out) must match T_target (poses, since IK solutions are not
 * unique).
 *
 * Regenerate vectors:
 *   cd validation && python gen_test_vectors.py
 * Then: make test-native
 */

#include <zephyr/ztest.h>
#include "kinematics_math.h"
#include "robot_geometry.h"
#include "forward_kinematics_poe.h"
#include "inverse_kinematics_poe.h"
#include "ik_test_vectors.h"
#include <math.h>

#define IK_GT_POSE_TOL 5e-3f

struct ik_ground_truth_fixture {
	poe_robot_model_t model;
};

static void *ik_gt_setup(void)
{
	static struct ik_ground_truth_fixture fixture;
	fixture.model = robot_geometry_factory_defaults();
	return &fixture;
}

ZTEST_F(ik_ground_truth, test_all_vectors)
{
	for (int v = 0; v < NUM_IK_TEST_VECTORS; v++) {
		const ik_test_vector_t *vec = &IK_TEST_VECTORS[v];

		if (!vec->converged) {
			continue; /* reference did not converge; skip */
		}

		mat4x4_t T_target;
		for (int r = 0; r < 4; r++) {
			for (int c = 0; c < 4; c++) {
				T_target.m[r][c] = vec->T_target[r][c];
			}
		}

		float out[NUM_JOINTS] = {0};
		ik_status_t st = inverse_kinematics_compute(
			&fixture->model, &T_target, vec->seed, out, NULL, NULL);

		zassert_true(st == IK_SUCCESS || st == IK_OUT_OF_LIMITS,
			     "vector %d (%s): expected convergence, got %d",
			     v, vec->description, st);

		mat4x4_t T_check;
		zassert_true(forward_kinematics_compute(&fixture->model, out,
							&T_check),
			     "vector %d (%s): FK recheck failed",
			     v, vec->description);
		zassert_true(mat4x4_is_equal(&T_check, &T_target, IK_GT_POSE_TOL),
			     "vector %d (%s): FK(IK(T)) != T within %.0e",
			     v, vec->description, (double)IK_GT_POSE_TOL);
	}
}

ZTEST_SUITE(ik_ground_truth, NULL, ik_gt_setup, NULL, NULL, NULL);
```

- [ ] **Step 4: Register the test**

In `tests/kinematics/CMakeLists.txt`, add to the "Test sources" `target_sources` block (after `src/test_inverse_kinematics.c`):

```cmake
  src/test_ik_ground_truth.c
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `make test-native`
Expected: PASS — `ik_ground_truth::test_all_vectors` passes for all converged reference vectors; all prior suites still pass.

- [ ] **Step 6: Commit**

```bash
git add validation/gen_test_vectors.py \
        tests/kinematics/include/ik_test_vectors.h \
        tests/kinematics/src/test_ik_ground_truth.c \
        tests/kinematics/CMakeLists.txt
git commit -m "test(ik): cross-validate IK against modern_robotics ground truth"
```

---

## Task 4: Python IK cross-validation (reference + Blist derivation)

**Files:**
- Create: `validation/test_ik_crossval.py`

This validates the `modern_robotics.IKinBody` reference behavior and the `Blist = Ad_{M⁻¹}·S` derivation that the C solver relies on, so `make test-py` covers IK too.

- [ ] **Step 1: Write the test**

Create `validation/test_ik_crossval.py`:

```python
"""
OctroBot IK Cross-Validation Tests

Validates the body-frame IK reference (modern_robotics.IKinBody) and the body
screw-list derivation Blist = Adjoint(TransInv(M)) @ Slist that the C solver
uses. Mirrors the FK cross-validation harness.

Run:  cd validation && python -m pytest test_ik_crossval.py -v
"""

import numpy as np
import modern_robotics as mr
import pytest

EOMG = 1e-3
EV = 1e-4


@pytest.fixture(scope="session")
def blist(slist, home_config):
    """Body screw list: each column B_i = Adjoint(TransInv(M)) @ S_i."""
    return mr.Adjoint(mr.TransInv(home_config)) @ slist


class TestBlistDerivation:
    """Body FK with Blist must equal space FK with Slist for any config."""

    @pytest.mark.parametrize("seed", range(10))
    def test_body_fk_matches_space_fk(self, slist, blist, home_config,
                                      num_joints, seed):
        rng = np.random.default_rng(seed)
        theta = rng.uniform(-np.pi / 2, np.pi / 2, size=num_joints)
        T_space = mr.FKinSpace(home_config, slist, theta)
        T_body = mr.FKinBody(home_config, blist, theta)
        np.testing.assert_allclose(T_body, T_space, atol=1e-9)


class TestIKinBodyRoundTrip:
    """IKinBody must recover a pose-equivalent solution from a nearby seed."""

    @pytest.mark.parametrize("seed", range(15))
    def test_round_trip(self, slist, blist, home_config, num_joints, seed):
        rng = np.random.default_rng(1000 + seed)
        theta_true = rng.uniform(-np.pi / 3, np.pi / 3, size=num_joints)
        T_target = mr.FKinSpace(home_config, slist, theta_true)

        guess = theta_true + rng.uniform(-0.1, 0.1, size=num_joints)
        theta_sol, ok = mr.IKinBody(blist, home_config, T_target,
                                    guess.copy(), EOMG, EV)
        assert ok, f"IKinBody failed to converge (seed {seed})"

        T_check = mr.FKinSpace(home_config, slist, theta_sol)
        np.testing.assert_allclose(T_check[:3, 3], T_target[:3, 3], atol=1e-3)
        np.testing.assert_allclose(T_check[:3, :3], T_target[:3, :3], atol=1e-3)

    def test_home_target_solves_to_zero(self, slist, blist, home_config,
                                        num_joints):
        theta_sol, ok = mr.IKinBody(blist, home_config, home_config,
                                    np.zeros(num_joints), EOMG, EV)
        assert ok
        np.testing.assert_allclose(theta_sol, np.zeros(num_joints), atol=1e-3)
```

- [ ] **Step 2: Run tests to verify they pass**

Run: `make test-py`
Expected: PASS — `test_ik_crossval.py` tests pass alongside the existing FK tests.

- [ ] **Step 3: Commit**

```bash
git add validation/test_ik_crossval.py
git commit -m "test(ik): add Python IK cross-validation (IKinBody + Blist)"
```

---

## Task 5: Full validation pass + docs

**Files:**
- Modify: `CLAUDE.md`
- Modify: `README.md` (only if it carries a phase/status table)

- [ ] **Step 1: Run the full test suite**

Run: `make test`
Expected: PASS — both `make test-native` and `make test-py` succeed end-to-end.

- [ ] **Step 2: Update the phase roadmap**

In `CLAUDE.md`, in the "Phase Roadmap" table, change the Phase 4 row from:

```
| 4     | ⏳ In Progress | FK complete; IK not started |
```

to:

```
| 4     | ⏳ In Progress | FK complete; IK solver complete + offline-validated; IK not yet wired to motion control |
```

And in the architecture diagram caption line, update:

```
    Kinematics Layer   (Phase 4 - POE FK complete; IK not started)
```

to:

```
    Kinematics Layer   (Phase 4 - POE FK + body-frame IK complete; offline-validated)
```

- [ ] **Step 3: Update README if it has a matching status line**

Run: `grep -n "IK not started\|IK not yet\|inverse kinematics" README.md`
If a Phase 4 / IK status line is found, edit it to read "body-frame IK solver complete (offline-validated); not yet wired to motion control." If `grep` finds nothing, skip this step (no change needed).

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md README.md
git commit -m "docs: mark Phase 4 IK solver complete (offline-validated)"
```

---

## Self-Review Notes

- **Spec coverage:** API (Task 1), body-screw derivation + body Jacobian + Newton-Raphson loop with DLS step (Task 2), error handling incl. NULL / pinv-failure / wrap-to-pi / limit flag (Tasks 1–2), native round-trip + edge cases (Task 2), native ground-truth vs `modern_robotics` (Task 3), Python cross-validation (Task 4), λ=0.0 default (Task 1). All spec sections map to a task.
- **λ default:** `inverse_kinematics_default_params` sets `lambda = 0.0` per the spec; the field is exposed for future hardware damping.
- **Type consistency:** `ik_status_t`, `ik_params_t`, `inverse_kinematics_default_params`, `inverse_kinematics_compute` signatures are identical across header, source, and tests. Twist flattening order is `[wx,wy,wz,vx,vy,vz]` in both `body_jacobian` (J rows) and the `Vb_arr` step, matching `mat4x4_log_se3`'s `vec6_t {w, v}` layout.
- **Tolerances:** convergence `eomg=1e-3`, `ev=1e-4`; native pose round-trip tol `5e-3` (margin for float32 over the iteration + FK recheck). Documented in tests.
- **No placeholders:** every code/command step is concrete.
