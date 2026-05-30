# Cartesian Move-to-Pose Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `$movec x y z roll pitch yaw [time_ms]` host-console command that moves the 6-DOF arm to a commanded Cartesian pose by running the IK solver and sending joint goals to the servos with onboard motion smoothing.

**Architecture:** Four small units. `mat3x3_from_rpy` (pure RPY→rotation helper in the math lib) → `cartesian_pose_to_joints` (pure: RPY+position → SE(3) target → IK → model-frame joint angles) → `joint_map` (model-radians ↔ servo-degrees via a per-joint sign+offset table) → `cartesian_move_to_pose` (hardware wrapper: e-stop check, read servos for the IK seed, run the pure core, map to servo degrees, range-check, `sync_write_angles` with a move time). The first three are unit-tested on native_sim; the hardware wrapper is validated on the device.

**Tech Stack:** C (Zephyr RTOS, native_sim Ztest), float32; Feetech servo driver; existing POE FK/IK kinematics.

**Reference spec:** `docs/superpowers/specs/2026-05-29-cartesian-move-to-pose-design.md`

---

## File Structure

- **Modify** `app/include/kinematics_math.h` + `app/src/kinematics/kinematics_math.c` — add `mat3x3_from_rpy`.
- **Create** `app/include/joint_map.h` + `app/src/controller/joint_map.c` — sign+offset table, two converters, runtime setter.
- **Create** `app/include/cartesian_pose.h` + `app/src/controller/cartesian_pose.c` — `cmove_status_t` enum + the pure `cartesian_pose_to_joints` core (no hardware deps).
- **Create** `app/include/cartesian_move.h` + `app/src/controller/cartesian_move.c` — the hardware wrapper `cartesian_move_to_pose` (servo + HAL).
- **Modify** `app/src/comms/uart_console.c` — add the `$movec` command.
- **Modify** `app/CMakeLists.txt` — register the three new `.c` files.
- **Modify** `tests/kinematics/CMakeLists.txt` — compile `joint_map.c` + `cartesian_pose.c` (NOT `cartesian_move.c`, which needs hardware) and the three new test files.
- **Create** test files: `tests/kinematics/src/test_rpy.c`, `test_joint_map.c`, `test_cartesian_pose.c`.

**Convention note (locked):** RPY = **ZYX intrinsic** `R = Rz(yaw)·Ry(pitch)·Rx(roll)`. Position in **meters**, RPY in **degrees**, move time in **ms** (default 1000). On servo-read failure the wrapper falls back to a zero seed and still attempts the move.

---

## Task 1: `mat3x3_from_rpy` rotation helper

**Files:**
- Modify: `app/include/kinematics_math.h`
- Modify: `app/src/kinematics/kinematics_math.c`
- Create: `tests/kinematics/src/test_rpy.c`
- Modify: `tests/kinematics/CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/kinematics/src/test_rpy.c`:

```c
/*
 * OctroBot - RPY (roll-pitch-yaw) rotation helper tests
 * SPDX-License-Identifier: Apache-2.0
 *
 * mat3x3_from_rpy uses the ZYX intrinsic convention:
 *   R = Rz(yaw) * Ry(pitch) * Rx(roll)
 * Inputs are in radians.
 */

#include <zephyr/ztest.h>
#include "kinematics_math.h"
#include "matrix_exp.h"
#include <math.h>

#define RPY_TOL 1e-5f

/* Build a rotation about a unit axis via the screw exponential (independent
 * reference path, already validated by the matrix_exp suite). */
static mat3x3_t rot_axis(float ax, float ay, float az, float theta)
{
	vec3_t axis = vec3_create(ax, ay, az);
	return matrix_exp_so3(&axis, theta);
}

ZTEST(rpy, test_zero_is_identity)
{
	mat3x3_t R = mat3x3_from_rpy(0.0f, 0.0f, 0.0f);
	mat3x3_t I = mat3x3_identity();
	zassert_true(mat3x3_is_equal(&R, &I, RPY_TOL), "rpy(0,0,0) should be I");
}

ZTEST(rpy, test_single_axis_matches_elementary)
{
	/* roll only -> Rx, pitch only -> Ry, yaw only -> Rz */
	float a = (float)(M_PI / 3.0); /* 60 deg */

	mat3x3_t Rroll = mat3x3_from_rpy(a, 0.0f, 0.0f);
	mat3x3_t Rx = rot_axis(1, 0, 0, a);
	zassert_true(mat3x3_is_equal(&Rroll, &Rx, RPY_TOL), "roll -> Rx");

	mat3x3_t Rpitch = mat3x3_from_rpy(0.0f, a, 0.0f);
	mat3x3_t Ry = rot_axis(0, 1, 0, a);
	zassert_true(mat3x3_is_equal(&Rpitch, &Ry, RPY_TOL), "pitch -> Ry");

	mat3x3_t Ryaw = mat3x3_from_rpy(0.0f, 0.0f, a);
	mat3x3_t Rz = rot_axis(0, 0, 1, a);
	zassert_true(mat3x3_is_equal(&Ryaw, &Rz, RPY_TOL), "yaw -> Rz");
}

ZTEST(rpy, test_matches_zyx_composition)
{
	/* For several triples: mat3x3_from_rpy == Rz(yaw)*Ry(pitch)*Rx(roll). */
	const float triples[][3] = {
		{ 0.1f, -0.2f, 0.3f },
		{ -0.5f, 0.4f, -0.6f },
		{ 1.0f, 0.7f, -1.2f },
	};
	for (int i = 0; i < 3; i++) {
		float r = triples[i][0], p = triples[i][1], y = triples[i][2];
		mat3x3_t R = mat3x3_from_rpy(r, p, y);

		mat3x3_t Rx = rot_axis(1, 0, 0, r);
		mat3x3_t Ry = rot_axis(0, 1, 0, p);
		mat3x3_t Rz = rot_axis(0, 0, 1, y);
		mat3x3_t RzRy = mat3x3_mul(&Rz, &Ry);
		mat3x3_t expected = mat3x3_mul(&RzRy, &Rx);

		zassert_true(mat3x3_is_equal(&R, &expected, RPY_TOL),
			     "triple %d: rpy != Rz*Ry*Rx", i);
	}
}

ZTEST(rpy, test_orthonormal)
{
	mat3x3_t R = mat3x3_from_rpy(0.3f, -0.9f, 1.1f);
	mat3x3_t Rt = mat3x3_transpose(&R);
	mat3x3_t RRt = mat3x3_mul(&R, &Rt);
	mat3x3_t I = mat3x3_identity();
	zassert_true(mat3x3_is_equal(&RRt, &I, RPY_TOL),
		     "R*R^T should be I (orthonormal)");
}

ZTEST_SUITE(rpy, NULL, NULL, NULL, NULL, NULL);
```

- [ ] **Step 2: Register the test in CMake**

In `tests/kinematics/CMakeLists.txt`, add to the "Test sources" `target_sources` block (after the last existing `src/test_*.c` line):

```cmake
  src/test_rpy.c
```

- [ ] **Step 3: Run test to verify it fails**

Run: `make test-native`
Expected: FAIL — build error, `mat3x3_from_rpy` undeclared.

- [ ] **Step 4: Declare `mat3x3_from_rpy` in the header**

In `app/include/kinematics_math.h`, add this declaration in the Mat3x3 section, immediately after the `mat3x3_skew_symmetric` declaration:

```c
/**
 * Rotation matrix from roll-pitch-yaw (ZYX intrinsic) Euler angles.
 *
 * R = Rz(yaw) * Ry(pitch) * Rx(roll), angles in radians.
 */
mat3x3_t mat3x3_from_rpy(float roll, float pitch, float yaw);
```

- [ ] **Step 5: Implement `mat3x3_from_rpy`**

In `app/src/kinematics/kinematics_math.c`, add this function next to the other `mat3x3_*` functions (e.g. after `mat3x3_skew_symmetric`):

```c
mat3x3_t mat3x3_from_rpy(float roll, float pitch, float yaw)
{
	float cr = cosf(roll),  sr = sinf(roll);
	float cp = cosf(pitch), sp = sinf(pitch);
	float cy = cosf(yaw),   sy = sinf(yaw);

	mat3x3_t R;
	/* R = Rz(yaw) * Ry(pitch) * Rx(roll) */
	R.m[0][0] = cy * cp;
	R.m[0][1] = cy * sp * sr - sy * cr;
	R.m[0][2] = cy * sp * cr + sy * sr;

	R.m[1][0] = sy * cp;
	R.m[1][1] = sy * sp * sr + cy * cr;
	R.m[1][2] = sy * sp * cr - cy * sr;

	R.m[2][0] = -sp;
	R.m[2][1] = cp * sr;
	R.m[2][2] = cp * cr;

	return R;
}
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `make test-native`
Expected: PASS — suite `rpy` (4 tests) passes; all pre-existing suites still pass.

- [ ] **Step 7: Commit**

```bash
git add app/include/kinematics_math.h app/src/kinematics/kinematics_math.c \
        tests/kinematics/src/test_rpy.c tests/kinematics/CMakeLists.txt
git commit -m "feat(kinematics): add mat3x3_from_rpy (ZYX intrinsic) helper"
```

---

## Task 2: `joint_map` model↔servo calibration module

**Files:**
- Create: `app/include/joint_map.h`
- Create: `app/src/controller/joint_map.c`
- Create: `tests/kinematics/src/test_joint_map.c`
- Modify: `tests/kinematics/CMakeLists.txt`
- Modify: `app/CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/kinematics/src/test_joint_map.c`:

```c
/*
 * OctroBot - joint_map (model radians <-> servo degrees) tests
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include "joint_map.h"
#include "kinematics_math.h"   /* deg_to_rad / rad_to_deg */
#include <math.h>

#define JM_TOL 1e-3f

/* Reset to identity calibration before every test (module state is static). */
static void jm_reset(void *unused)
{
	ARG_UNUSED(unused);
	const float sign[6]   = { 1, 1, 1, 1, 1, 1 };
	const float offset[6] = { 0, 0, 0, 0, 0, 0 };
	joint_map_set_calibration(sign, offset);
}

ZTEST(joint_map, test_round_trip_identity)
{
	const float theta[6] = { 0.1f, -0.3f, 0.5f, -0.2f, 0.4f, -0.6f };
	float servo[6], back[6];
	joint_map_model_to_servo(theta, servo);
	joint_map_servo_to_model(servo, back);
	for (int i = 0; i < 6; i++) {
		zassert_within(back[i], theta[i], 1e-5f,
			       "joint %d round-trip", i);
	}
}

ZTEST(joint_map, test_identity_known_values)
{
	float theta[6] = { 0, 0, 0, 0, 0, 0 };
	float servo[6];
	joint_map_model_to_servo(theta, servo);
	for (int i = 0; i < 6; i++) {
		zassert_within(servo[i], 0.0f, JM_TOL, "zero -> 0 deg");
	}

	theta[0] = (float)(M_PI / 2.0); /* 90 deg */
	joint_map_model_to_servo(theta, servo);
	zassert_within(servo[0], 90.0f, JM_TOL, "pi/2 rad -> 90 deg");
}

ZTEST(joint_map, test_sign_offset_known_value)
{
	const float sign[6]   = { 1, -1, 1, 1, 1, 1 };
	const float offset[6] = { 0, 90, 0, 0, 0, 0 };
	joint_map_set_calibration(sign, offset);

	/* theta1 = 0.30 rad = 17.1887 deg ; servo1 = -1*17.1887 + 90 = 72.8113 */
	const float theta[6] = { 0, 0.30f, 0, 0, 0, 0 };
	float servo[6];
	joint_map_model_to_servo(theta, servo);
	zassert_within(servo[1], 72.8113f, JM_TOL, "sign+offset applied");
}

ZTEST(joint_map, test_round_trip_non_identity)
{
	const float sign[6]   = { 1, -1, 1, -1, 1, -1 };
	const float offset[6] = { 10, 90, -45, 30, 0, 5 };
	joint_map_set_calibration(sign, offset);

	const float theta[6] = { 0.1f, -0.3f, 0.5f, -0.2f, 0.4f, -0.6f };
	float servo[6], back[6];
	joint_map_model_to_servo(theta, servo);
	joint_map_servo_to_model(servo, back);
	for (int i = 0; i < 6; i++) {
		zassert_within(back[i], theta[i], 1e-5f,
			       "joint %d non-identity round-trip", i);
	}
}

ZTEST_SUITE(joint_map, NULL, NULL, jm_reset, NULL, NULL);
```

(Note: `jm_reset` is the per-test `before` hook, so each test starts from identity.)

- [ ] **Step 2: Register sources in CMake**

In `tests/kinematics/CMakeLists.txt`, add to the "Application kinematics sources under test" `target_sources` block:

```cmake
  ${APP_SRC_DIR}/src/controller/joint_map.c
```

and to the "Test sources" block:

```cmake
  src/test_joint_map.c
```

In `app/CMakeLists.txt`, add to the "Motion controller" `target_sources` block (next to `src/controller/servo_control.c`):

```cmake
  src/controller/joint_map.c
```

- [ ] **Step 3: Run test to verify it fails**

Run: `make test-native`
Expected: FAIL — build error, `joint_map.h` not found.

- [ ] **Step 4: Create the header**

Create `app/include/joint_map.h`:

```c
/*
 * OctroBot Robot Arm Firmware - Joint Calibration Map
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Maps kinematic-model joint angles (radians, model frame, theta=0 => home
 * config M) to physical servo angles (degrees, servo center = 0 deg) and back,
 * via a per-joint sign + offset calibration table:
 *
 *   servo_deg[i] = sign[i] * rad_to_deg(theta_rad[i]) + offset_deg[i]
 *
 * The table defaults to identity (sign=+1, offset=0) and is set at runtime via
 * joint_map_set_calibration (used by tests now; the hook for NVS field
 * calibration later).
 */

#ifndef JOINT_MAP_H
#define JOINT_MAP_H

#include "robot_geometry.h"   /* NUM_JOINTS */

#ifdef __cplusplus
extern "C" {
#endif

/** model radians -> servo degrees (forward). */
void joint_map_model_to_servo(const float theta_rad[NUM_JOINTS],
			      float servo_deg[NUM_JOINTS]);

/** servo degrees -> model radians (inverse). */
void joint_map_servo_to_model(const float servo_deg[NUM_JOINTS],
			      float theta_rad[NUM_JOINTS]);

/** Install a calibration table (copies both arrays). */
void joint_map_set_calibration(const float sign[NUM_JOINTS],
			       const float offset_deg[NUM_JOINTS]);

#ifdef __cplusplus
}
#endif

#endif /* JOINT_MAP_H */
```

- [ ] **Step 5: Implement the module**

Create `app/src/controller/joint_map.c`:

```c
/*
 * OctroBot Robot Arm Firmware - Joint Calibration Map Implementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "joint_map.h"
#include "kinematics_math.h"   /* deg_to_rad / rad_to_deg */
#include <string.h>

static float s_sign[NUM_JOINTS]   = { 1, 1, 1, 1, 1, 1 };
static float s_offset_deg[NUM_JOINTS] = { 0, 0, 0, 0, 0, 0 };

void joint_map_model_to_servo(const float theta_rad[NUM_JOINTS],
			      float servo_deg[NUM_JOINTS])
{
	for (int i = 0; i < NUM_JOINTS; i++) {
		servo_deg[i] = s_sign[i] * rad_to_deg(theta_rad[i]) +
			       s_offset_deg[i];
	}
}

void joint_map_servo_to_model(const float servo_deg[NUM_JOINTS],
			      float theta_rad[NUM_JOINTS])
{
	for (int i = 0; i < NUM_JOINTS; i++) {
		/* sign[i] is +/-1, so dividing by sign == multiplying. */
		theta_rad[i] = deg_to_rad((servo_deg[i] - s_offset_deg[i]) *
					  s_sign[i]);
	}
}

void joint_map_set_calibration(const float sign[NUM_JOINTS],
			       const float offset_deg[NUM_JOINTS])
{
	memcpy(s_sign, sign, sizeof(s_sign));
	memcpy(s_offset_deg, offset_deg, sizeof(s_offset_deg));
}
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `make test-native`
Expected: PASS — suite `joint_map` (4 tests) passes; all prior suites still pass.

- [ ] **Step 7: Commit**

```bash
git add app/include/joint_map.h app/src/controller/joint_map.c \
        tests/kinematics/src/test_joint_map.c \
        tests/kinematics/CMakeLists.txt app/CMakeLists.txt
git commit -m "feat(controller): add joint_map model<->servo calibration module"
```

---

## Task 3: `cartesian_pose_to_joints` pure core

**Files:**
- Create: `app/include/cartesian_pose.h`
- Create: `app/src/controller/cartesian_pose.c`
- Create: `tests/kinematics/src/test_cartesian_pose.c`
- Modify: `tests/kinematics/CMakeLists.txt`
- Modify: `app/CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/kinematics/src/test_cartesian_pose.c`:

```c
/*
 * OctroBot - cartesian_pose_to_joints (pure pose->joints core) tests
 * SPDX-License-Identifier: Apache-2.0
 *
 * Round-trip: pick known joint angles -> FK -> pose (position + ZYX RPY) ->
 * cartesian_pose_to_joints -> out_theta -> FK(out_theta) must match the pose.
 */

#include <zephyr/ztest.h>
#include "kinematics_math.h"
#include "robot_geometry.h"
#include "forward_kinematics_poe.h"
#include "cartesian_pose.h"
#include <math.h>

#define CP_POSE_TOL 5e-3f

struct cartesian_pose_fixture {
	poe_robot_model_t model;
};

static void *cp_setup(void)
{
	static struct cartesian_pose_fixture fixture;
	fixture.model = robot_geometry_factory_defaults();
	return &fixture;
}

/* Extract ZYX intrinsic roll/pitch/yaw (radians) from a rotation matrix. */
static void rpy_from_mat3(const mat3x3_t *R, float *roll, float *pitch,
			  float *yaw)
{
	*yaw = atan2f(R->m[1][0], R->m[0][0]);
	*pitch = atan2f(-R->m[2][0],
			sqrtf(R->m[2][1] * R->m[2][1] + R->m[2][2] * R->m[2][2]));
	*roll = atan2f(R->m[2][1], R->m[2][2]);
}

ZTEST_F(cartesian_pose, test_round_trip)
{
	const float theta_true[NUM_JOINTS] = {
		0.2f, -0.3f, 0.4f, -0.2f, 0.3f, -0.1f
	};
	mat4x4_t T_true;
	zassert_true(forward_kinematics_compute(&fixture->model, theta_true,
						&T_true), "FK setup");

	vec3_t p = mat4x4_get_translation(&T_true);
	mat3x3_t R = mat4x4_get_rotation(&T_true);
	float roll, pitch, yaw;
	rpy_from_mat3(&R, &roll, &pitch, &yaw);

	const float seed[NUM_JOINTS] = {
		0.1f, -0.2f, 0.3f, -0.1f, 0.2f, 0.0f
	};
	float out[NUM_JOINTS] = {0};
	cmove_status_t st = cartesian_pose_to_joints(
		&fixture->model, p.x, p.y, p.z,
		rad_to_deg(roll), rad_to_deg(pitch), rad_to_deg(yaw),
		seed, out);
	zassert_equal(st, CMOVE_OK, "expected CMOVE_OK, got %d", st);

	mat4x4_t T_check;
	zassert_true(forward_kinematics_compute(&fixture->model, out, &T_check),
		     "FK recheck");
	zassert_true(mat4x4_is_equal(&T_check, &T_true, CP_POSE_TOL),
		     "FK(pose_to_joints(pose)) should match pose");
}

ZTEST_F(cartesian_pose, test_null_args)
{
	const float seed[NUM_JOINTS] = {0};
	float out[NUM_JOINTS] = {0};
	zassert_equal(cartesian_pose_to_joints(NULL, 0, 0, 0, 0, 0, 0, seed, out),
		      CMOVE_BAD_ARGS, "NULL model");
	zassert_equal(cartesian_pose_to_joints(&fixture->model, 0, 0, 0, 0, 0, 0,
					       NULL, out),
		      CMOVE_BAD_ARGS, "NULL seed");
	zassert_equal(cartesian_pose_to_joints(&fixture->model, 0, 0, 0, 0, 0, 0,
					       seed, NULL),
		      CMOVE_BAD_ARGS, "NULL out");
}

ZTEST_F(cartesian_pose, test_unreachable)
{
	const float seed[NUM_JOINTS] = {0};
	float out[NUM_JOINTS] = {0};
	/* 5 m away is far outside the ~0.3 m workspace. */
	cmove_status_t st = cartesian_pose_to_joints(
		&fixture->model, 5.0f, 5.0f, 5.0f, 0, 0, 0, seed, out);
	zassert_equal(st, CMOVE_NO_SOLUTION,
		      "unreachable pose should be CMOVE_NO_SOLUTION, got %d", st);
}

ZTEST_SUITE(cartesian_pose, NULL, cp_setup, NULL, NULL, NULL);
```

- [ ] **Step 2: Register sources in CMake**

In `tests/kinematics/CMakeLists.txt`, add to the "Application kinematics sources under test" block:

```cmake
  ${APP_SRC_DIR}/src/controller/cartesian_pose.c
```

and to the "Test sources" block:

```cmake
  src/test_cartesian_pose.c
```

In `app/CMakeLists.txt`, add to the "Motion controller" block:

```cmake
  src/controller/cartesian_pose.c
```

- [ ] **Step 3: Run test to verify it fails**

Run: `make test-native`
Expected: FAIL — build error, `cartesian_pose.h` not found.

- [ ] **Step 4: Create the header**

Create `app/include/cartesian_pose.h`:

```c
/*
 * OctroBot Robot Arm Firmware - Cartesian Pose -> Joints (pure core)
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure (hardware-free) conversion of a commanded Cartesian pose into
 * model-frame joint angles via the IK solver. RPY uses the ZYX intrinsic
 * convention; angles in degrees, position in meters.
 */

#ifndef CARTESIAN_POSE_H
#define CARTESIAN_POSE_H

#include "robot_geometry.h"   /* poe_robot_model_t, NUM_JOINTS */

#ifdef __cplusplus
extern "C" {
#endif

/** Result of a Cartesian move-to-pose request. */
typedef enum {
	CMOVE_OK = 0,         /* solved (and, in the wrapper, motion sent)     */
	CMOVE_NO_SOLUTION,    /* IK did not converge                           */
	CMOVE_OUT_OF_LIMITS,  /* solution violates joint/servo limits          */
	CMOVE_ESTOP,          /* emergency stop active (wrapper only)          */
	CMOVE_SERVO_ERR,      /* servo read/write failure (wrapper only)       */
	CMOVE_BAD_ARGS,       /* NULL/invalid argument                         */
} cmove_status_t;

/**
 * Compute model-frame joint angles for a Cartesian pose.
 *
 * @param model       robot geometry
 * @param x,y,z       target position in meters (model frame)
 * @param roll_deg,pitch_deg,yaw_deg  target orientation, ZYX intrinsic, degrees
 * @param seed_theta  IK initial guess (model radians)
 * @param out_theta   solved joint angles (model radians)
 * @return CMOVE_OK / CMOVE_NO_SOLUTION / CMOVE_OUT_OF_LIMITS / CMOVE_BAD_ARGS
 */
cmove_status_t cartesian_pose_to_joints(const poe_robot_model_t *model,
					float x, float y, float z,
					float roll_deg, float pitch_deg,
					float yaw_deg,
					const float seed_theta[NUM_JOINTS],
					float out_theta[NUM_JOINTS]);

#ifdef __cplusplus
}
#endif

#endif /* CARTESIAN_POSE_H */
```

- [ ] **Step 5: Implement the core**

Create `app/src/controller/cartesian_pose.c`:

```c
/*
 * OctroBot Robot Arm Firmware - Cartesian Pose -> Joints (pure core)
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "cartesian_pose.h"
#include "kinematics_math.h"
#include "inverse_kinematics_poe.h"

cmove_status_t cartesian_pose_to_joints(const poe_robot_model_t *model,
					float x, float y, float z,
					float roll_deg, float pitch_deg,
					float yaw_deg,
					const float seed_theta[NUM_JOINTS],
					float out_theta[NUM_JOINTS])
{
	if (model == NULL || seed_theta == NULL || out_theta == NULL) {
		return CMOVE_BAD_ARGS;
	}

	/* Build the SE(3) target T_sd from RPY + position. */
	mat3x3_t R = mat3x3_from_rpy(deg_to_rad(roll_deg),
				     deg_to_rad(pitch_deg),
				     deg_to_rad(yaw_deg));
	vec3_t p = vec3_create(x, y, z);
	mat4x4_t T_target = mat4x4_from_rt(&R, &p);

	float out[NUM_JOINTS];
	ik_status_t ik = inverse_kinematics_compute(model, &T_target,
						    seed_theta, out, NULL, NULL);

	switch (ik) {
	case IK_SUCCESS:
		for (int i = 0; i < NUM_JOINTS; i++) {
			out_theta[i] = out[i];
		}
		return CMOVE_OK;
	case IK_OUT_OF_LIMITS:
		for (int i = 0; i < NUM_JOINTS; i++) {
			out_theta[i] = out[i];
		}
		return CMOVE_OUT_OF_LIMITS;
	case IK_NO_CONVERGENCE:
		return CMOVE_NO_SOLUTION;
	case IK_INVALID_INPUT:
	default:
		return CMOVE_BAD_ARGS;
	}
}
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `make test-native`
Expected: PASS — suite `cartesian_pose` (3 tests) passes; all prior suites still pass.

- [ ] **Step 7: Commit**

```bash
git add app/include/cartesian_pose.h app/src/controller/cartesian_pose.c \
        tests/kinematics/src/test_cartesian_pose.c \
        tests/kinematics/CMakeLists.txt app/CMakeLists.txt
git commit -m "feat(controller): add cartesian_pose_to_joints pure core"
```

---

## Task 4: `cartesian_move_to_pose` hardware wrapper

**Files:**
- Create: `app/include/cartesian_move.h`
- Create: `app/src/controller/cartesian_move.c`
- Modify: `app/CMakeLists.txt`

No native unit test — this unit talks to servos and the e-stop GPIO, so it is
validated on hardware (Task 6). It is NOT added to the test CMakeLists.

- [ ] **Step 1: Create the header**

Create `app/include/cartesian_move.h`:

```c
/*
 * OctroBot Robot Arm Firmware - Cartesian Move-to-Pose (hardware wrapper)
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Drives the arm to a commanded Cartesian pose: checks e-stop, reads the
 * current servo angles for the IK seed, runs cartesian_pose_to_joints, maps the
 * solution to servo degrees, range-checks, and sync-writes goal angles with an
 * onboard move time. RPY is ZYX intrinsic (degrees); position in meters.
 */

#ifndef CARTESIAN_MOVE_H
#define CARTESIAN_MOVE_H

#include <stdint.h>
#include "cartesian_pose.h"   /* cmove_status_t */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Move the arm to a Cartesian pose.
 *
 * @param x,y,z       target position in meters (model frame)
 * @param roll_deg,pitch_deg,yaw_deg  target orientation, ZYX intrinsic, degrees
 * @param move_time_ms  servo move duration in ms (onboard smoothing)
 * @return cmove_status_t (CMOVE_OK on success)
 */
cmove_status_t cartesian_move_to_pose(float x, float y, float z,
				      float roll_deg, float pitch_deg,
				      float yaw_deg, uint16_t move_time_ms);

#ifdef __cplusplus
}
#endif

#endif /* CARTESIAN_MOVE_H */
```

- [ ] **Step 2: Implement the wrapper**

Create `app/src/controller/cartesian_move.c`:

```c
/*
 * OctroBot Robot Arm Firmware - Cartesian Move-to-Pose (hardware wrapper)
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "cartesian_move.h"
#include "cartesian_pose.h"
#include "joint_map.h"
#include "robot_geometry.h"
#include "feetech_servo.h"
#include "hal_gpio.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cartesian_move, LOG_LEVEL_WRN);

#define SERVO_DEG_LIMIT 180.0f

static const uint8_t k_ids[NUM_JOINTS] = { 1, 2, 3, 4, 5, 6 };

cmove_status_t cartesian_move_to_pose(float x, float y, float z,
				      float roll_deg, float pitch_deg,
				      float yaw_deg, uint16_t move_time_ms)
{
	/* 1. Refuse to move while the e-stop button is pressed. */
	if (hal_gpio_button_is_pressed()) {
		LOG_WRN("movec refused: emergency stop active");
		return CMOVE_ESTOP;
	}

	const poe_robot_model_t *model = robot_geometry_get_model();

	/* 2. Seed IK from the current servo angles (fall back to zeros). */
	float seed_theta[NUM_JOINTS] = { 0 };
	uint16_t positions[NUM_JOINTS];
	if (feetech_servo_read_multi_positions(k_ids, positions, NUM_JOINTS) ==
	    0) {
		float servo_deg[NUM_JOINTS];
		for (int i = 0; i < NUM_JOINTS; i++) {
			servo_deg[i] = FEETECH_POS_TO_DEG(positions[i]);
		}
		joint_map_servo_to_model(servo_deg, seed_theta);
	} else {
		LOG_WRN("movec: servo read failed, using zero seed");
	}

	/* 3. Solve IK for the target pose. */
	float out_theta[NUM_JOINTS];
	cmove_status_t st = cartesian_pose_to_joints(model, x, y, z, roll_deg,
						     pitch_deg, yaw_deg,
						     seed_theta, out_theta);
	if (st != CMOVE_OK) {
		return st;
	}

	/* 4. Map to servo degrees and range-check. */
	float servo_goal[NUM_JOINTS];
	joint_map_model_to_servo(out_theta, servo_goal);
	for (int i = 0; i < NUM_JOINTS; i++) {
		if (servo_goal[i] < -SERVO_DEG_LIMIT ||
		    servo_goal[i] > SERVO_DEG_LIMIT) {
			LOG_WRN("movec: joint %d servo angle %.1f out of range",
				i, (double)servo_goal[i]);
			return CMOVE_OUT_OF_LIMITS;
		}
	}

	/* 5. Apply move time, then sync-write goal angles. */
	for (int i = 0; i < NUM_JOINTS; i++) {
		(void)feetech_servo_set_goal_time(k_ids[i], move_time_ms);
	}
	if (feetech_servo_sync_write_angles(k_ids, servo_goal, NUM_JOINTS) != 0) {
		LOG_WRN("movec: sync write failed");
		return CMOVE_SERVO_ERR;
	}

	return CMOVE_OK;
}
```

- [ ] **Step 3: Register the source in the firmware build**

In `app/CMakeLists.txt`, add to the "Motion controller" block:

```cmake
  src/controller/cartesian_move.c
```

- [ ] **Step 4: Verify the firmware builds**

Run: `make build`
Expected: SUCCESS — the firmware image compiles with the new module linked.
(If `make build` requires hardware-side Zephyr config not available in this
environment, instead run `make test-native` to confirm nothing regressed and
note that `make build` should be run before flashing.)

- [ ] **Step 5: Commit**

```bash
git add app/include/cartesian_move.h app/src/controller/cartesian_move.c \
        app/CMakeLists.txt
git commit -m "feat(controller): add cartesian_move_to_pose hardware wrapper"
```

---

## Task 5: `$movec` console command

**Files:**
- Modify: `app/src/comms/uart_console.c`

- [ ] **Step 1: Include the wrapper header**

In `app/src/comms/uart_console.c`, add near the existing includes (with the
other project headers):

```c
#include "cartesian_move.h"
```

- [ ] **Step 2: Add the `$movec` branch to the command dispatcher**

In `uart_console_process_command`, add a new `else if` branch in the command
if/else chain (e.g. immediately after the `set_angles` branch). Use the exact
existing style (`strcmp`, `atof`, `LOG_WRN` on bad input, return `-EINVAL`):

```c
	/* Cartesian move-to-pose command:
	 *   $movec X Y Z ROLL PITCH YAW [TIME_MS]
	 *   X Y Z in meters; ROLL PITCH YAW in degrees (ZYX); TIME_MS default 1000.
	 */
	else if (strcmp(command, "movec") == 0) {
		if (argc != 7 && argc != 8) {
			LOG_WRN("Invalid 'movec' command format. Expected: "
				"$movec X Y Z ROLL PITCH YAW [TIME_MS]");
			return -EINVAL;
		}
		float x = atof(argv[1]);
		float y = atof(argv[2]);
		float z = atof(argv[3]);
		float roll = atof(argv[4]);
		float pitch = atof(argv[5]);
		float yaw = atof(argv[6]);
		uint16_t time_ms = (argc == 8) ? (uint16_t)atoi(argv[7]) : 1000;

		cmove_status_t st = cartesian_move_to_pose(x, y, z, roll, pitch,
							   yaw, time_ms);
		if (st != CMOVE_OK) {
			LOG_WRN("movec failed: status %d", st);
			return -EIO;
		}
		LOG_INF("movec: moving to [%.3f %.3f %.3f] rpy[%.1f %.1f %.1f] "
			"in %u ms", (double)x, (double)y, (double)z,
			(double)roll, (double)pitch, (double)yaw, time_ms);
	}
```

- [ ] **Step 3: Verify the firmware builds**

Run: `make build`
Expected: SUCCESS — firmware compiles with the `$movec` command.
(If `make build` cannot run in this environment, run `make test-native` to
confirm no regression and note `make build` must pass before flashing.)

- [ ] **Step 4: Commit**

```bash
git add app/src/comms/uart_console.c
git commit -m "feat(comms): add \$movec Cartesian move-to-pose console command"
```

---

## Task 6: Full validation + docs + hardware bring-up notes

**Files:**
- Modify: `CLAUDE.md`
- Create: `docs/CARTESIAN_MOVE_README.md`

- [ ] **Step 1: Run the full native + Python test suite**

Run: `make test`
Expected: PASS — native suites include `rpy`, `joint_map`, `cartesian_pose`
(plus all prior suites); Python tests still pass. If anything fails, stop and
report it (do not patch over a real failure).

- [ ] **Step 2: Confirm the firmware builds**

Run: `make build`
Expected: SUCCESS. (If unavailable in this environment, note it must be run
before flashing.)

- [ ] **Step 3: Update the phase roadmap in `CLAUDE.md`**

In `CLAUDE.md`, in the "Phase Roadmap" table, change the Phase 6 row from:

```
| 6     | ❌ Partial | Motion controller (1 ms loop, PID) |
```

to:

```
| 6     | ⏳ In Progress | Cartesian move-to-pose (IK→servo via $movec) done; servos do onboard PID, no MCU trajectory loop yet |
```

And in the architecture diagram near the top, change the Motion Controller line
from:

```
Motion Controller  (Phase 6 - partial placeholder)
```

to:

```
Motion Controller  (Phase 6 - Cartesian move-to-pose via IK; servo onboard motion)
```

(Match the exact existing text before editing; adapt if the wording differs.)

- [ ] **Step 4: Write the hardware bring-up README**

Create `docs/CARTESIAN_MOVE_README.md`:

```markdown
# Cartesian Move-to-Pose ($movec)

Move the end-effector to a Cartesian pose. The firmware runs body-frame IK and
sends the resulting joint angles to the servos with an onboard move time.

## Command

```
$movec X Y Z ROLL PITCH YAW [TIME_MS]
```

- `X Y Z` — target position in **meters** (model frame; the home position is
  roughly `0.168 0 0.243`).
- `ROLL PITCH YAW` — target orientation in **degrees**, ZYX intrinsic
  (`R = Rz(yaw)·Ry(pitch)·Rx(roll)`).
- `TIME_MS` — optional servo move duration in ms (default **1000**).

Example (move to the home pose orientation, slightly forward, over 2 s):

```
$movec 0.20 0.00 0.243 -90 0 -180 2000
```

## Joint calibration (REQUIRED before trusting motion)

The kinematic model's joint zero is not the servo center. Until calibrated, the
`joint_map` table is identity (`sign=+1, offset=0`), so the arm will very likely
go to the wrong pose. Calibrate by:

1. Command the model home pose: `$set_angles` each joint so the arm is in the
   kinematic home configuration, and note each servo's reported angle (`$read`)
   — those become `offset_deg[i]`.
2. Jog each joint in the model's positive direction; if the servo angle
   decreases, that joint needs `sign[i] = -1`.
3. Put the twelve numbers into `s_sign` / `s_offset_deg` in
   `app/src/controller/joint_map.c` (or call `joint_map_set_calibration`).

## Statuses

`$movec` reports a non-zero status on failure: no IK solution, out of limits,
emergency stop active, servo error, or bad arguments.

## Safety

Press the e-stop button (GPIO 39) to disable torque. `$movec` refuses to start a
move while the e-stop button is held.
```

- [ ] **Step 5: Commit**

```bash
git add CLAUDE.md docs/CARTESIAN_MOVE_README.md
git commit -m "docs: mark Phase 6 Cartesian move-to-pose done; add bring-up README"
```

- [ ] **Step 6: Hardware validation (manual, on device)**

This is a manual checklist for the developer with hardware — not an automated
step:

1. `make bfm` (build, flash, monitor).
2. Power the arm; confirm servos respond (`$read`).
3. Calibrate `joint_map` per the README.
4. Send a small move near the current pose, e.g. `$movec 0.18 0 0.243 -90 0 -180 2000`.
5. Confirm the arm moves smoothly to the pose; verify with `$read` + a mental FK
   check, or by eye.
6. Test failure paths: an unreachable target (e.g. `$movec 5 5 5 0 0 0`) reports
   a non-zero status and no motion; holding the e-stop button blocks motion.

---

## Self-Review Notes

- **Spec coverage:** `mat3x3_from_rpy` (Task 1), `joint_map` + setter (Task 2),
  `cartesian_pose_to_joints` core (Task 3), `cartesian_move_to_pose` wrapper with
  e-stop/seed/range-check/sync-write (Task 4), `$movec` console command (Task 5),
  full test pass + docs + hardware checklist (Task 6). All spec sections map to a
  task.
- **Deviation from spec (intentional):** the spec described one `cartesian_move`
  module with a core + wrapper; the plan splits it into `cartesian_pose.c` (pure,
  unit-tested) and `cartesian_move.c` (hardware) so the core compiles into the
  native test target without servo/HAL stubs. The `cmove_status_t` enum lives in
  `cartesian_pose.h`.
- **RPY validation:** done in-C via known single-axis cases, orthonormality, and
  a self-composition cross-check against `matrix_exp_so3` (`Rz·Ry·Rx`) — an
  independent path already validated by the `matrix_exp` suite. No new Python
  vectors needed; this is stronger and simpler than the spec's "Python RPY
  cross-check" note.
- **Type consistency:** `cmove_status_t` enum and the `cartesian_pose_to_joints`
  / `cartesian_move_to_pose` signatures are identical across headers, sources,
  tests, and the console call. `NUM_JOINTS` (6) used throughout. Servo IDs 1–6.
- **No placeholders:** every code/command step is concrete.
```
