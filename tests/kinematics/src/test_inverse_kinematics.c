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

#define IK_POSE_TOL 5e-3f

struct inverse_kinematics_fixture {
	poe_robot_model_t model;
};

static void *ik_setup(void)
{
	static struct inverse_kinematics_fixture fixture;
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
	zassert_equal(p.max_iters, 50, "max_iters default");
	zassert_true(is_near_equal(p.lambda, 0.01f, 1e-9f), "lambda default");
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

ZTEST_SUITE(inverse_kinematics, NULL, ik_setup, NULL, NULL, NULL);
