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
