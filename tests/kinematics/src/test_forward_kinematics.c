/*
 * OctroBot - Forward Kinematics Unit Tests
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tests: forward_kinematics_compute, _compute_pose, _partial.
 * Uses factory default geometry from robot_geometry_factory_defaults().
 */

#include <zephyr/ztest.h>
#include "kinematics_math.h"
#include "robot_geometry.h"
#include "forward_kinematics_poe.h"
#include <math.h>
#include <string.h>

#define FK_TOL 1e-4f

/* ======================================================================== */
/* Shared fixture: factory-default robot model                              */
/* ======================================================================== */

struct forward_kinematics_fixture {
	poe_robot_model_t model;
};

static void *fk_setup(void)
{
	static struct forward_kinematics_fixture fixture;
	fixture.model = robot_geometry_factory_defaults();
	return &fixture;
}

/* ======================================================================== */
/* Zero configuration tests                                                 */
/* ======================================================================== */

ZTEST_F(forward_kinematics, test_zero_config_equals_home)
{
	/* FK(0,0,0,0,0,0) must equal the home configuration M */
	float theta[NUM_JOINTS] = {0};
	mat4x4_t T;
	bool ok = forward_kinematics_compute(&fixture->model, theta, &T);

	zassert_true(ok, "FK compute should succeed");
	zassert_true(mat4x4_is_equal(&T, &fixture->model.M, FK_TOL),
		     "FK at zero config should equal M");
}

ZTEST_F(forward_kinematics, test_zero_config_position)
{
	/* Position at zero config should be [0.5, 0.0, 0.1] */
	float theta[NUM_JOINTS] = {0};
	vec3_t pos;
	mat3x3_t orient;
	bool ok = forward_kinematics_compute_pose(&fixture->model, theta,
						   &pos, &orient);

	zassert_true(ok);
	zassert_true(is_near_equal(pos.x, 0.5f, FK_TOL));
	zassert_true(is_near_equal(pos.y, 0.0f, FK_TOL));
	zassert_true(is_near_equal(pos.z, 0.1f, FK_TOL));
}

/* ======================================================================== */
/* Single-joint rotation tests                                              */
/* ======================================================================== */

ZTEST_F(forward_kinematics, test_base_rotation_90deg)
{
	/* Joint 0: Z-rotation by π/2 at origin.
	 * Home position [0.5, 0, 0.1] → expected [0, 0.5, 0.1]
	 * (x → -y, y → x under 90° Z rotation)
	 */
	float theta[NUM_JOINTS] = {0};
	theta[0] = (float)(M_PI / 2.0);

	vec3_t pos;
	mat3x3_t orient;
	bool ok = forward_kinematics_compute_pose(&fixture->model, theta,
						   &pos, &orient);

	zassert_true(ok);
	zassert_true(is_near_equal(pos.x, 0.0f, FK_TOL),
		     "x should be ~0 after 90 deg Z rotation");
	zassert_true(is_near_equal(pos.y, 0.5f, FK_TOL),
		     "y should be ~0.5 after 90 deg Z rotation");
	zassert_true(is_near_equal(pos.z, 0.1f, FK_TOL),
		     "z should be unchanged");
}

ZTEST_F(forward_kinematics, test_base_rotation_180deg)
{
	/* Joint 0: Z-rotation by π → position [0.5, 0, 0.1] → [-0.5, 0, 0.1] */
	float theta[NUM_JOINTS] = {0};
	theta[0] = (float)M_PI;

	vec3_t pos;
	mat3x3_t orient;
	bool ok = forward_kinematics_compute_pose(&fixture->model, theta,
						   &pos, &orient);

	zassert_true(ok);
	zassert_true(is_near_equal(pos.x, -0.5f, FK_TOL));
	zassert_true(is_near_equal(pos.y, 0.0f, FK_TOL));
	zassert_true(is_near_equal(pos.z, 0.1f, FK_TOL));
}

/* ======================================================================== */
/* Partial chain tests                                                      */
/* ======================================================================== */

ZTEST_F(forward_kinematics, test_partial_chain_joint0_only)
{
	/* Partial FK up to joint 0 with zero angles should equal M */
	float theta[NUM_JOINTS] = {0};
	mat4x4_t T;
	bool ok = forward_kinematics_partial(&fixture->model, theta, 0, &T);

	zassert_true(ok, "Partial FK should succeed");
	/* exp([ξ₀]·0) · M = I · M = M */
	zassert_true(mat4x4_is_equal(&T, &fixture->model.M, FK_TOL));
}

ZTEST_F(forward_kinematics, test_partial_chain_differs)
{
	/* Partial chain to joint 2 vs full chain should generally differ
	 * when joints 3-5 are non-zero
	 */
	float theta[NUM_JOINTS] = {0};
	theta[3] = deg_to_rad(30.0f);

	mat4x4_t T_partial, T_full;
	forward_kinematics_partial(&fixture->model, theta, 2, &T_partial);
	forward_kinematics_compute(&fixture->model, theta, &T_full);

	/* Partial chain ignores joints 3-5, full chain doesn't */
	zassert_false(mat4x4_is_equal(&T_partial, &T_full, FK_TOL),
		      "Partial and full should differ when later joints nonzero");
}

/* ======================================================================== */
/* Null/invalid input tests                                                 */
/* ======================================================================== */

ZTEST(forward_kinematics, test_null_model_returns_false)
{
	float theta[NUM_JOINTS] = {0};
	mat4x4_t T;
	bool ok = forward_kinematics_compute(NULL, theta, &T);
	zassert_false(ok);
}

ZTEST(forward_kinematics, test_null_angles_returns_false)
{
	poe_robot_model_t model = robot_geometry_factory_defaults();
	mat4x4_t T;
	bool ok = forward_kinematics_compute(&model, NULL, &T);
	zassert_false(ok);
}

ZTEST(forward_kinematics, test_null_output_returns_false)
{
	poe_robot_model_t model = robot_geometry_factory_defaults();
	float theta[NUM_JOINTS] = {0};
	bool ok = forward_kinematics_compute(&model, theta, NULL);
	zassert_false(ok);
}

/* ======================================================================== */
/* Consistency tests                                                        */
/* ======================================================================== */

ZTEST_F(forward_kinematics, test_deterministic_output)
{
	/* Same input → identical output, every time */
	float theta[NUM_JOINTS] = {0.1f, -0.2f, 0.3f, -0.4f, 0.5f, -0.6f};
	mat4x4_t T1, T2;

	forward_kinematics_compute(&fixture->model, theta, &T1);
	forward_kinematics_compute(&fixture->model, theta, &T2);

	zassert_true(mat4x4_is_equal(&T1, &T2, 1e-6f),
		     "FK must be deterministic");
}

ZTEST_F(forward_kinematics, test_compute_pose_matches_full)
{
	/* forward_kinematics_compute_pose should extract same position/orientation
	 * as manually extracting from forward_kinematics_compute result
	 */
	float theta[NUM_JOINTS] = {0.3f, -0.1f, 0.5f, 0.0f, -0.2f, 0.4f};
	mat4x4_t T;
	vec3_t pos;
	mat3x3_t orient;

	forward_kinematics_compute(&fixture->model, theta, &T);
	forward_kinematics_compute_pose(&fixture->model, theta, &pos, &orient);

	/* Compare position */
	vec3_t t_from_T = mat4x4_get_translation(&T);
	zassert_true(is_near_equal(pos.x, t_from_T.x, FK_TOL));
	zassert_true(is_near_equal(pos.y, t_from_T.y, FK_TOL));
	zassert_true(is_near_equal(pos.z, t_from_T.z, FK_TOL));

	/* Compare orientation */
	mat3x3_t R_from_T = mat4x4_get_rotation(&T);
	zassert_true(mat3x3_is_equal(&orient, &R_from_T, FK_TOL));
}

ZTEST_F(forward_kinematics, test_fk_output_is_valid_se3)
{
	/* FK output must always be a valid SE(3) transform */
	float theta[NUM_JOINTS] = {1.0f, -0.5f, 0.7f, -1.2f, 0.3f, 0.8f};
	mat4x4_t T;
	forward_kinematics_compute(&fixture->model, theta, &T);

	/* Bottom row = [0, 0, 0, 1] */
	zassert_true(is_near_equal(T.m[3][0], 0.0f, 1e-6f));
	zassert_true(is_near_equal(T.m[3][1], 0.0f, 1e-6f));
	zassert_true(is_near_equal(T.m[3][2], 0.0f, 1e-6f));
	zassert_true(is_near_equal(T.m[3][3], 1.0f, 1e-6f));

	/* R^T R = I (orthogonal rotation) */
	mat3x3_t R = mat4x4_get_rotation(&T);
	mat3x3_t Rt = mat3x3_transpose(&R);
	mat3x3_t product = mat3x3_mul(&Rt, &R);
	mat3x3_t I = mat3x3_identity();
	zassert_true(mat3x3_is_equal(&product, &I, FK_TOL),
		     "R^T R should equal I");
}

/* ======================================================================== */
/* Test suite registration                                                  */
/* ======================================================================== */

ZTEST_SUITE(forward_kinematics, NULL, fk_setup, NULL, NULL, NULL);
