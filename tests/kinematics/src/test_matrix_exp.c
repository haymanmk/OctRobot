/*
 * OctroBot - Matrix Exponential Unit Tests
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tests: matrix_exp_so3, matrix_exp_se3, compute_g_matrix.
 */

#include <zephyr/ztest.h>
#include "kinematics_math.h"
#include "matrix_exp.h"
#include <math.h>

#define TEST_TOL 1e-4f

/* ======================================================================== */
/* SO(3) Matrix Exponential tests                                           */
/* ======================================================================== */

ZTEST(matrix_exp, test_so3_zero_angle_identity)
{
	/* exp([ω] · 0) = I₃ for any ω */
	vec3_t w = vec3_create(0.0f, 0.0f, 1.0f);
	mat3x3_t R = matrix_exp_so3(&w, 0.0f);
	mat3x3_t I = mat3x3_identity();

	zassert_true(mat3x3_is_equal(&R, &I, TEST_TOL),
		     "exp(0) should equal identity");
}

ZTEST(matrix_exp, test_so3_z_rotation_90deg)
{
	/* exp([ω_z] · π/2) = 90° rotation about Z:
	 *  [ 0  -1  0 ]
	 *  [ 1   0  0 ]
	 *  [ 0   0  1 ]
	 */
	vec3_t w = vec3_create(0.0f, 0.0f, 1.0f);
	float theta = (float)(M_PI / 2.0);
	mat3x3_t R = matrix_exp_so3(&w, theta);

	zassert_true(is_near_equal(R.m[0][0], 0.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[0][1], -1.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[0][2], 0.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[1][0], 1.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[1][1], 0.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[1][2], 0.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[2][0], 0.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[2][1], 0.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[2][2], 1.0f, TEST_TOL));
}

ZTEST(matrix_exp, test_so3_x_rotation_90deg)
{
	/* exp([ω_x] · π/2) = 90° rotation about X:
	 *  [ 1  0   0 ]
	 *  [ 0  0  -1 ]
	 *  [ 0  1   0 ]
	 */
	vec3_t w = vec3_create(1.0f, 0.0f, 0.0f);
	float theta = (float)(M_PI / 2.0);
	mat3x3_t R = matrix_exp_so3(&w, theta);

	zassert_true(is_near_equal(R.m[0][0], 1.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[0][1], 0.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[0][2], 0.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[1][0], 0.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[1][1], 0.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[1][2], -1.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[2][0], 0.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[2][1], 1.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[2][2], 0.0f, TEST_TOL));
}

ZTEST(matrix_exp, test_so3_180deg_rotation)
{
	/* exp([ω_z] · π) = 180° rotation about Z:
	 *  [ -1  0   0 ]
	 *  [  0 -1   0 ]
	 *  [  0  0   1 ]
	 */
	vec3_t w = vec3_create(0.0f, 0.0f, 1.0f);
	float theta = (float)M_PI;
	mat3x3_t R = matrix_exp_so3(&w, theta);

	zassert_true(is_near_equal(R.m[0][0], -1.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[0][1], 0.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[1][0], 0.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[1][1], -1.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[2][2], 1.0f, TEST_TOL));
}

ZTEST(matrix_exp, test_so3_rotation_is_orthogonal)
{
	/* R^T R = I for any valid rotation */
	vec3_t w = vec3_create(0.0f, 1.0f, 0.0f);
	float theta = deg_to_rad(37.0f);  /* arbitrary angle */
	mat3x3_t R = matrix_exp_so3(&w, theta);

	mat3x3_t Rt = mat3x3_transpose(&R);
	mat3x3_t product = mat3x3_mul(&Rt, &R);
	mat3x3_t I = mat3x3_identity();

	zassert_true(mat3x3_is_equal(&product, &I, TEST_TOL),
		     "R^T R should equal I");
}

/* ======================================================================== */
/* SE(3) Matrix Exponential tests                                           */
/* ======================================================================== */

ZTEST(matrix_exp, test_se3_zero_angle_identity)
{
	/* exp([ξ] · 0) = I₄ for any screw axis */
	vec6_t xi;
	xi.w = vec3_create(0.0f, 0.0f, 1.0f);
	xi.v = vec3_create(0.0f, 0.0f, 0.0f);

	mat4x4_t T = matrix_exp_se3(&xi, 0.0f);
	mat4x4_t I = mat4x4_identity();

	zassert_true(mat4x4_is_equal(&T, &I, TEST_TOL),
		     "exp(0) should equal I_4");
}

ZTEST(matrix_exp, test_se3_pure_rotation_no_translation)
{
	/* Pure rotation about Z at origin: v = -ω × q = 0 (q at origin) */
	vec6_t xi;
	xi.w = vec3_create(0.0f, 0.0f, 1.0f);
	xi.v = vec3_create(0.0f, 0.0f, 0.0f);

	float theta = (float)(M_PI / 2.0);
	mat4x4_t T = matrix_exp_se3(&xi, theta);

	/* Translation should be zero for rotation at origin */
	vec3_t t = mat4x4_get_translation(&T);
	zassert_true(is_near_equal(t.x, 0.0f, TEST_TOL));
	zassert_true(is_near_equal(t.y, 0.0f, TEST_TOL));
	zassert_true(is_near_equal(t.z, 0.0f, TEST_TOL));

	/* Rotation should be 90° about Z */
	mat3x3_t R = mat4x4_get_rotation(&T);
	zassert_true(is_near_equal(R.m[0][0], 0.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[0][1], -1.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[1][0], 1.0f, TEST_TOL));
	zassert_true(is_near_equal(R.m[1][1], 0.0f, TEST_TOL));
}

ZTEST(matrix_exp, test_se3_rotation_off_origin)
{
	/* Joint 1: Y-rotation at [0, 0, 0.1]
	 * ω = [0, 1, 0], v = -ω × q = -[0,1,0] × [0,0,0.1] = [0.1, 0, 0]
	 * At θ=π/2: point on Z-axis above origin swings to X-axis
	 */
	vec6_t xi;
	xi.w = vec3_create(0.0f, 1.0f, 0.0f);
	xi.v = vec3_create(0.1f, 0.0f, 0.0f);

	float theta = (float)(M_PI / 2.0);
	mat4x4_t T = matrix_exp_se3(&xi, theta);

	/* Verify it's a valid SE(3) */
	zassert_true(is_near_equal(T.m[3][0], 0.0f, 1e-6f));
	zassert_true(is_near_equal(T.m[3][1], 0.0f, 1e-6f));
	zassert_true(is_near_equal(T.m[3][2], 0.0f, 1e-6f));
	zassert_true(is_near_equal(T.m[3][3], 1.0f, 1e-6f));

	/* Rotation is orthogonal */
	mat3x3_t R = mat4x4_get_rotation(&T);
	mat3x3_t Rt = mat3x3_transpose(&R);
	mat3x3_t product = mat3x3_mul(&Rt, &R);
	mat3x3_t I3 = mat3x3_identity();
	zassert_true(mat3x3_is_equal(&product, &I3, TEST_TOL));
}

ZTEST(matrix_exp, test_se3_bottom_row_preserved)
{
	/* Bottom row of exp([ξ]θ) must always be [0, 0, 0, 1] */
	vec6_t xi;
	xi.w = vec3_create(0.3f, 0.4f, 0.0f);
	/* Normalize ω to unit vector */
	vec3_normalize(&xi.w);
	xi.v = vec3_create(0.1f, -0.2f, 0.3f);

	float theta = 1.234f;
	mat4x4_t T = matrix_exp_se3(&xi, theta);

	zassert_true(is_near_equal(T.m[3][0], 0.0f, 1e-6f));
	zassert_true(is_near_equal(T.m[3][1], 0.0f, 1e-6f));
	zassert_true(is_near_equal(T.m[3][2], 0.0f, 1e-6f));
	zassert_true(is_near_equal(T.m[3][3], 1.0f, 1e-6f));
}

/* ======================================================================== */
/* Test suite registration                                                  */
/* ======================================================================== */

ZTEST_SUITE(matrix_exp, NULL, NULL, NULL, NULL, NULL);
