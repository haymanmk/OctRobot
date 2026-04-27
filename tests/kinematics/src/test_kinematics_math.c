/*
 * OctroBot - Kinematics Math Unit Tests
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tests: vec3, mat3x3, mat4x4 operations, utility functions.
 */

#include <zephyr/ztest.h>
#include "kinematics_math.h"
#include <math.h>

/* ======================================================================== */
/* Vec3 tests                                                               */
/* ======================================================================== */

ZTEST(kinematics_math, test_vec3_cross_product)
{
	/* i × j = k */
	vec3_t a = vec3_create(1.0f, 0.0f, 0.0f);
	vec3_t b = vec3_create(0.0f, 1.0f, 0.0f);
	vec3_t c = vec3_cross(&a, &b);

	zassert_true(is_near_equal(c.x, 0.0f, 1e-6f));
	zassert_true(is_near_equal(c.y, 0.0f, 1e-6f));
	zassert_true(is_near_equal(c.z, 1.0f, 1e-6f));
}

ZTEST(kinematics_math, test_vec3_cross_anticommutative)
{
	/* a × b = -(b × a) */
	vec3_t a = vec3_create(1.0f, 2.0f, 3.0f);
	vec3_t b = vec3_create(4.0f, 5.0f, 6.0f);
	vec3_t ab = vec3_cross(&a, &b);
	vec3_t ba = vec3_cross(&b, &a);

	zassert_true(is_near_equal(ab.x, -ba.x, 1e-6f));
	zassert_true(is_near_equal(ab.y, -ba.y, 1e-6f));
	zassert_true(is_near_equal(ab.z, -ba.z, 1e-6f));
}

ZTEST(kinematics_math, test_vec3_dot_product)
{
	vec3_t a = vec3_create(1.0f, 2.0f, 3.0f);
	vec3_t b = vec3_create(4.0f, 5.0f, 6.0f);
	float d = vec3_dot(&a, &b);

	/* 1*4 + 2*5 + 3*6 = 32 */
	zassert_true(is_near_equal(d, 32.0f, 1e-6f));
}

ZTEST(kinematics_math, test_vec3_normalize)
{
	vec3_t v = vec3_create(3.0f, 4.0f, 0.0f);
	bool ok = vec3_normalize(&v);

	zassert_true(ok);
	zassert_true(is_near_equal(vec3_norm(&v), 1.0f, 1e-6f));
	zassert_true(is_near_equal(v.x, 0.6f, 1e-6f));
	zassert_true(is_near_equal(v.y, 0.8f, 1e-6f));
}

ZTEST(kinematics_math, test_vec3_norm)
{
	vec3_t v = vec3_create(3.0f, 4.0f, 0.0f);
	zassert_true(is_near_equal(vec3_norm(&v), 5.0f, 1e-6f));
}

/* ======================================================================== */
/* Mat3x3 tests                                                             */
/* ======================================================================== */

ZTEST(kinematics_math, test_mat3x3_identity_multiply)
{
	/* I × A = A */
	mat3x3_t I = mat3x3_identity();
	mat3x3_t A = mat3x3_identity();
	A.m[0][1] = 2.0f;
	A.m[1][2] = 3.0f;

	mat3x3_t result = mat3x3_mul(&I, &A);
	zassert_true(mat3x3_is_equal(&result, &A, 1e-6f));
}

ZTEST(kinematics_math, test_mat3x3_transpose)
{
	mat3x3_t A = mat3x3_zero();
	A.m[0][1] = 5.0f;
	A.m[1][0] = 7.0f;

	mat3x3_t At = mat3x3_transpose(&A);
	zassert_true(is_near_equal(At.m[0][1], 7.0f, 1e-6f));
	zassert_true(is_near_equal(At.m[1][0], 5.0f, 1e-6f));
}

ZTEST(kinematics_math, test_skew_symmetric_antisymmetric)
{
	/* [ω]ᵀ = -[ω] */
	vec3_t w = vec3_create(1.0f, 2.0f, 3.0f);
	mat3x3_t S = mat3x3_skew_symmetric(&w);
	mat3x3_t St = mat3x3_transpose(&S);
	mat3x3_t neg_S = mat3x3_scale(&S, -1.0f);

	zassert_true(mat3x3_is_equal(&St, &neg_S, 1e-6f));
}

ZTEST(kinematics_math, test_skew_symmetric_values)
{
	/* [ω] for ω = (1,2,3) should be:
	 *  [  0  -3   2 ]
	 *  [  3   0  -1 ]
	 *  [ -2   1   0 ]
	 */
	vec3_t w = vec3_create(1.0f, 2.0f, 3.0f);
	mat3x3_t S = mat3x3_skew_symmetric(&w);

	zassert_true(is_near_equal(S.m[0][0], 0.0f, 1e-6f));
	zassert_true(is_near_equal(S.m[0][1], -3.0f, 1e-6f));
	zassert_true(is_near_equal(S.m[0][2], 2.0f, 1e-6f));
	zassert_true(is_near_equal(S.m[1][0], 3.0f, 1e-6f));
	zassert_true(is_near_equal(S.m[1][1], 0.0f, 1e-6f));
	zassert_true(is_near_equal(S.m[1][2], -1.0f, 1e-6f));
	zassert_true(is_near_equal(S.m[2][0], -2.0f, 1e-6f));
	zassert_true(is_near_equal(S.m[2][1], 1.0f, 1e-6f));
	zassert_true(is_near_equal(S.m[2][2], 0.0f, 1e-6f));
}

/* ======================================================================== */
/* Mat4x4 tests                                                             */
/* ======================================================================== */

ZTEST(kinematics_math, test_mat4x4_identity)
{
	mat4x4_t I = mat4x4_identity();

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			float expected = (r == c) ? 1.0f : 0.0f;
			zassert_true(is_near_equal(I.m[r][c], expected, 1e-6f));
		}
	}
}

ZTEST(kinematics_math, test_mat4x4_inverse_transform)
{
	/* T × T⁻¹ = I for a valid SE(3) transform */
	/* Build a simple rotation about Z by 30° + translation */
	float angle = deg_to_rad(30.0f);
	mat3x3_t R = mat3x3_identity();
	R.m[0][0] = cosf(angle);
	R.m[0][1] = -sinf(angle);
	R.m[1][0] = sinf(angle);
	R.m[1][1] = cosf(angle);

	vec3_t t = vec3_create(1.0f, 2.0f, 3.0f);
	mat4x4_t T = mat4x4_from_rt(&R, &t);
	mat4x4_t T_inv = mat4x4_inverse_transform(&T);
	mat4x4_t product = mat4x4_mul(&T, &T_inv);

	mat4x4_t I = mat4x4_identity();
	zassert_true(mat4x4_is_equal(&product, &I, 1e-4f));
}

ZTEST(kinematics_math, test_mat4x4_from_rt_extraction)
{
	/* Build T from R and t, then extract them back */
	mat3x3_t R = mat3x3_identity();
	vec3_t t = vec3_create(1.0f, 2.0f, 3.0f);

	mat4x4_t T = mat4x4_from_rt(&R, &t);
	mat3x3_t R_out = mat4x4_get_rotation(&T);
	vec3_t t_out = mat4x4_get_translation(&T);

	zassert_true(mat3x3_is_equal(&R_out, &R, 1e-6f));
	zassert_true(is_near_equal(t_out.x, 1.0f, 1e-6f));
	zassert_true(is_near_equal(t_out.y, 2.0f, 1e-6f));
	zassert_true(is_near_equal(t_out.z, 3.0f, 1e-6f));
}

/* ======================================================================== */
/* Utility tests                                                            */
/* ======================================================================== */

ZTEST(kinematics_math, test_deg_rad_roundtrip)
{
	float deg = 45.0f;
	float rad = deg_to_rad(deg);
	float deg_back = rad_to_deg(rad);

	zassert_true(is_near_equal(deg_back, deg, 1e-4f));
}

ZTEST(kinematics_math, test_deg_to_rad_known_values)
{
	zassert_true(is_near_equal(deg_to_rad(0.0f), 0.0f, 1e-6f));
	zassert_true(is_near_equal(deg_to_rad(90.0f), (float)(M_PI / 2.0), 1e-4f));
	zassert_true(is_near_equal(deg_to_rad(180.0f), (float)M_PI, 1e-4f));
	zassert_true(is_near_equal(deg_to_rad(360.0f), (float)(2.0 * M_PI), 1e-4f));
}

ZTEST(kinematics_math, test_clamp_float)
{
	zassert_true(is_near_equal(clamp_float(5.0f, 0.0f, 10.0f), 5.0f, 1e-6f));
	zassert_true(is_near_equal(clamp_float(-1.0f, 0.0f, 10.0f), 0.0f, 1e-6f));
	zassert_true(is_near_equal(clamp_float(15.0f, 0.0f, 10.0f), 10.0f, 1e-6f));
}

/* ======================================================================== */
/* Test suite registration                                                  */
/* ======================================================================== */

ZTEST_SUITE(kinematics_math, NULL, NULL, NULL, NULL, NULL);
