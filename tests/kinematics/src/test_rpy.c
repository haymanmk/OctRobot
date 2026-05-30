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
