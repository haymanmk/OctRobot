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
