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
