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
#include "kinematics_math.h"   /* rad_to_deg */
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cartesian_move, LOG_LEVEL_INF);

#define LOG_JOINTS_DEG(tag, a)                                              \
	LOG_INF("%s: %.1f %.1f %.1f %.1f %.1f %.1f", tag, (double)(a)[0],   \
		(double)(a)[1], (double)(a)[2], (double)(a)[3],            \
		(double)(a)[4], (double)(a)[5])

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

	/*
	 * 2. Candidate IK seeds, tried in order. The current servo config is best
	 * when the target is near the current pose; the fixed fallbacks rescue far
	 * targets and near-singular goals (e.g. the all-zero "home" config) whose
	 * basin the current seed may miss. IK is a local solver, so seed restarts
	 * are how reachable-but-missed poses get found.
	 */
	float seeds[][NUM_JOINTS] = {
		{ 0, 0, 0, 0, 0, 0 },                          /* [0] current (filled) */
		{ 0, 0, 0, 0, 0, 0 },                          /* [1] home / zero       */
		{ 0.0f, 0.6f, -1.2f, 0.0f, 0.6f, 0.0f },       /* [2] elbow-bent        */
		{ 0.0f, -0.6f, 1.2f, 0.0f, -0.6f, 0.0f },      /* [3] elbow-bent (alt)  */
	};
	const int n_seeds = (int)(sizeof(seeds) / sizeof(seeds[0]));

	uint16_t positions[NUM_JOINTS];
	if (feetech_servo_read_multi_positions(k_ids, positions, NUM_JOINTS) ==
	    0) {
		float servo_deg[NUM_JOINTS];
		for (int i = 0; i < NUM_JOINTS; i++) {
			servo_deg[i] = FEETECH_POS_TO_DEG(positions[i]);
		}
		joint_map_servo_to_model(servo_deg, seeds[0]);
	} else {
		LOG_WRN("movec: servo read failed, using zero seed");
	}

	/* 3. Solve IK, retrying from each seed until one converges. */
	float out_theta[NUM_JOINTS];
	int ik_iters = 0;
	cmove_status_t st = CMOVE_NO_SOLUTION;
	int used_seed = -1;
	for (int s = 0; s < n_seeds; s++) {
		st = cartesian_pose_to_joints(model, x, y, z, roll_deg, pitch_deg,
					      yaw_deg, seeds[s], out_theta,
					      &ik_iters);
		used_seed = s;
		/* OK and OUT_OF_LIMITS are definitive; only retry on no-solution. */
		if (st == CMOVE_OK || st == CMOVE_OUT_OF_LIMITS) {
			break;
		}
	}

	LOG_INF("IK result: status=%d iters=%d seed=%d", st, ik_iters, used_seed);
	if (st != CMOVE_OK) {
		return st;
	}

	float out_deg[NUM_JOINTS];
	for (int i = 0; i < NUM_JOINTS; i++) {
		out_deg[i] = rad_to_deg(out_theta[i]);
	}
	LOG_JOINTS_DEG("IK angles (deg)", out_deg);

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

	LOG_JOINTS_DEG("servo goal(deg)", servo_goal);

	/* 5. Sync-write goal angles with the requested move time (single atomic
	 * packet; all joints interpolate over move_time_ms and finish together).
	 * NOTE: the plain feetech_servo_sync_write_angles() hardcodes the servo
	 * time field to 0 (immediate) and cannot honor move_time_ms, so the
	 * _timed variant is required. */
	if (feetech_servo_sync_write_angles_timed(k_ids, servo_goal, NUM_JOINTS,
						  move_time_ms) != 0) {
		LOG_WRN("movec: sync write failed");
		return CMOVE_SERVO_ERR;
	}

	return CMOVE_OK;
}
