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
