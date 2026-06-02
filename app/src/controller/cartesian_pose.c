/*
 * OctroBot Robot Arm Firmware - Cartesian Pose -> Joints (pure core)
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "cartesian_pose.h"
#include "kinematics_math.h"
#include "inverse_kinematics_poe.h"
#include <stddef.h>   /* NULL */

cmove_status_t cartesian_pose_to_joints(const poe_robot_model_t *model,
					float x, float y, float z,
					float roll_deg, float pitch_deg,
					float yaw_deg,
					const float seed_theta[NUM_JOINTS],
					float out_theta[NUM_JOINTS],
					int *iters_used)
{
	if (iters_used != NULL) {
		*iters_used = 0;
	}
	if (model == NULL || seed_theta == NULL || out_theta == NULL) {
		return CMOVE_BAD_ARGS;
	}

	/* Build the SE(3) target T_sd from RPY + position. */
	mat3x3_t R = mat3x3_from_rpy(deg_to_rad(roll_deg),
				     deg_to_rad(pitch_deg),
				     deg_to_rad(yaw_deg));
	vec3_t p = vec3_create(x, y, z);
	mat4x4_t T_target = mat4x4_from_rt(&R, &p);

	float out[NUM_JOINTS];
	ik_status_t ik = inverse_kinematics_compute(model, &T_target,
						    seed_theta, out, NULL,
						    iters_used);

	switch (ik) {
	case IK_SUCCESS:
		for (int i = 0; i < NUM_JOINTS; i++) {
			out_theta[i] = out[i];
		}
		return CMOVE_OK;
	/* Solver still writes the final iterate; surface it but flag the limit. */
	case IK_OUT_OF_LIMITS:
		for (int i = 0; i < NUM_JOINTS; i++) {
			out_theta[i] = out[i];
		}
		return CMOVE_OUT_OF_LIMITS;
	case IK_NO_CONVERGENCE:
		return CMOVE_NO_SOLUTION;
	/* Unreachable in practice (NULLs are caught above); mapped defensively. */
	case IK_INVALID_INPUT:
	default:
		return CMOVE_BAD_ARGS;
	}
}
