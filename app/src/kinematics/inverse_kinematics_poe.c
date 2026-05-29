/*
 * OctroBot Robot Arm Firmware - Inverse Kinematics Implementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "inverse_kinematics_poe.h"
#include "forward_kinematics_poe.h"
#include "matrix_exp.h"
#include <string.h>

ik_params_t inverse_kinematics_default_params(void)
{
	ik_params_t p = {
		.eomg = 1e-3f,
		.ev = 1e-4f,
		.max_iters = 20,
		.lambda = 0.0f,
	};
	return p;
}

ik_status_t inverse_kinematics_compute(const poe_robot_model_t *model,
				       const mat4x4_t *T_target,
				       const float seed[NUM_JOINTS],
				       float out_angles[NUM_JOINTS],
				       const ik_params_t *params,
				       int *iters_used)
{
	if (model == NULL || T_target == NULL || seed == NULL ||
	    out_angles == NULL) {
		return IK_INVALID_INPUT;
	}

	(void)params;
	if (iters_used != NULL) {
		*iters_used = 0;
	}
	memcpy(out_angles, seed, sizeof(float) * NUM_JOINTS);
	return IK_NO_CONVERGENCE; /* replaced in a later task */
}
