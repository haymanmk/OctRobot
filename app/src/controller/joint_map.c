/*
 * OctroBot Robot Arm Firmware - Joint Calibration Map Implementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "joint_map.h"
#include "kinematics_math.h"   /* deg_to_rad / rad_to_deg */
#include <string.h>

static float s_sign[NUM_JOINTS]       = { -1, 1, 1, -1, 1, -1 };
static float s_offset_deg[NUM_JOINTS] = { 0, 0, 0, 0, 0, 0 };

void joint_map_model_to_servo(const float theta_rad[NUM_JOINTS],
			      float servo_deg[NUM_JOINTS])
{
	for (int i = 0; i < NUM_JOINTS; i++) {
		servo_deg[i] = s_sign[i] * rad_to_deg(theta_rad[i]) +
			       s_offset_deg[i];
	}
}

void joint_map_servo_to_model(const float servo_deg[NUM_JOINTS],
			      float theta_rad[NUM_JOINTS])
{
	for (int i = 0; i < NUM_JOINTS; i++) {
		/* sign[i] is +/-1, so dividing by sign == multiplying. */
		theta_rad[i] = deg_to_rad((servo_deg[i] - s_offset_deg[i]) *
					  s_sign[i]);
	}
}

void joint_map_set_calibration(const float sign[NUM_JOINTS],
			       const float offset_deg[NUM_JOINTS])
{
	memcpy(s_sign, sign, sizeof(s_sign));
	memcpy(s_offset_deg, offset_deg, sizeof(s_offset_deg));
}
