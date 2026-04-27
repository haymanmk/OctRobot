/*
 * OctroBot Robot Arm Firmware - Forward Kinematics Implementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "forward_kinematics_poe.h"
#include "matrix_exp.h"
#include <string.h>

/* ========================================================================
 * Forward Kinematics - Full Chain
 * ======================================================================== */

bool forward_kinematics_compute(const poe_robot_model_t *model,
                                 const float joint_angles[NUM_JOINTS],
                                 mat4x4_t *T)
{
	/* Validate inputs */
	if (model == NULL || joint_angles == NULL || T == NULL) {
		return false;
	}
	
	/* Start with identity (will accumulate transformations) */
	*T = mat4x4_identity();
	
	/* Chain matrix exponentials: T = exp([ξ₁]θ₁) · exp([ξ₂]θ₂) · ... · exp([ξ₆]θ₆) */
	for (int i = 0; i < NUM_JOINTS; i++) {
		/* Get screw axis for joint i */
		vec6_t xi;
		if (!robot_geometry_get_screw_axis(model, i, &xi)) {
			return false;
		}
		
		/* Compute exp([ξᵢ]θᵢ) */
		mat4x4_t T_i = matrix_exp_se3(&xi, joint_angles[i]);
		
		/* Accumulate: T = T · T_i */
		*T = mat4x4_mul(T, &T_i);
	}
	
	/* Multiply by home configuration: T = T · M */
	mat4x4_t M = robot_geometry_get_home_config(model);
	*T = mat4x4_mul(T, &M);
	
	return true;
}

/* ========================================================================
 * Forward Kinematics - Pose Extraction
 * ======================================================================== */

bool forward_kinematics_compute_pose(const poe_robot_model_t *model,
                                      const float joint_angles[NUM_JOINTS],
                                      vec3_t *position,
                                      mat3x3_t *orientation)
{
	/* Compute full transformation */
	mat4x4_t T;
	if (!forward_kinematics_compute(model, joint_angles, &T)) {
		return false;
	}
	
	/* Extract position and orientation */
	if (position != NULL) {
		*position = mat4x4_get_translation(&T);
	}
	
	if (orientation != NULL) {
		*orientation = mat4x4_get_rotation(&T);
	}
	
	return true;
}

/* ========================================================================
 * Forward Kinematics - Partial Chain
 * ======================================================================== */

bool forward_kinematics_partial(const poe_robot_model_t *model,
                                 const float joint_angles[NUM_JOINTS],
                                 int end_joint_idx,
                                 mat4x4_t *T)
{
	/* Validate inputs */
	if (model == NULL || joint_angles == NULL || T == NULL) {
		return false;
	}
	
	if (end_joint_idx < 0 || end_joint_idx >= NUM_JOINTS) {
		return false;
	}
	
	/* Start with identity */
	*T = mat4x4_identity();
	
	/* Chain matrix exponentials up to end_joint_idx */
	for (int i = 0; i <= end_joint_idx; i++) {
		/* Get screw axis for joint i */
		vec6_t xi;
		if (!robot_geometry_get_screw_axis(model, i, &xi)) {
			return false;
		}
		
		/* Compute exp([ξᵢ]θᵢ) */
		mat4x4_t T_i = matrix_exp_se3(&xi, joint_angles[i]);
		
		/* Accumulate: T = T · T_i */
		*T = mat4x4_mul(T, &T_i);
	}
	
	/* Multiply by home configuration */
	mat4x4_t M = robot_geometry_get_home_config(model);
	*T = mat4x4_mul(T, &M);
	
	return true;
}
