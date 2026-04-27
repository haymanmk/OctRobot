/*
 * OctroBot Robot Arm Firmware - Robot Geometry Implementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "robot_geometry.h"
#include "hal_flash.h"
#include <zephyr/kernel.h>
#include <string.h>
#include <math.h>

/* Flash storage key for POE model */
#define NVS_KEY_POE_MODEL "poe_model"

/* Global robot model instance */
static poe_robot_model_t g_robot_model;
static K_MUTEX_DEFINE(g_robot_model_mutex);

/* CRC32 lookup table (for data validation) */
static const uint32_t crc32_table[256] = {
	0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F,
	0xE963A535, 0x9E6495A3, 0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
	/* ... (full table omitted for brevity, standard CRC32 table) ... */
	0xBE2DA0A5, 0x4C4623A6, 0x5F16D052, 0xAD7D5351
};

/* ========================================================================
 * Helper Functions
 * ======================================================================== */

static uint32_t compute_crc32(const uint8_t *data, size_t len)
{
	uint32_t crc = 0xFFFFFFFF;
	for (size_t i = 0; i < len; i++) {
		crc = crc32_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
	}
	return ~crc;
}

/* ========================================================================
 * Factory Defaults
 * ======================================================================== */

poe_robot_model_t robot_geometry_factory_defaults(void)
{
	poe_robot_model_t model;
	memset(&model, 0, sizeof(model));
	
	/* ============================================================
	 * Factory Default Screw Axes
	 * ============================================================
	 * These are PLACEHOLDER values for a generic 6-DOF arm.
	 * User MUST calibrate and save actual values!
	 * 
	 * Assumed kinematic chain:
	 *   Joint 0: Base rotation (Z-axis, at origin)
	 *   Joint 1: Shoulder pitch (Y-axis, 0.1m up)
	 *   Joint 2: Elbow pitch (Y-axis, 0.2m along X from shoulder)
	 *   Joint 3: Wrist roll (X-axis, 0.15m along X from elbow)
	 *   Joint 4: Wrist pitch (Y-axis, same point as J3)
	 *   Joint 5: Wrist roll (X-axis, 0.1m along X from J4)
	 */
	
	/* Joint 0: Base rotation (Z-axis at origin) */
	model.screw_axes[0].w = vec3_create(0.0f, 0.0f, 1.0f);  /* ω = Z */
	model.screw_axes[0].v = vec3_create(0.0f, 0.0f, 0.0f);  /* v = -ω × q = 0 */
	
	/* Joint 1: Shoulder pitch (Y-axis at [0, 0, 0.1]) */
	model.screw_axes[1].w = vec3_create(0.0f, 1.0f, 0.0f);  /* ω = Y */
	model.screw_axes[1].v = vec3_create(-0.138f, 0.0f, 0.0f);  /* v = -ω × [0,0,0.1] = [0.1,0,0] */
	
	/* Joint 2: Elbow pitch (Y-axis at [0.2, 0, 0.1]) */
	model.screw_axes[2].w = vec3_create(0.0f, 1.0f, 0.0f);  /* ω = Y */
	model.screw_axes[2].v = vec3_create(-0.238f, 0.0f, 0.0f);  /* v = -ω × [0.2,0,0.1] = [0.1,0,0.2] */
	
	/* Joint 3: Wrist roll (X-axis at [0.35, 0, 0.1]) */
	model.screw_axes[3].w = vec3_create(1.0f, 0.0f, 0.0f);  /* ω = X */
	model.screw_axes[3].v = vec3_create(0.0f, 0.243f, 0.001f); /* v = -ω × [0.35,0,0.1] = [0,-0.1,0] */
	
	/* Joint 4: Wrist pitch (Y-axis at [0.35, 0, 0.1]) */
	model.screw_axes[4].w = vec3_create(0.0f, 1.0f, 0.0f);  /* ω = Y */
	model.screw_axes[4].v = vec3_create(-0.243f, 0.0f, 0.108f); /* v = -ω × [0.35,0,0.1] = [0.1,0,0.35] */
	
	/* Joint 5: Wrist roll (X-axis at [0.45, 0, 0.1]) */
	model.screw_axes[5].w = vec3_create(1.0f, 0.0f, 0.0f);  /* ω = X */
	model.screw_axes[5].v = vec3_create(0.0f, 0.243f, 0.0f); /* v = -ω × [0.45,0,0.1] = [0,-0.1,0] */
	
	/* ============================================================
	 * Home Configuration M
	 * ============================================================
	 * End-effector pose at θ=[0,0,0,0,0,0]
	 * Assume: arm straight along +X, end-effector 0.5m from base
	 */
	model.M = mat4x4_identity();
	model.M.m[0][3] = 0.5f;  /* X = 0.5m */
	model.M.m[1][3] = 0.0f;  /* Y = 0m */
	model.M.m[2][3] = 0.1f;  /* Z = 0.1m (height of base) */
	
	/* ============================================================
	 * Joint Limits (Conservative Defaults)
	 * ============================================================
	 * User should measure actual mechanical limits!
	 */
	for (int i = 0; i < NUM_JOINTS; i++) {
		model.joint_limits_min[i] = deg_to_rad(-150.0f);  /* -150° */
		model.joint_limits_max[i] = deg_to_rad(150.0f);   /* +150° */
	}
	
	/* Compute CRC */
	model.crc32 = compute_crc32((uint8_t*)&model, 
	                             sizeof(model) - sizeof(uint32_t));
	
	return model;
}

/* ========================================================================
 * Flash Storage
 * ======================================================================== */

bool robot_geometry_load_from_flash(poe_robot_model_t *model)
{
	/* Attempt to read from flash */
	int ret = hal_flash_read(NVS_KEY_POE_MODEL, 
	                          (uint8_t*)model, 
	                          sizeof(poe_robot_model_t));
	
	if (ret != 0) {
		/* Failed to read, use factory defaults */
		*model = robot_geometry_factory_defaults();
		return false;
	}
	
	/* Validate CRC */
	uint32_t computed_crc = compute_crc32((uint8_t*)model, 
	                                       sizeof(*model) - sizeof(uint32_t));
	if (computed_crc != model->crc32) {
		/* CRC mismatch, use factory defaults */
		*model = robot_geometry_factory_defaults();
		return false;
	}
	
	/* Validate parameters */
	if (!robot_geometry_validate(model)) {
		*model = robot_geometry_factory_defaults();
		return false;
	}
	
	return true;
}

bool robot_geometry_save_to_flash(const poe_robot_model_t *model)
{
	/* Create mutable copy to update CRC */
	poe_robot_model_t model_copy = *model;
	
	/* Compute and store CRC */
	model_copy.crc32 = compute_crc32((uint8_t*)&model_copy, 
	                                  sizeof(model_copy) - sizeof(uint32_t));
	
	/* Write to flash */
	int ret = hal_flash_write(NVS_KEY_POE_MODEL, 
	                           (const uint8_t*)&model_copy, 
	                           sizeof(poe_robot_model_t));
	
	return (ret == 0);
}

/* ========================================================================
 * Validation
 * ======================================================================== */

bool robot_geometry_validate(const poe_robot_model_t *model)
{
	/* Check screw axes normalization (|ω| ≈ 1 for revolute joints) */
	for (int i = 0; i < NUM_JOINTS; i++) {
		float w_norm = vec3_norm(&model->screw_axes[i].w);
		if (!is_near_equal(w_norm, 1.0f, 0.01f)) {
			return false;  /* Angular velocity not unit vector */
		}
	}
	
	/* Check joint limits validity */
	for (int i = 0; i < NUM_JOINTS; i++) {
		if (model->joint_limits_min[i] >= model->joint_limits_max[i]) {
			return false;  /* Invalid limits */
		}
		if (fabsf(model->joint_limits_min[i]) > 2.0f * M_PI ||
		    fabsf(model->joint_limits_max[i]) > 2.0f * M_PI) {
			return false;  /* Limits out of reasonable range */
		}
	}
	
	/* Check home configuration M is valid SE(3)
	 * Simplified: just check bottom row is [0 0 0 1] */
	if (!is_near_zero(model->M.m[3][0], 0.001f) ||
	    !is_near_zero(model->M.m[3][1], 0.001f) ||
	    !is_near_zero(model->M.m[3][2], 0.001f) ||
	    !is_near_equal(model->M.m[3][3], 1.0f, 0.001f)) {
		return false;  /* Invalid homogeneous transform */
	}
	
	return true;
}

/* ========================================================================
 * Parameter Access
 * ======================================================================== */

bool robot_geometry_set_screw_axis(poe_robot_model_t *model, 
                                    int joint_idx, 
                                    const vec6_t *xi)
{
	if (joint_idx < 0 || joint_idx >= NUM_JOINTS) {
		return false;
	}
	
	model->screw_axes[joint_idx] = *xi;
	return true;
}

bool robot_geometry_get_screw_axis(const poe_robot_model_t *model, 
                                    int joint_idx, 
                                    vec6_t *xi)
{
	if (joint_idx < 0 || joint_idx >= NUM_JOINTS) {
		return false;
	}
	
	*xi = model->screw_axes[joint_idx];
	return true;
}

void robot_geometry_set_home_config(poe_robot_model_t *model, 
                                     const mat4x4_t *M)
{
	model->M = *M;
}

mat4x4_t robot_geometry_get_home_config(const poe_robot_model_t *model)
{
	return model->M;
}

bool robot_geometry_set_joint_limits(poe_robot_model_t *model,
                                      int joint_idx,
                                      float min_rad, float max_rad)
{
	if (joint_idx < 0 || joint_idx >= NUM_JOINTS) {
		return false;
	}
	
	if (min_rad >= max_rad) {
		return false;  /* Invalid range */
	}
	
	if (fabsf(min_rad) > 2.0f * M_PI || fabsf(max_rad) > 2.0f * M_PI) {
		return false;  /* Out of reasonable range */
	}
	
	model->joint_limits_min[joint_idx] = min_rad;
	model->joint_limits_max[joint_idx] = max_rad;
	return true;
}

bool robot_geometry_get_joint_limits(const poe_robot_model_t *model,
                                      int joint_idx,
                                      float *min_rad, float *max_rad)
{
	if (joint_idx < 0 || joint_idx >= NUM_JOINTS) {
		return false;
	}
	
	*min_rad = model->joint_limits_min[joint_idx];
	*max_rad = model->joint_limits_max[joint_idx];
	return true;
}

bool robot_geometry_check_joint_limits(const poe_robot_model_t *model,
                                        const float joint_angles[NUM_JOINTS])
{
	for (int i = 0; i < NUM_JOINTS; i++) {
		if (joint_angles[i] < model->joint_limits_min[i] ||
		    joint_angles[i] > model->joint_limits_max[i]) {
			return false;
		}
	}
	return true;
}

void robot_geometry_clamp_joint_angles(const poe_robot_model_t *model,
                                        float joint_angles[NUM_JOINTS])
{
	for (int i = 0; i < NUM_JOINTS; i++) {
		joint_angles[i] = clamp_float(joint_angles[i], 
		                               model->joint_limits_min[i],
		                               model->joint_limits_max[i]);
	}
}

/* ========================================================================
 * Global Model Instance
 * ======================================================================== */

const poe_robot_model_t* robot_geometry_get_model(void)
{
	return &g_robot_model;
}

void robot_geometry_set_model(const poe_robot_model_t *model)
{
	k_mutex_lock(&g_robot_model_mutex, K_FOREVER);
	g_robot_model = *model;
	k_mutex_unlock(&g_robot_model_mutex);
}

/* ========================================================================
 * Initialization (Called Once at Startup)
 * ======================================================================== */

void robot_geometry_init(void)
{
	/* Try to load from flash, fall back to factory defaults */
	if (!robot_geometry_load_from_flash(&g_robot_model)) {
		/* No calibration found, use factory defaults */
		g_robot_model = robot_geometry_factory_defaults();
	}
}
