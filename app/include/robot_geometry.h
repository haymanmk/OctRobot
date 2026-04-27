/*
 * OctroBot Robot Arm Firmware - Robot Geometry (POE Model)
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Robot geometric parameters using Product of Exponentials (POE) formulation.
 * Stores: screw axes (ξ₁...ξ₆), home configuration (M), joint limits.
 * Supports: factory defaults, user calibration, flash persistence.
 */

#ifndef ROBOT_GEOMETRY_H
#define ROBOT_GEOMETRY_H

#include "kinematics_math.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Number of joints in the robot arm */
#define NUM_JOINTS 6

/* ========================================================================
 * Data Structures
 * ======================================================================== */

/**
 * POE Robot Model
 * 
 * Contains all geometric parameters for POE kinematics:
 *   - screw_axes[6]: Screw axes ξ₁...ξ₆ (each is ω + v)
 *   - M: Home configuration (end-effector pose at θ=[0,0,0,0,0,0])
 *   - joint_limits_min[6]: Minimum joint angles (radians)
 *   - joint_limits_max[6]: Maximum joint angles (radians)
 * 
 * Total size: 6*24 + 64 + 24 + 24 = 256 bytes
 */
typedef struct {
	vec6_t screw_axes[NUM_JOINTS];    /* Screw axes ξ₁...ξ₆ (144 bytes) */
	mat4x4_t M;                        /* Home configuration (64 bytes) */
	float joint_limits_min[NUM_JOINTS]; /* Min angles (24 bytes) */
	float joint_limits_max[NUM_JOINTS]; /* Max angles (24 bytes) */
	uint32_t crc32;                    /* CRC for validation (4 bytes) */
} poe_robot_model_t;

/* ========================================================================
 * Initialization and Configuration
 * ======================================================================== */

/**
 * Initialize robot geometry with factory defaults
 * 
 * This function sets up placeholder values for a typical 6-DOF arm.
 * User should calibrate and save custom parameters.
 * 
 * Factory defaults assume:
 *   - Vertical base rotation (Z-axis)
 *   - Shoulder/elbow pitch joints (Y-axis)
 *   - Wrist roll/pitch/roll (Z-Y-Z Euler)
 *   - Home pose: arm straight up, end-effector 0.5m above base
 * 
 * Returns:
 *   - Initialized POE robot model with factory defaults
 */
poe_robot_model_t robot_geometry_factory_defaults(void);

/**
 * Load robot geometry from flash (NVS)
 * 
 * Attempts to load user-calibrated parameters from non-volatile storage.
 * If no calibration exists or CRC fails, returns factory defaults.
 * 
 * Returns:
 *   - true: Successfully loaded from flash
 *   - false: Flash load failed, using factory defaults
 */
bool robot_geometry_load_from_flash(poe_robot_model_t *model);

/**
 * Save robot geometry to flash (NVS)
 * 
 * Persists calibrated parameters to non-volatile storage.
 * Computes and stores CRC32 for data integrity.
 * 
 * Returns:
 *   - true: Successfully saved to flash
 *   - false: Flash write failed
 */
bool robot_geometry_save_to_flash(const poe_robot_model_t *model);

/**
 * Validate robot geometry parameters
 * 
 * Checks:
 *   - Screw axes are properly normalized (|ω| = 1 for revolute joints)
 *   - Joint limits are valid (min < max, within [-2π, 2π])
 *   - Home configuration M is valid SE(3) matrix
 * 
 * Returns:
 *   - true: Model is valid
 *   - false: Model has errors
 */
bool robot_geometry_validate(const poe_robot_model_t *model);

/* ========================================================================
 * Parameter Access and Modification
 * ======================================================================== */

/**
 * Set screw axis for a joint
 * 
 * Updates the screw axis (ω, v) for joint i.
 * For revolute joints, ω should be unit vector (|ω| = 1).
 * For prismatic joints (not used in this robot), ω = 0, |v| = 1.
 * 
 * Inputs:
 *   - model: Robot model to modify
 *   - joint_idx: Joint index (0-5)
 *   - xi: Screw axis (ω, v)
 * 
 * Returns:
 *   - true: Successfully set
 *   - false: Invalid joint index
 */
bool robot_geometry_set_screw_axis(poe_robot_model_t *model, 
                                    int joint_idx, 
                                    const vec6_t *xi);

/**
 * Get screw axis for a joint
 * 
 * Retrieves the screw axis (ω, v) for joint i.
 * 
 * Inputs:
 *   - model: Robot model to query
 *   - joint_idx: Joint index (0-5)
 *   - xi: Output screw axis
 * 
 * Returns:
 *   - true: Successfully retrieved
 *   - false: Invalid joint index
 */
bool robot_geometry_get_screw_axis(const poe_robot_model_t *model, 
                                    int joint_idx, 
                                    vec6_t *xi);

/**
 * Set home configuration M
 * 
 * Sets the end-effector pose at home position θ=[0,0,0,0,0,0].
 * M should be a valid SE(3) homogeneous transformation matrix.
 */
void robot_geometry_set_home_config(poe_robot_model_t *model, 
                                     const mat4x4_t *M);

/**
 * Get home configuration M
 */
mat4x4_t robot_geometry_get_home_config(const poe_robot_model_t *model);

/**
 * Set joint limits
 * 
 * Sets the mechanical joint limits (min, max) for joint i.
 * 
 * Inputs:
 *   - model: Robot model to modify
 *   - joint_idx: Joint index (0-5)
 *   - min_rad: Minimum angle (radians)
 *   - max_rad: Maximum angle (radians)
 * 
 * Returns:
 *   - true: Successfully set
 *   - false: Invalid parameters (min >= max, out of range)
 */
bool robot_geometry_set_joint_limits(poe_robot_model_t *model,
                                      int joint_idx,
                                      float min_rad, float max_rad);

/**
 * Get joint limits
 */
bool robot_geometry_get_joint_limits(const poe_robot_model_t *model,
                                      int joint_idx,
                                      float *min_rad, float *max_rad);

/**
 * Check if joint angles are within limits
 * 
 * Validates all 6 joint angles against stored limits.
 * 
 * Inputs:
 *   - model: Robot model with joint limits
 *   - joint_angles: Array of 6 joint angles (radians)
 * 
 * Returns:
 *   - true: All angles within limits
 *   - false: One or more angles out of bounds
 */
bool robot_geometry_check_joint_limits(const poe_robot_model_t *model,
                                        const float joint_angles[NUM_JOINTS]);

/**
 * Clamp joint angles to limits
 * 
 * Enforces joint limits by clamping out-of-bounds angles.
 * Modifies joint_angles array in-place.
 */
void robot_geometry_clamp_joint_angles(const poe_robot_model_t *model,
                                        float joint_angles[NUM_JOINTS]);

/* ========================================================================
 * Global Robot Model Instance
 * ======================================================================== */

/**
 * Get pointer to global robot model
 * 
 * Returns pointer to singleton robot model instance.
 * This instance is initialized once at startup and used by FK/IK.
 * 
 * Thread-safe: Multiple threads can read, but only one should write.
 */
const poe_robot_model_t* robot_geometry_get_model(void);

/**
 * Update global robot model
 * 
 * Updates the singleton robot model instance.
 * Should only be called during calibration or initialization.
 * 
 * Thread-safe: Uses mutex to protect write access.
 */
void robot_geometry_set_model(const poe_robot_model_t *model);

/**
 * Initialize robot geometry subsystem
 * 
 * Loads robot model from flash (with CRC validation).
 * Falls back to factory defaults if flash read fails.
 * 
 * Must be called once at startup before using FK/IK.
 */
void robot_geometry_init(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_GEOMETRY_H */
