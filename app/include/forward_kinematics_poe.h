/*
 * OctroBot Robot Arm Firmware - Forward Kinematics (POE)
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Forward kinematics using Product of Exponentials (POE) formula:
 *   FK(θ) = exp([ξ₁]θ₁) · exp([ξ₂]θ₂) · ... · exp([ξ₆]θ₆) · M
 * 
 * Computational complexity: O(n) matrix multiplications (6× 4x4 matrices)
 * Expected runtime: 2-5ms on ESP32 @ 240MHz with FPU
 */

#ifndef FORWARD_KINEMATICS_POE_H
#define FORWARD_KINEMATICS_POE_H

#include "kinematics_math.h"
#include "robot_geometry.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Forward Kinematics API
 * ======================================================================== */

/**
 * Compute forward kinematics using POE formula
 * 
 * Computes end-effector pose T ∈ SE(3) from joint angles θ:
 *   T(θ) = exp([ξ₁]θ₁) · exp([ξ₂]θ₂) · ... · exp([ξ₆]θ₆) · M
 * 
 * Inputs:
 *   - model: Robot geometric parameters (screw axes, home config)
 *   - joint_angles: Array of 6 joint angles (radians)
 * 
 * Output:
 *   - T: 4x4 homogeneous transformation matrix (end-effector pose)
 * 
 * Returns:
 *   - true: Computation successful
 *   - false: Invalid inputs
 * 
 * Performance: ~2-5ms on ESP32 @ 240MHz
 */
bool forward_kinematics_compute(const poe_robot_model_t *model,
                                 const float joint_angles[NUM_JOINTS],
                                 mat4x4_t *T);

/**
 * Compute forward kinematics with position and orientation extraction
 * 
 * Convenience function that computes FK and extracts:
 *   - Position: (x, y, z) in meters
 *   - Orientation: 3x3 rotation matrix
 * 
 * Inputs:
 *   - model: Robot geometric parameters
 *   - joint_angles: Array of 6 joint angles (radians)
 * 
 * Outputs:
 *   - position: End-effector position (vec3)
 *   - orientation: End-effector orientation (mat3x3)
 * 
 * Returns:
 *   - true: Computation successful
 *   - false: Invalid inputs
 */
bool forward_kinematics_compute_pose(const poe_robot_model_t *model,
                                      const float joint_angles[NUM_JOINTS],
                                      vec3_t *position,
                                      mat3x3_t *orientation);

/**
 * Compute forward kinematics for partial chain
 * 
 * Computes transformation from base to joint i:
 *   T_i(θ) = exp([ξ₁]θ₁) · ... · exp([ξᵢ]θᵢ) · M
 * 
 * Useful for:
 *   - Collision checking
 *   - Jacobian computation
 *   - Visualization of intermediate frames
 * 
 * Inputs:
 *   - model: Robot geometric parameters
 *   - joint_angles: Array of joint angles (only first end_joint_idx used)
 *   - end_joint_idx: Index of last joint to include (0-5)
 * 
 * Output:
 *   - T: Transformation to joint i
 * 
 * Returns:
 *   - true: Computation successful
 *   - false: Invalid inputs
 */
bool forward_kinematics_partial(const poe_robot_model_t *model,
                                 const float joint_angles[NUM_JOINTS],
                                 int end_joint_idx,
                                 mat4x4_t *T);

#ifdef __cplusplus
}
#endif

#endif /* FORWARD_KINEMATICS_POE_H */
