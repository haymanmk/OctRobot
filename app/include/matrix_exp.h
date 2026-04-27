/*
 * OctroBot Robot Arm Firmware - Matrix Exponential (POE Kinematics)
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Matrix exponential for screw theory (Product of Exponentials).
 * Implements Rodrigues' formula for efficient computation of exp([ξ]θ).
 */

#ifndef MATRIX_EXP_H
#define MATRIX_EXP_H

#include "kinematics_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Matrix Exponential Functions
 * ======================================================================== */

/**
 * Compute matrix exponential for rotation: exp([ω]θ) ∈ SO(3)
 * 
 * Uses Rodrigues' formula:
 *   exp([ω]θ) = I + sin(θ)[ω̂] + (1-cos(θ))[ω̂]²
 * 
 * where ω̂ is the unit angular velocity vector and θ is the angle.
 * 
 * Inputs:
 *   - w: Angular velocity vector (ω)
 *   - theta: Rotation angle (radians)
 * 
 * Output:
 *   - Rotation matrix R ∈ SO(3)
 * 
 * Special cases:
 *   - If |ω| ≈ 0: returns identity matrix
 *   - If θ ≈ 0: returns identity matrix
 */
mat3x3_t matrix_exp_so3(const vec3_t *w, float theta);

/**
 * Compute matrix exponential for rigid motion: exp([ξ]θ) ∈ SE(3)
 * 
 * Uses exponential formula for screw motion:
 *   exp([ξ]θ) = [ exp([ω]θ)   G(θ)v ]
 *               [     0          1   ]
 * 
 * where:
 *   - ξ = (ω, v) is the screw axis (angular + linear velocity)
 *   - exp([ω]θ) is computed via Rodrigues' formula
 *   - G(θ) = I*θ + (1-cos(θ))[ω̂] + (θ-sin(θ))[ω̂]²
 * 
 * Inputs:
 *   - xi: Screw axis (ω, v)
 *   - theta: Joint angle (radians)
 * 
 * Output:
 *   - Homogeneous transformation matrix T ∈ SE(3)
 * 
 * Special cases:
 *   - If |ω| ≈ 0 (pure translation): T = [I, v*θ; 0, 1]
 *   - If θ ≈ 0: returns identity matrix
 */
mat4x4_t matrix_exp_se3(const vec6_t *xi, float theta);

/**
 * Compute G(θ) matrix for exponential formula:
 *   G(θ) = I*θ + (1-cos(θ))[ω̂] + (θ-sin(θ))[ω̂]²
 * 
 * Used internally by matrix_exp_se3 to compute translation part.
 * 
 * This function is exposed for testing and debugging purposes.
 */
mat3x3_t compute_g_matrix(const vec3_t *w_unit, float theta);

#ifdef __cplusplus
}
#endif

#endif /* MATRIX_EXP_H */
