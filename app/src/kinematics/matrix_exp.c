/*
 * OctroBot Robot Arm Firmware - Matrix Exponential Implementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "matrix_exp.h"
#include <math.h>

#define EPSILON 1e-6f

/* ========================================================================
 * Matrix Exponential - SO(3) [Rotation Only]
 * ======================================================================== */

mat3x3_t matrix_exp_so3(const vec3_t *w, float theta)
{
	/* Handle zero-angle case */
	if (is_near_zero(theta, EPSILON)) {
		return mat3x3_identity();
	}
	
	/* Compute magnitude of angular velocity */
	float w_norm = vec3_norm(w);
	
	/* Handle zero angular velocity */
	if (is_near_zero(w_norm, EPSILON)) {
		return mat3x3_identity();
	}
	
	/* Normalize ω to unit vector ω̂ */
	vec3_t w_unit;
	w_unit.x = w->x / w_norm;
	w_unit.y = w->y / w_norm;
	w_unit.z = w->z / w_norm;
	
	/* Compute [ω̂] (skew-symmetric matrix) */
	mat3x3_t w_skew = mat3x3_skew_symmetric(&w_unit);
	
	/* Compute [ω̂]² */
	mat3x3_t w_skew_sq = mat3x3_mul(&w_skew, &w_skew);
	
	/* Rodrigues' formula:
	 * exp([ω]θ) = I + sin(θ)[ω̂] + (1-cos(θ))[ω̂]² */
	float sin_theta = sinf(theta);
	float cos_theta = cosf(theta);
	
	mat3x3_t result = mat3x3_identity();
	mat3x3_t term1 = mat3x3_scale(&w_skew, sin_theta);
	mat3x3_t term2 = mat3x3_scale(&w_skew_sq, 1.0f - cos_theta);
	
	result = mat3x3_add(&result, &term1);
	result = mat3x3_add(&result, &term2);
	
	return result;
}

/* ========================================================================
 * Helper: Compute G(θ) Matrix
 * ======================================================================== */

mat3x3_t compute_g_matrix(const vec3_t *w_unit, float theta)
{
	/* G(θ) = I*θ + (1-cos(θ))[ω̂] + (θ-sin(θ))[ω̂]²
	 * 
	 * This matrix transforms the linear velocity v into the translation
	 * component of the homogeneous transformation.
	 */
	
	mat3x3_t w_skew = mat3x3_skew_symmetric(w_unit);
	mat3x3_t w_skew_sq = mat3x3_mul(&w_skew, &w_skew);
	
	float sin_theta = sinf(theta);
	float cos_theta = cosf(theta);
	
	/* Start with I*θ */
	mat3x3_t result = mat3x3_identity();
	result = mat3x3_scale(&result, theta);
	
	/* Add (1-cos(θ))[ω̂] */
	mat3x3_t term1 = mat3x3_scale(&w_skew, 1.0f - cos_theta);
	result = mat3x3_add(&result, &term1);
	
	/* Add (θ-sin(θ))[ω̂]² */
	mat3x3_t term2 = mat3x3_scale(&w_skew_sq, theta - sin_theta);
	result = mat3x3_add(&result, &term2);
	
	return result;
}

/* ========================================================================
 * Matrix Exponential - SE(3) [Rotation + Translation]
 * ======================================================================== */

mat4x4_t matrix_exp_se3(const vec6_t *xi, float theta)
{
	/* Handle zero-angle case */
	if (is_near_zero(theta, EPSILON)) {
		return mat4x4_identity();
	}
	
	/* Extract angular and linear parts */
	vec3_t w = xi->w;
	vec3_t v = xi->v;
	
	/* Compute magnitude of angular velocity */
	float w_norm = vec3_norm(&w);
	
	/* Case 1: Pure translation (ω ≈ 0)
	 * exp([ξ]θ) = [I, v*θ; 0, 1] */
	if (is_near_zero(w_norm, EPSILON)) {
		mat4x4_t result = mat4x4_identity();
		result.m[0][3] = v.x * theta;
		result.m[1][3] = v.y * theta;
		result.m[2][3] = v.z * theta;
		return result;
	}
	
	/* Case 2: General screw motion (ω ≠ 0)
	 * exp([ξ]θ) = [ exp([ω]θ)   G(θ)v ]
	 *             [     0          1   ] */
	
	/* Normalize ω to unit vector ω̂ */
	vec3_t w_unit;
	w_unit.x = w.x / w_norm;
	w_unit.y = w.y / w_norm;
	w_unit.z = w.z / w_norm;
	
	/* Compute rotation part: exp([ω̂]θ) using Rodrigues */
	mat3x3_t R = matrix_exp_so3(&w_unit, theta);
	
	/* Compute G(θ) matrix */
	mat3x3_t G = compute_g_matrix(&w_unit, theta);
	
	/* Compute translation part: p = G(θ) * v */
	vec3_t p = mat3x3_mul_vec3(&G, &v);
	
	/* Construct homogeneous transformation matrix */
	mat4x4_t result = mat4x4_from_rt(&R, &p);
	
	return result;
}
