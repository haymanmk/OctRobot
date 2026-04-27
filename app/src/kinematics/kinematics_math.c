/*
 * OctroBot Robot Arm Firmware - Kinematics Math Library Implementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "kinematics_math.h"
#include <string.h>

#define EPSILON 1e-6f

/* ========================================================================
 * Vec3 Operations
 * ======================================================================== */

float vec3_dot(const vec3_t *a, const vec3_t *b)
{
	return a->x * b->x + a->y * b->y + a->z * b->z;
}

vec3_t vec3_cross(const vec3_t *a, const vec3_t *b)
{
	vec3_t result;
	result.x = a->y * b->z - a->z * b->y;
	result.y = a->z * b->x - a->x * b->z;
	result.z = a->x * b->y - a->y * b->x;
	return result;
}

float vec3_norm(const vec3_t *v)
{
	return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

bool vec3_normalize(vec3_t *v)
{
	float norm = vec3_norm(v);
	if (norm < EPSILON) {
		return false;
	}
	v->x /= norm;
	v->y /= norm;
	v->z /= norm;
	return true;
}

vec3_t vec3_add(const vec3_t *a, const vec3_t *b)
{
	vec3_t result;
	result.x = a->x + b->x;
	result.y = a->y + b->y;
	result.z = a->z + b->z;
	return result;
}

vec3_t vec3_sub(const vec3_t *a, const vec3_t *b)
{
	vec3_t result;
	result.x = a->x - b->x;
	result.y = a->y - b->y;
	result.z = a->z - b->z;
	return result;
}

vec3_t vec3_scale(const vec3_t *v, float s)
{
	vec3_t result;
	result.x = v->x * s;
	result.y = v->y * s;
	result.z = v->z * s;
	return result;
}

/* ========================================================================
 * Mat3x3 Operations
 * ======================================================================== */

mat3x3_t mat3x3_identity(void)
{
	mat3x3_t m;
	memset(&m, 0, sizeof(m));
	m.m[0][0] = 1.0f;
	m.m[1][1] = 1.0f;
	m.m[2][2] = 1.0f;
	return m;
}

mat3x3_t mat3x3_zero(void)
{
	mat3x3_t m;
	memset(&m, 0, sizeof(m));
	return m;
}

vec3_t mat3x3_mul_vec3(const mat3x3_t *m, const vec3_t *v)
{
	vec3_t result;
	result.x = m->m[0][0] * v->x + m->m[0][1] * v->y + m->m[0][2] * v->z;
	result.y = m->m[1][0] * v->x + m->m[1][1] * v->y + m->m[1][2] * v->z;
	result.z = m->m[2][0] * v->x + m->m[2][1] * v->y + m->m[2][2] * v->z;
	return result;
}

mat3x3_t mat3x3_mul(const mat3x3_t *a, const mat3x3_t *b)
{
	mat3x3_t result;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			result.m[i][j] = 0.0f;
			for (int k = 0; k < 3; k++) {
				result.m[i][j] += a->m[i][k] * b->m[k][j];
			}
		}
	}
	return result;
}

mat3x3_t mat3x3_add(const mat3x3_t *a, const mat3x3_t *b)
{
	mat3x3_t result;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			result.m[i][j] = a->m[i][j] + b->m[i][j];
		}
	}
	return result;
}

mat3x3_t mat3x3_scale(const mat3x3_t *m, float s)
{
	mat3x3_t result;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			result.m[i][j] = m->m[i][j] * s;
		}
	}
	return result;
}

mat3x3_t mat3x3_transpose(const mat3x3_t *m)
{
	mat3x3_t result;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			result.m[i][j] = m->m[j][i];
		}
	}
	return result;
}

float mat3x3_trace(const mat3x3_t *m)
{
	return m->m[0][0] + m->m[1][1] + m->m[2][2];
}

mat3x3_t mat3x3_skew_symmetric(const vec3_t *w)
{
	mat3x3_t result;
	result.m[0][0] =  0.0f;
	result.m[0][1] = -w->z;
	result.m[0][2] =  w->y;
	result.m[1][0] =  w->z;
	result.m[1][1] =  0.0f;
	result.m[1][2] = -w->x;
	result.m[2][0] = -w->y;
	result.m[2][1] =  w->x;
	result.m[2][2] =  0.0f;
	return result;
}

bool mat3x3_is_equal(const mat3x3_t *a, const mat3x3_t *b, float tol)
{
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (!is_near_equal(a->m[i][j], b->m[i][j], tol)) {
				return false;
			}
		}
	}
	return true;
}

/* ========================================================================
 * Mat4x4 Operations
 * ======================================================================== */

mat4x4_t mat4x4_identity(void)
{
	mat4x4_t m;
	memset(&m, 0, sizeof(m));
	m.m[0][0] = 1.0f;
	m.m[1][1] = 1.0f;
	m.m[2][2] = 1.0f;
	m.m[3][3] = 1.0f;
	return m;
}

mat4x4_t mat4x4_zero(void)
{
	mat4x4_t m;
	memset(&m, 0, sizeof(m));
	return m;
}

mat4x4_t mat4x4_mul(const mat4x4_t *a, const mat4x4_t *b)
{
	mat4x4_t result;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			result.m[i][j] = 0.0f;
			for (int k = 0; k < 4; k++) {
				result.m[i][j] += a->m[i][k] * b->m[k][j];
			}
		}
	}
	return result;
}

mat4x4_t mat4x4_from_rt(const mat3x3_t *r, const vec3_t *t)
{
	mat4x4_t result = mat4x4_identity();
	
	/* Copy rotation part */
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			result.m[i][j] = r->m[i][j];
		}
	}
	
	/* Copy translation part */
	result.m[0][3] = t->x;
	result.m[1][3] = t->y;
	result.m[2][3] = t->z;
	
	return result;
}

mat3x3_t mat4x4_get_rotation(const mat4x4_t *m)
{
	mat3x3_t r;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			r.m[i][j] = m->m[i][j];
		}
	}
	return r;
}

vec3_t mat4x4_get_translation(const mat4x4_t *m)
{
	vec3_t t;
	t.x = m->m[0][3];
	t.y = m->m[1][3];
	t.z = m->m[2][3];
	return t;
}

void mat4x4_set_rotation(mat4x4_t *m, const mat3x3_t *r)
{
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			m->m[i][j] = r->m[i][j];
		}
	}
}

void mat4x4_set_translation(mat4x4_t *m, const vec3_t *t)
{
	m->m[0][3] = t->x;
	m->m[1][3] = t->y;
	m->m[2][3] = t->z;
}

mat4x4_t mat4x4_inverse_transform(const mat4x4_t *m)
{
	mat4x4_t inv;
	
	/* R^T (transpose upper-left 3x3) */
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			inv.m[i][j] = m->m[j][i];
		}
	}
	
	/* -R^T * p */
	inv.m[0][3] = -(inv.m[0][0] * m->m[0][3] + 
	                inv.m[0][1] * m->m[1][3] + 
	                inv.m[0][2] * m->m[2][3]);
	inv.m[1][3] = -(inv.m[1][0] * m->m[0][3] + 
	                inv.m[1][1] * m->m[1][3] + 
	                inv.m[1][2] * m->m[2][3]);
	inv.m[2][3] = -(inv.m[2][0] * m->m[0][3] + 
	                inv.m[2][1] * m->m[1][3] + 
	                inv.m[2][2] * m->m[2][3]);
	
	/* Bottom row [0 0 0 1] */
	inv.m[3][0] = 0.0f;
	inv.m[3][1] = 0.0f;
	inv.m[3][2] = 0.0f;
	inv.m[3][3] = 1.0f;
	
	return inv;
}

bool mat4x4_is_equal(const mat4x4_t *a, const mat4x4_t *b, float tol)
{
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (!is_near_equal(a->m[i][j], b->m[i][j], tol)) {
				return false;
			}
		}
	}
	return true;
}

/* ========================================================================
 * SE(3) Logarithm
 * ======================================================================== */

bool mat4x4_log_se3(const mat4x4_t *T, vec6_t *twist)
{
	/* Extract rotation and translation */
	mat3x3_t R = mat4x4_get_rotation(T);
	vec3_t p = mat4x4_get_translation(T);
	
	/* Compute angle from rotation matrix trace */
	float trace = mat3x3_trace(&R);
	float cos_theta = (trace - 1.0f) / 2.0f;
	
	/* Clamp for numerical stability */
	cos_theta = clamp_float(cos_theta, -1.0f, 1.0f);
	float theta = acosf(cos_theta);
	
	/* Case 1: theta ≈ 0 (identity rotation) */
	if (is_near_zero(theta, EPSILON)) {
		twist->w.x = 0.0f;
		twist->w.y = 0.0f;
		twist->w.z = 0.0f;
		twist->v = p;  /* Pure translation */
		return true;
	}
	
	/* Case 2: theta ≈ π (180° rotation) */
	if (is_near_equal(theta, M_PI, EPSILON)) {
		/* Find axis from eigenvector of R */
		/* Simplified: take column with largest diagonal element */
		int max_idx = 0;
		if (R.m[1][1] > R.m[max_idx][max_idx]) max_idx = 1;
		if (R.m[2][2] > R.m[max_idx][max_idx]) max_idx = 2;
		
		vec3_t axis;
		if (max_idx == 0) {
			axis.x = sqrtf((R.m[0][0] + 1.0f) / 2.0f);
			axis.y = R.m[0][1] / (2.0f * axis.x);
			axis.z = R.m[0][2] / (2.0f * axis.x);
		} else if (max_idx == 1) {
			axis.y = sqrtf((R.m[1][1] + 1.0f) / 2.0f);
			axis.x = R.m[1][0] / (2.0f * axis.y);
			axis.z = R.m[1][2] / (2.0f * axis.y);
		} else {
			axis.z = sqrtf((R.m[2][2] + 1.0f) / 2.0f);
			axis.x = R.m[2][0] / (2.0f * axis.z);
			axis.y = R.m[2][1] / (2.0f * axis.z);
		}
		
		twist->w = vec3_scale(&axis, theta);
		
		/* Compute linear part (complex, simplified version) */
		twist->v = vec3_scale(&p, 0.5f);
		return true;
	}
	
	/* Case 3: General case (0 < theta < π) */
	/* ω = (1 / 2sin(θ)) * (R - R^T)_vee */
	float coeff = 1.0f / (2.0f * sinf(theta));
	twist->w.x = coeff * (R.m[2][1] - R.m[1][2]);
	twist->w.y = coeff * (R.m[0][2] - R.m[2][0]);
	twist->w.z = coeff * (R.m[1][0] - R.m[0][1]);
	
	/* Normalize ω to unit length and scale by theta */
	vec3_normalize(&twist->w);
	twist->w = vec3_scale(&twist->w, theta);
	
	/* Compute linear part: v = G^-1 * p
	 * where G^-1 = (1/θ)I - (1/2)[ω] + (1/θ - cot(θ/2)/2)[ω]^2 */
	mat3x3_t w_skew = mat3x3_skew_symmetric(&twist->w);
	mat3x3_t w_skew_sq = mat3x3_mul(&w_skew, &w_skew);
	
	float a = 1.0f / theta;
	float b = -0.5f;
	float c = 1.0f / theta - 0.5f / tanf(theta / 2.0f);
	
	mat3x3_t G_inv = mat3x3_identity();
	mat3x3_t w_skew_scaled = mat3x3_scale(&w_skew, b);
	G_inv = mat3x3_add(&G_inv, &w_skew_scaled);
	mat3x3_t w_skew_sq_scaled = mat3x3_scale(&w_skew_sq, c);
	G_inv = mat3x3_add(&G_inv, &w_skew_sq_scaled);
	G_inv = mat3x3_scale(&G_inv, a);
	
	twist->v = mat3x3_mul_vec3(&G_inv, &p);
	
	return true;
}

/* ========================================================================
 * Adjoint Transformation
 * ======================================================================== */

mat6x6_t mat4x4_adjoint(const mat4x4_t *T)
{
	mat6x6_t ad;
	memset(&ad, 0, sizeof(ad));
	
	mat3x3_t R = mat4x4_get_rotation(T);
	vec3_t p = mat4x4_get_translation(T);
	mat3x3_t p_skew = mat3x3_skew_symmetric(&p);
	mat3x3_t pR = mat3x3_mul(&p_skew, &R);
	
	/* Upper-left 3x3: R */
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			ad.m[i][j] = R.m[i][j];
		}
	}
	
	/* Upper-right 3x3: [p]R */
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			ad.m[i][j+3] = pR.m[i][j];
		}
	}
	
	/* Lower-left 3x3: 0 */
	/* (already zeroed) */
	
	/* Lower-right 3x3: R */
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			ad.m[i+3][j+3] = R.m[i][j];
		}
	}
	
	return ad;
}

vec6_t mat6x6_mul_vec6(const mat6x6_t *ad, const vec6_t *xi)
{
	vec6_t result;
	float vec[6] = {xi->w.x, xi->w.y, xi->w.z, xi->v.x, xi->v.y, xi->v.z};
	float out[6] = {0};
	
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			out[i] += ad->m[i][j] * vec[j];
		}
	}
	
	result.w.x = out[0];
	result.w.y = out[1];
	result.w.z = out[2];
	result.v.x = out[3];
	result.v.y = out[4];
	result.v.z = out[5];
	
	return result;
}

/* ========================================================================
 * Pseudoinverse (Damped Least Squares)
 * ======================================================================== */

/* Helper: 6x6 matrix inverse (for JJ^T + λ²I) */
static bool invert_6x6(const float A[6][6], float A_inv[6][6])
{
	/* Simple Gauss-Jordan elimination for 6x6 matrix
	 * Not optimized, but sufficient for IK computation */
	float aug[6][12];
	
	/* Initialize augmented matrix [A | I] */
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			aug[i][j] = A[i][j];
			aug[i][j+6] = (i == j) ? 1.0f : 0.0f;
		}
	}
	
	/* Forward elimination */
	for (int k = 0; k < 6; k++) {
		/* Find pivot */
		int pivot = k;
		for (int i = k + 1; i < 6; i++) {
			if (fabsf(aug[i][k]) > fabsf(aug[pivot][k])) {
				pivot = i;
			}
		}
		
		/* Check for singularity */
		if (fabsf(aug[pivot][k]) < EPSILON) {
			return false;
		}
		
		/* Swap rows */
		if (pivot != k) {
			for (int j = 0; j < 12; j++) {
				float tmp = aug[k][j];
				aug[k][j] = aug[pivot][j];
				aug[pivot][j] = tmp;
			}
		}
		
		/* Scale pivot row */
		float scale = 1.0f / aug[k][k];
		for (int j = 0; j < 12; j++) {
			aug[k][j] *= scale;
		}
		
		/* Eliminate column */
		for (int i = 0; i < 6; i++) {
			if (i != k) {
				float factor = aug[i][k];
				for (int j = 0; j < 12; j++) {
					aug[i][j] -= factor * aug[k][j];
				}
			}
		}
	}
	
	/* Extract inverse from augmented matrix */
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			A_inv[i][j] = aug[i][j+6];
		}
	}
	
	return true;
}

bool jacobian_damped_pinv(const float J[6][6], float J_pinv[6][6], 
                           float lambda, int n_joints)
{
	/* Compute JJ^T + λ²I */
	float JJT[6][6];
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			JJT[i][j] = 0.0f;
			for (int k = 0; k < n_joints; k++) {
				JJT[i][j] += J[i][k] * J[j][k];
			}
		}
		JJT[i][i] += lambda * lambda;  /* Add damping */
	}
	
	/* Invert JJ^T + λ²I */
	float JJT_inv[6][6];
	if (!invert_6x6(JJT, JJT_inv)) {
		return false;
	}
	
	/* Compute J^+ = J^T * (JJ^T + λ²I)^-1 */
	for (int i = 0; i < n_joints; i++) {
		for (int j = 0; j < 6; j++) {
			J_pinv[i][j] = 0.0f;
			for (int k = 0; k < 6; k++) {
				J_pinv[i][j] += J[k][i] * JJT_inv[k][j];
			}
		}
	}
	
	return true;
}

/* ========================================================================
 * Utility Functions
 * ======================================================================== */

float wrap_to_pi(float angle)
{
	/* Wrap angle to [-π, π] */
	while (angle > M_PI) {
		angle -= 2.0f * M_PI;
	}
	while (angle < -M_PI) {
		angle += 2.0f * M_PI;
	}
	return angle;
}
