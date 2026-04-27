/*
 * OctroBot Robot Arm Firmware - Kinematics Math Library
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal linear algebra library for POE kinematics.
 * Provides: vec3, vec4, mat3x3, mat4x4, quaternion operations.
 * 
 * Design: Hand-rolled, no external dependencies (Eigen/BLAS).
 * Target: ESP32 with FPU (single-precision float).
 */

#ifndef KINEMATICS_MATH_H
#define KINEMATICS_MATH_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Define M_PI if not already defined */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Data Structures
 * ======================================================================== */

/**
 * 3D vector (position or angular velocity)
 */
typedef struct {
	float x, y, z;
} vec3_t;

/**
 * 6D spatial vector (screw axis: ω + v)
 */
typedef struct {
	vec3_t w;  /* Angular velocity part (ω) */
	vec3_t v;  /* Linear velocity part (v) */
} vec6_t;

/**
 * 3x3 matrix (rotation matrices, skew-symmetric matrices)
 * Row-major storage: m[row][col]
 */
typedef struct {
	float m[3][3];
} mat3x3_t;

/**
 * 4x4 homogeneous transformation matrix
 * Row-major storage: m[row][col]
 * Layout:
 *   [ R R R p ]
 *   [ R R R p ]
 *   [ R R R p ]
 *   [ 0 0 0 1 ]
 */
typedef struct {
	float m[4][4];
} mat4x4_t;

/**
 * 6x6 adjoint matrix (for transforming screw coordinates)
 */
typedef struct {
	float m[6][6];
} mat6x6_t;

/* ========================================================================
 * Vec3 Operations
 * ======================================================================== */

/**
 * Create vec3 from components
 */
static inline vec3_t vec3_create(float x, float y, float z)
{
	vec3_t v = {x, y, z};
	return v;
}

/**
 * Vector dot product: a · b
 */
float vec3_dot(const vec3_t *a, const vec3_t *b);

/**
 * Vector cross product: a × b
 */
vec3_t vec3_cross(const vec3_t *a, const vec3_t *b);

/**
 * Vector norm (magnitude): |v|
 */
float vec3_norm(const vec3_t *v);

/**
 * Normalize vector to unit length: v / |v|
 * Returns false if vector is near-zero
 */
bool vec3_normalize(vec3_t *v);

/**
 * Vector addition: a + b
 */
vec3_t vec3_add(const vec3_t *a, const vec3_t *b);

/**
 * Vector subtraction: a - b
 */
vec3_t vec3_sub(const vec3_t *a, const vec3_t *b);

/**
 * Scalar multiplication: s * v
 */
vec3_t vec3_scale(const vec3_t *v, float s);

/* ========================================================================
 * Mat3x3 Operations
 * ======================================================================== */

/**
 * Create identity matrix
 */
mat3x3_t mat3x3_identity(void);

/**
 * Create zero matrix
 */
mat3x3_t mat3x3_zero(void);

/**
 * Matrix-vector multiplication: M * v
 */
vec3_t mat3x3_mul_vec3(const mat3x3_t *m, const vec3_t *v);

/**
 * Matrix-matrix multiplication: A * B
 */
mat3x3_t mat3x3_mul(const mat3x3_t *a, const mat3x3_t *b);

/**
 * Matrix addition: A + B
 */
mat3x3_t mat3x3_add(const mat3x3_t *a, const mat3x3_t *b);

/**
 * Scalar multiplication: s * M
 */
mat3x3_t mat3x3_scale(const mat3x3_t *m, float s);

/**
 * Matrix transpose: M^T
 */
mat3x3_t mat3x3_transpose(const mat3x3_t *m);

/**
 * Matrix trace: tr(M) = M[0][0] + M[1][1] + M[2][2]
 */
float mat3x3_trace(const mat3x3_t *m);

/**
 * Skew-symmetric matrix from vector: [ω]
 * Used in Rodrigues formula and screw theory.
 * 
 * [ω] = [  0  -ω_z  ω_y ]
 *       [ ω_z   0  -ω_x ]
 *       [-ω_y  ω_x   0  ]
 */
mat3x3_t mat3x3_skew_symmetric(const vec3_t *w);

/**
 * Check if matrix is approximately equal (tolerance 1e-6)
 */
bool mat3x3_is_equal(const mat3x3_t *a, const mat3x3_t *b, float tol);

/* ========================================================================
 * Mat4x4 Operations
 * ======================================================================== */

/**
 * Create identity matrix
 */
mat4x4_t mat4x4_identity(void);

/**
 * Create zero matrix
 */
mat4x4_t mat4x4_zero(void);

/**
 * Matrix-matrix multiplication: A * B
 * Critical for FK computation (chaining transforms)
 */
mat4x4_t mat4x4_mul(const mat4x4_t *a, const mat4x4_t *b);

/**
 * Create homogeneous transform from rotation + translation
 */
mat4x4_t mat4x4_from_rt(const mat3x3_t *r, const vec3_t *t);

/**
 * Extract rotation part (upper-left 3x3)
 */
mat3x3_t mat4x4_get_rotation(const mat4x4_t *m);

/**
 * Extract translation part (first 3 elements of last column)
 */
vec3_t mat4x4_get_translation(const mat4x4_t *m);

/**
 * Set rotation part (upper-left 3x3)
 */
void mat4x4_set_rotation(mat4x4_t *m, const mat3x3_t *r);

/**
 * Set translation part (first 3 elements of last column)
 */
void mat4x4_set_translation(mat4x4_t *m, const vec3_t *t);

/**
 * Inverse of homogeneous transform (assumes valid SE(3) matrix)
 * T^-1 = [ R^T  -R^T*p ]
 *        [  0      1   ]
 */
mat4x4_t mat4x4_inverse_transform(const mat4x4_t *m);

/**
 * Check if matrix is approximately equal (tolerance 1e-6)
 */
bool mat4x4_is_equal(const mat4x4_t *a, const mat4x4_t *b, float tol);

/* ========================================================================
 * SE(3) Logarithm (Matrix Logarithm for Homogeneous Transforms)
 * ======================================================================== */

/**
 * Matrix logarithm: log(T) → spatial twist ξ
 * Returns 6D twist vector in se(3).
 * 
 * Used by IK solver to compute error between target and current pose.
 * 
 * Returns twist vector [ω; v] where:
 *   - ω is angular velocity (3D)
 *   - v is linear velocity (3D)
 * 
 * Returns false if computation fails (e.g., invalid rotation).
 */
bool mat4x4_log_se3(const mat4x4_t *T, vec6_t *twist);

/* ========================================================================
 * Adjoint Transformation
 * ======================================================================== */

/**
 * Compute adjoint matrix from homogeneous transform: Ad_T
 * Used for transforming screw axes between frames.
 * 
 * Ad_T = [ R      [p]R ]
 *        [ 0       R   ]
 * 
 * where [p] is skew-symmetric matrix of position vector.
 */
mat6x6_t mat4x4_adjoint(const mat4x4_t *T);

/**
 * Apply adjoint transformation to screw axis: Ad_T * ξ
 */
vec6_t mat6x6_mul_vec6(const mat6x6_t *ad, const vec6_t *xi);

/* ========================================================================
 * Pseudoinverse (for Jacobian damped least squares)
 * ======================================================================== */

/**
 * Compute damped pseudoinverse of 6xN Jacobian: J^+ = J^T (JJ^T + λ²I)^-1
 * Used in IK Newton-Raphson solver near singularities.
 * 
 * Inputs:
 *   - J: 6xN Jacobian matrix (N = number of joints, typically 6)
 *   - lambda: damping factor (typically 0.01 to 0.1)
 *   - n_joints: number of columns in J
 * 
 * Output:
 *   - J_pinv: Nx6 pseudoinverse matrix
 * 
 * Returns false if computation fails.
 */
bool jacobian_damped_pinv(const float J[6][6], float J_pinv[6][6], 
                           float lambda, int n_joints);

/* ========================================================================
 * Utility Functions
 * ======================================================================== */

/**
 * Clamp value to [min, max]
 * Note: Zephyr provides a clamp() macro, but we define this as a function
 * for type safety and to avoid macro expansion issues in our context.
 */
static inline float clamp_float(float val, float min, float max)
{
	if (val < min) return min;
	if (val > max) return max;
	return val;
}

/**
 * Check if float is approximately zero
 */
static inline bool is_near_zero(float val, float tol)
{
	return fabsf(val) < tol;
}

/**
 * Check if two floats are approximately equal
 */
static inline bool is_near_equal(float a, float b, float tol)
{
	return fabsf(a - b) < tol;
}

/**
 * Wrap angle to [-π, π]
 */
float wrap_to_pi(float angle);

/**
 * Convert degrees to radians
 */
static inline float deg_to_rad(float deg)
{
	return deg * (M_PI / 180.0f);
}

/**
 * Convert radians to degrees
 */
static inline float rad_to_deg(float rad)
{
	return rad * (180.0f / M_PI);
}

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_MATH_H */
