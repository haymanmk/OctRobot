/*
 * OctroBot Robot Arm Firmware - Inverse Kinematics Implementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Body-frame Newton-Raphson IK (Modern Robotics IKinBody).
 *   T_sb  = FK(theta)            (space-frame FK; same end pose as body FK)
 *   Vb    = log(T_sb^-1 * T_sd)  (body twist error)
 *   theta = theta + Jb^+ * Vb    (damped least-squares step)
 */

#include "inverse_kinematics_poe.h"
#include "forward_kinematics_poe.h"
#include "matrix_exp.h"
#include <string.h>

/*
 * Body screw list: B_i = Ad_{M^-1} * S_i.
 * Valid because exp([S]th)...M == M*exp([B]th)...  iff  B_i = Ad_{M^-1} S_i.
 */
static void derive_body_screw_axes(const poe_robot_model_t *model,
				   vec6_t Blist[NUM_JOINTS])
{
	mat4x4_t M = robot_geometry_get_home_config(model);
	mat4x4_t M_inv = mat4x4_inverse_transform(&M);
	mat6x6_t AdMinv = mat4x4_adjoint(&M_inv);

	for (int i = 0; i < NUM_JOINTS; i++) {
		vec6_t Si;
		robot_geometry_get_screw_axis(model, i, &Si);
		Blist[i] = mat6x6_mul_vec6(&AdMinv, &Si);
	}
}

/*
 * Body Jacobian Jb(theta), stored as J[6][6] with rows [wx,wy,wz,vx,vy,vz].
 *   Jb[:, n-1] = B_{n-1}
 *   T = I; for i = n-2 .. 0:  T = T * exp(-[B_{i+1}] theta_{i+1});
 *                             Jb[:, i] = Ad_T * B_i
 */
static void body_jacobian(const vec6_t Blist[NUM_JOINTS],
			  const float theta[NUM_JOINTS],
			  float J[6][6])
{
	vec6_t col[NUM_JOINTS];

	col[NUM_JOINTS - 1] = Blist[NUM_JOINTS - 1];

	mat4x4_t T = mat4x4_identity();
	for (int i = NUM_JOINTS - 2; i >= 0; i--) {
		mat4x4_t E = matrix_exp_se3(&Blist[i + 1], -theta[i + 1]);
		T = mat4x4_mul(&T, &E);
		mat6x6_t Ad = mat4x4_adjoint(&T);
		col[i] = mat6x6_mul_vec6(&Ad, &Blist[i]);
	}

	for (int c = 0; c < NUM_JOINTS; c++) {
		J[0][c] = col[c].w.x;
		J[1][c] = col[c].w.y;
		J[2][c] = col[c].w.z;
		J[3][c] = col[c].v.x;
		J[4][c] = col[c].v.y;
		J[5][c] = col[c].v.z;
	}
}

ik_params_t inverse_kinematics_default_params(void)
{
	ik_params_t p = {
		.eomg = 1e-3f,
		.ev = 1e-4f,
		.max_iters = 50,
		/* Damped least-squares: lambda>0 keeps JJ^T invertible at singular
		 * configs (lambda=0 aborts the solve there) and caps the step gain
		 * near singularities at ~1/(2*lambda), so far targets converge
		 * instead of overshooting. Small enough not to spoil the tight
		 * eomg/ev tolerance for well-conditioned poses. */
		.lambda = 0.01f,
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

	ik_params_t p = (params != NULL) ? *params
					 : inverse_kinematics_default_params();

	vec6_t Blist[NUM_JOINTS];
	derive_body_screw_axes(model, Blist);

	float theta[NUM_JOINTS];
	memcpy(theta, seed, sizeof(theta));

	bool converged = false;
	int iter = 0;

	for (iter = 0; iter < p.max_iters; iter++) {
		mat4x4_t T_sb;
		if (!forward_kinematics_compute(model, theta, &T_sb)) {
			break;
		}

		mat4x4_t T_sb_inv = mat4x4_inverse_transform(&T_sb);
		mat4x4_t T_bd = mat4x4_mul(&T_sb_inv, T_target);

		vec6_t Vb;
		if (!mat4x4_log_se3(&T_bd, &Vb)) {
			break;
		}

		if (vec3_norm(&Vb.w) < p.eomg && vec3_norm(&Vb.v) < p.ev) {
			converged = true;
			break;
		}

		float J[6][6];
		body_jacobian(Blist, theta, J);

		float Jpinv[6][6];
		if (!jacobian_damped_pinv(J, Jpinv, p.lambda, NUM_JOINTS)) {
			break;
		}

		const float Vb_arr[6] = {
			Vb.w.x, Vb.w.y, Vb.w.z, Vb.v.x, Vb.v.y, Vb.v.z
		};
		for (int i = 0; i < NUM_JOINTS; i++) {
			float dtheta = 0.0f;
			for (int k = 0; k < 6; k++) {
				dtheta += Jpinv[i][k] * Vb_arr[k];
			}
			theta[i] += dtheta;
		}
	}

	for (int i = 0; i < NUM_JOINTS; i++) {
		out_angles[i] = wrap_to_pi(theta[i]);
	}
	if (iters_used != NULL) {
		*iters_used = iter;
	}

	if (!converged) {
		return IK_NO_CONVERGENCE;
	}
	if (!robot_geometry_check_joint_limits(model, out_angles)) {
		return IK_OUT_OF_LIMITS;
	}
	return IK_SUCCESS;
}
