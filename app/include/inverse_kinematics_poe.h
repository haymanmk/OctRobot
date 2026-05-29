/*
 * OctroBot Robot Arm Firmware - Inverse Kinematics (POE, body-frame)
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Body-frame Newton-Raphson IK solver (Modern Robotics IKinBody).
 * Offline-validated against the modern_robotics Python library.
 */

#ifndef INVERSE_KINEMATICS_POE_H
#define INVERSE_KINEMATICS_POE_H

#include "kinematics_math.h"
#include "robot_geometry.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Outcome of an IK solve. */
typedef enum {
	IK_SUCCESS = 0,    /* converged, all joints within limits      */
	IK_OUT_OF_LIMITS,  /* converged, but >=1 joint out of limits   */
	IK_NO_CONVERGENCE, /* hit max_iters without converging         */
	IK_INVALID_INPUT,  /* NULL argument                            */
} ik_status_t;

/** Solver tuning parameters. */
typedef struct {
	float eomg;      /* angular error tolerance (rad)   */
	float ev;        /* linear error tolerance (m)      */
	int   max_iters; /* maximum Newton-Raphson iterations */
	float lambda;    /* damped-least-squares factor (0 = pure pinv) */
} ik_params_t;

/**
 * Default parameters: eomg=1e-3, ev=1e-4, max_iters=20, lambda=0.0.
 * lambda=0.0 reproduces modern_robotics.IKinBody for clean cross-validation.
 */
ik_params_t inverse_kinematics_default_params(void);

/**
 * Solve IK for a desired end-effector pose.
 *
 * Inputs:
 *   - model:     robot geometry (space-frame screw axes + home config M)
 *   - T_target:  desired end-effector pose T_sd
 *   - seed:      initial joint-angle guess (radians)
 *   - params:    tuning parameters, or NULL for defaults
 * Outputs:
 *   - out_angles:  final joint angles, wrapped to [-pi, pi] (always written)
 *   - iters_used:  iterations performed (optional, may be NULL)
 *
 * Returns an ik_status_t. out_angles always holds the final iterate, even on
 * non-convergence.
 */
ik_status_t inverse_kinematics_compute(const poe_robot_model_t *model,
				       const mat4x4_t *T_target,
				       const float seed[NUM_JOINTS],
				       float out_angles[NUM_JOINTS],
				       const ik_params_t *params,
				       int *iters_used);

#ifdef __cplusplus
}
#endif

#endif /* INVERSE_KINEMATICS_POE_H */
