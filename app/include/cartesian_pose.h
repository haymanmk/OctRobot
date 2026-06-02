/*
 * OctroBot Robot Arm Firmware - Cartesian Pose -> Joints (pure core)
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure (hardware-free) conversion of a commanded Cartesian pose into
 * model-frame joint angles via the IK solver. RPY uses the ZYX intrinsic
 * convention; angles in degrees, position in meters.
 */

#ifndef CARTESIAN_POSE_H
#define CARTESIAN_POSE_H

#include "robot_geometry.h"   /* poe_robot_model_t, NUM_JOINTS */

#ifdef __cplusplus
extern "C" {
#endif

/** Result of a Cartesian move-to-pose request. */
typedef enum {
	CMOVE_OK = 0,         /* solved (and, in the wrapper, motion sent)     */
	CMOVE_NO_SOLUTION,    /* IK did not converge                           */
	CMOVE_OUT_OF_LIMITS,  /* solution violates joint/servo limits          */
	CMOVE_ESTOP,          /* emergency stop active (wrapper only)          */
	CMOVE_SERVO_ERR,      /* servo read/write failure (wrapper only)       */
	CMOVE_BAD_ARGS,       /* NULL/invalid argument                         */
} cmove_status_t;

/**
 * Compute model-frame joint angles for a Cartesian pose.
 *
 * @param model       robot geometry
 * @param x,y,z       target position in meters (model frame)
 * @param roll_deg,pitch_deg,yaw_deg  target orientation, ZYX intrinsic, degrees
 * @param seed_theta  IK initial guess (model radians)
 * @param out_theta   solved joint angles (model radians)
 * @param iters_used  if non-NULL, receives the IK iteration count (debug)
 * @return CMOVE_OK / CMOVE_NO_SOLUTION / CMOVE_OUT_OF_LIMITS / CMOVE_BAD_ARGS.
 *         out_theta is written on both CMOVE_OK and CMOVE_OUT_OF_LIMITS (the
 *         final IK iterate); it is left untouched on the other statuses.
 *         Callers must not command motion unless the status is CMOVE_OK.
 */
cmove_status_t cartesian_pose_to_joints(const poe_robot_model_t *model,
					float x, float y, float z,
					float roll_deg, float pitch_deg,
					float yaw_deg,
					const float seed_theta[NUM_JOINTS],
					float out_theta[NUM_JOINTS],
					int *iters_used);

#ifdef __cplusplus
}
#endif

#endif /* CARTESIAN_POSE_H */
