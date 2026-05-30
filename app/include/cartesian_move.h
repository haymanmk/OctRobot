/*
 * OctroBot Robot Arm Firmware - Cartesian Move-to-Pose (hardware wrapper)
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Drives the arm to a commanded Cartesian pose: checks e-stop, reads the
 * current servo angles for the IK seed, runs cartesian_pose_to_joints, maps the
 * solution to servo degrees, range-checks, and sync-writes goal angles with an
 * onboard move time. RPY is ZYX intrinsic (degrees); position in meters.
 */

#ifndef CARTESIAN_MOVE_H
#define CARTESIAN_MOVE_H

#include <stdint.h>
#include "cartesian_pose.h"   /* cmove_status_t */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Move the arm to a Cartesian pose.
 *
 * @param x,y,z       target position in meters (model frame)
 * @param roll_deg,pitch_deg,yaw_deg  target orientation, ZYX intrinsic, degrees
 * @param move_time_ms  servo move duration in ms (onboard smoothing)
 * @return cmove_status_t (CMOVE_OK on success; no motion is commanded on any
 *         non-OK status)
 */
cmove_status_t cartesian_move_to_pose(float x, float y, float z,
				      float roll_deg, float pitch_deg,
				      float yaw_deg, uint16_t move_time_ms);

#ifdef __cplusplus
}
#endif

#endif /* CARTESIAN_MOVE_H */
