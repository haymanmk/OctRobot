/*
 * OctroBot Robot Arm Firmware - Joint Calibration Map
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Maps kinematic-model joint angles (radians, model frame, theta=0 => home
 * config M) to physical servo angles (degrees, servo center = 0 deg) and back,
 * via a per-joint sign + offset calibration table:
 *
 *   servo_deg[i] = sign[i] * rad_to_deg(theta_rad[i]) + offset_deg[i]
 *
 * The table defaults to identity (sign=+1, offset=0) and is set at runtime via
 * joint_map_set_calibration (used by tests now; the hook for NVS field
 * calibration later).
 */

#ifndef JOINT_MAP_H
#define JOINT_MAP_H

#include "robot_geometry.h"   /* NUM_JOINTS */

#ifdef __cplusplus
extern "C" {
#endif

/** model radians -> servo degrees (forward). */
void joint_map_model_to_servo(const float theta_rad[NUM_JOINTS],
			      float servo_deg[NUM_JOINTS]);

/** servo degrees -> model radians (inverse). */
void joint_map_servo_to_model(const float servo_deg[NUM_JOINTS],
			      float theta_rad[NUM_JOINTS]);

/** Install a calibration table (copies both arrays). */
void joint_map_set_calibration(const float sign[NUM_JOINTS],
			       const float offset_deg[NUM_JOINTS]);

#ifdef __cplusplus
}
#endif

#endif /* JOINT_MAP_H */
