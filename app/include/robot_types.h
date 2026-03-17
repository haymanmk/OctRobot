/*
 * OctroBot Robot Arm Firmware - Robot Types
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Common robot data structures used across multiple modules.
 */

#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Robot State Structure
 * 
 * Contains real-time state information from all joints.
 * Used by: comms, kinematics, controller, trajectory planner.
 */
struct robot_state {
	float joint_angles[6];     /* Current joint angles in radians */
	uint8_t temperatures[6];   /* Servo temperatures in °C */
	uint8_t voltages[6];       /* Servo voltages in 0.1V units */
	uint16_t loads[6];         /* Servo loads */
	bool is_moving;            /* True if any joint is moving */
	uint8_t error_flags;       /* Error flags (bit field) */
};

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_TYPES_H */
