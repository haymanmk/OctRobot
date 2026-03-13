/*
 * OctroBot Robot Arm Firmware - Host Communication Header
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB CDC-ACM based host communication for:
 * - Manual control commands
 * - Demo recording/playback
 * - System status reporting
 *
 * Educational Note:
 * This module implements a simple blocking command parser for Phase 3b.
 * In later phases, it will be upgraded to a threaded implementation with
 * message queues for non-blocking operation.
 */

#ifndef HOST_COMMS_H
#define HOST_COMMS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Robot State Structure
 */
struct robot_state {
	float joint_angles[6];     /* Current joint angles in radians */
	uint8_t temperatures[6];   /* Servo temperatures in °C */
	uint8_t voltages[6];       /* Servo voltages in 0.1V units */
	uint16_t loads[6];         /* Servo loads */
	bool is_moving;            /* True if any joint is moving */
	uint8_t error_flags;       /* Error flags (bit field) */
};

/**
 * @brief Initialize host communication over USB CDC-ACM.
 * 
 * This function:
 * - Initializes USB device stack
 * - Opens CDC-ACM device
 * - Sets up receive buffer
 * 
 * @return 0 on success, negative error code on failure
 */
int host_comms_init(void);

/**
 * @brief Enable or disable manual control mode.
 * 
 * When manual control mode is enabled, system logging to UART is
 * suppressed to prevent interference with packet communication.
 * 
 * @param enable True to enable manual mode, false to disable
 */
void host_comms_set_manual_mode(bool enable);

/**
 * @brief Check if manual control mode is active.
 * 
 * @return True if manual mode is active, false otherwise
 */
bool host_comms_is_manual_mode(void);

/**
 * @brief Check for and process incoming commands (blocking).
 * 
 * Call this periodically from main loop to handle host commands.
 * In Phase 3b, this is a simple blocking implementation.
 * 
 * Educational Note:
 * This function reads bytes from USB CDC, searches for packet start marker,
 * assembles complete packets, and dispatches commands. Later phases will
 * move this to a dedicated thread for non-blocking operation.
 * 
 * @param timeout_ms Maximum time to wait for data (0 = non-blocking)
 * @return 0 if command processed, -1 if no command or error
 */
int host_comms_process(uint32_t timeout_ms);

/**
 * @brief Send status report to host.
 * 
 * Sends current robot state as a STATUS_REPORT packet.
 * 
 * @param state Pointer to current robot state
 * @return 0 on success, negative error code on failure
 */
int host_comms_send_status(const struct robot_state *state);

/**
 * @brief Send demo recording status to host.
 * 
 * @param demo_id Demo slot ID (0-2)
 * @param waypoint_count Number of waypoints recorded
 * @param bytes_remaining Remaining flash space
 * @return 0 on success, negative error code on failure
 */
int host_comms_send_demo_status(uint8_t demo_id, uint8_t waypoint_count,
                                 uint16_t bytes_remaining);

/**
 * @brief Get current robot state (for status reports).
 * 
 * This function queries all servos and populates the robot_state structure.
 * 
 * @param state Pointer to robot_state structure to populate
 * @return 0 on success, negative error code on failure
 */
int host_comms_get_robot_state(struct robot_state *state);

#ifdef __cplusplus
}
#endif

#endif /* HOST_COMMS_H */
