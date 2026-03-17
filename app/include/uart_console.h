/**
 * OctroBot Robot Arm Firmware - UART Console Implementation
 * Copyright (c) 2026 OctroBot Project
 * 
 * SPDX-License-Identifier: Apache-2.0
 * Command protocol for host communication over USB-UART (CH340) console.
 * Format: $CMD [PAYLOAD ...] \n
 */

#ifndef OCTROBOT_UART_CONSOLE_H
#define OCTROBOT_UART_CONSOLE_H

#include "feetech_servo.h"

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * Constants
 */
#define UART_CONSOLE_START_BYTE '$'
#define UART_CONSOLE_MAX_PAYLOAD 128
#define UART_CONSOLE_SPLIT_CHAR ' '
#define UART_CONSOLE_TERMINATOR '\n'

int uart_console_init(void);

/**
 * @brief Get the current state of a specific servo joint
 * @param joint_id ID of the joint to query (1-6)
 * @param state Pointer to a feetech_servo_state struct to populate with the current state
 * @return 0 on success, negative error code on failure
 */
int uart_console_get_servo_state(uint8_t joint_id, struct feetech_servo_state *state);

/**
 * @brief Send the current state of a specific servo joint back to the host
 * @param joint_id ID of the joint to report (1-6)
 * @param state Pointer to a feetech_servo_state struct containing the state to send
 * @return 0 on success, negative error code on failure
 */
int uart_console_send_servo_state(uint8_t joint_id, const struct feetech_servo_state *state);

/**
 * @brief Collect current servo states for status reporting
 * This function will query all servos for their current angles,
 * temperatures, voltages, and loads, and populate the robot_state structure,
 * and then send this information back to the host in a structured format.
 * The response format is:
 * "$read JOINT_ID TEMP_C VOLTAGE_V LOAD_PCT ANGLE_DEG IS_MOVING ERROR\n"
 * @return 0 on success, negative error code on failure
 */
int uart_console_get_servo_states();

/**
 * @brief Set the desired position of a specific joint
 * @param joint_id ID of the joint to move (1-6)
 * @param angle_deg Desired angle in degrees (-180 to +180)
 * @return 0 on success, negative error code on failure
 */
int uart_console_set_joint_position(uint8_t joint_id, float angle_deg);

/**
 * @brief Jog a specific joint in a direction at a certain speed
 * @param joint_id ID of the joint to jog (1-6)
 * @param direction '+' for positive, '-' for negative
 * @param degrees Step size in degrees (1-180)
 * @return 0 on success, negative error code on failure
 */
int uart_console_jog_joint(uint8_t joint_id, char direction, uint8_t degrees);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* OCTROBOT_UART_CONSOLE_H */