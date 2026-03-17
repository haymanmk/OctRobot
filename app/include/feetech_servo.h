/*
 * OctroBot Robot Arm Firmware - Feetech Servo Driver
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * High-level servo control API for Feetech SCS/STS servos.
 */

#ifndef OCTROBOT_FEETECH_SERVO_H
#define OCTROBOT_FEETECH_SERVO_H

#include "hal_types.h"
#include "half_duplex_uart.h"

/* Servo position range (typical for STS3215) */
#define FEETECH_POS_MIN      0
#define FEETECH_POS_MAX      4095
#define FEETECH_POS_CENTER   2048

/* Convert between servo ticks and degrees */
#define FEETECH_POS_TO_DEG(pos)   (((float)(pos) - FEETECH_POS_CENTER) * (360.0f / 4096.0f))
#define FEETECH_DEG_TO_POS(deg)   ((uint16_t)(((deg) * (4096.0f / 360.0f)) + FEETECH_POS_CENTER))

/* Velocity range (typical for STS3215) */
#define FEETECH_VEL_MIN      0
#define FEETECH_VEL_MAX      4095
#define FEETECH_DEFAULT_SPEED 1000  /* Default speed in steps/second (~50% max) */

/* Maximum number of servos in robot */
#define FEETECH_MAX_SERVOS   6

/**
 * @brief Servo configuration structure
 */
struct feetech_servo_config {
	uint8_t id;                     /* Servo ID (1-253) */
	uint16_t pos_min;               /* Minimum position limit (ticks) */
	uint16_t pos_max;               /* Maximum position limit (ticks) */
	uint16_t pos_offset;            /* Position offset for calibration */
	bool reverse;                   /* Reverse direction flag */
};

/**
 * @brief Servo state structure
 */
struct feetech_servo_state {
	uint16_t present_position;      /* Current position (ticks) */
	uint16_t present_speed;         /* Current speed */
	uint16_t present_load;          /* Current load */
	uint8_t present_voltage;        /* Current voltage (0.1V units) */
	uint8_t present_temperature;    /* Current temperature (°C) */
	bool is_moving;                 /* Movement status */
	uint8_t error;                  /* Error flags */
};

/**
 * @brief Initialize Feetech servo driver
 *
 * @param uart UART handle for servo communication
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_init(hal_uart_handle_t uart);

/**
 * @brief Ping servo to check if it's responding
 *
 * @param id Servo ID
 * @return HAL_OK if servo responds, error code otherwise
 */
int feetech_servo_ping(uint8_t id);

/**
 * @brief Enable/disable servo torque
 *
 * @param id Servo ID
 * @param enable true to enable torque, false to disable
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_set_torque_enable(uint8_t id, bool enable);

/**
 * @brief Set servo goal position (in ticks)
 *
 * Sets goal position with default time (0 = immediate) and default speed
 * (FEETECH_DEFAULT_SPEED = 2000 steps/second).
 * Uses optimized 6-byte write per Feetech protocol (0x2A-0x2F).
 *
 * @param id Servo ID
 * @param position Goal position (0-4095)
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_set_goal_position(uint8_t id, uint16_t position);

/**
 * @brief Set servo goal position with time and speed control (extended)
 *
 * Sets position, time, and speed in single 6-byte write to 0x2A-0x2F
 * per Feetech protocol. More efficient than separate writes.
 *
 * @param id Servo ID
 * @param position Goal position (0-4095)
 * @param time_ms Time to reach position in milliseconds (0 = immediate)
 * @param speed Maximum speed during movement in steps/second (0 = no limit)
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_set_goal_position_ex(uint8_t id, uint16_t position,
                                        uint16_t time_ms, uint16_t speed);

/**
 * @brief Set servo goal position (in degrees)
 *
 * @param id Servo ID
 * @param angle Goal angle in degrees (-180 to +180)
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_set_goal_angle(uint8_t id, float angle);

/**
 * @brief Set servo goal speed
 *
 * @param id Servo ID
 * @param speed Goal speed (0-4095)
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_set_goal_speed(uint8_t id, uint16_t speed);

/**
 * @brief Set servo goal time (trajectory duration)
 *
 * @param id Servo ID
 * @param time_ms Goal time in milliseconds
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_set_goal_time(uint8_t id, uint16_t time_ms);

/**
 * @brief Set servo acceleration
 *
 * @param id Servo ID
 * @param acceleration Acceleration value (0-254)
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_set_acceleration(uint8_t id, uint8_t acceleration);

/**
 * @brief Read servo current position (in ticks)
 *
 * @param id Servo ID
 * @param position Pointer to store position
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_read_position(uint8_t id, uint16_t *position);

/**
 * @brief Read servo current position (in degrees)
 *
 * @param id Servo ID
 * @param angle Pointer to store angle in degrees
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_read_angle(uint8_t id, float *angle);

/**
 * @brief Read servo load (torque feedback)
 *
 * @param id Servo ID
 * @param load Pointer to store load value
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_read_load(uint8_t id, uint16_t *load);

/**
 * @brief Read servo temperature
 *
 * @param id Servo ID
 * @param temperature Pointer to store temperature (°C)
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_read_temperature(uint8_t id, uint8_t *temperature);

/**
 * @brief Read servo voltage
 *
 * @param id Servo ID
 * @param voltage Pointer to store voltage (0.1V units)
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_read_voltage(uint8_t id, uint8_t *voltage);

/**
 * @brief Read full servo state
 *
 * @param id Servo ID
 * @param state Pointer to state structure to fill
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_read_state(uint8_t id, struct feetech_servo_state *state);

/**
 * @brief Read states of multiple servos
 *
 * @param ids Array of servo IDs
 * @param states Array of state structures to fill
 * @param count Number of servos (max FEETECH_MAX_SERVOS)
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_read_multi_states(const uint8_t *ids, struct feetech_servo_state *states,
									uint8_t count);

/**
 * @brief Synchronized write - set goal positions for multiple servos
 *
 * Sends goal positions to all servos in a single packet for coordinated motion.
 * This is critical for smooth robot arm movement.
 *
 * @param ids Array of servo IDs
 * @param positions Array of goal positions (ticks)
 * @param count Number of servos (max FEETECH_MAX_SERVOS)
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_sync_write_positions(const uint8_t *ids, const uint16_t *positions,
                                        uint8_t count);

/**
 * @brief Synchronized write - set goal angles for multiple servos (degrees)
 *
 * @param ids Array of servo IDs
 * @param angles Array of goal angles in degrees
 * @param count Number of servos (max FEETECH_MAX_SERVOS)
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_sync_write_angles(const uint8_t *ids, const float *angles,
                                     uint8_t count);

/**
 * @brief Read positions from multiple servos
 *
 * @param ids Array of servo IDs
 * @param positions Array to store positions
 * @param count Number of servos
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_read_multi_positions(const uint8_t *ids, uint16_t *positions,
                                        uint8_t count);

#endif /* OCTROBOT_FEETECH_SERVO_H */
