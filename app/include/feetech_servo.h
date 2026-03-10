/*
 * OctroBot Robot Arm Firmware - Feetech Servo Driver
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * High-level servo control API with unit conversion (radians, rad/s).
 */

#ifndef OCTROBOT_FEETECH_SERVO_H
#define OCTROBOT_FEETECH_SERVO_H

#include "hal_types.h"
#include <stdint.h>
#include <stdbool.h>

/* Servo model specifications */
#define FEETECH_STS3215_POSITION_MIN    0
#define FEETECH_STS3215_POSITION_MAX    4095
#define FEETECH_STS3215_ANGLE_RANGE     (2.0 * 3.14159265359)  /* ~360 degrees in radians */
#define FEETECH_STS3215_SPEED_MAX       4095
#define FEETECH_STS3215_SPEED_RPM_MAX   62.5   /* RPM at max speed value */

/* Default servo configuration */
#define FEETECH_DEFAULT_TORQUE_ENABLE   1
#define FEETECH_DEFAULT_MAX_TORQUE      1000
#define FEETECH_DEFAULT_GOAL_TIME_MS    0      /* 0 = max speed */

/**
 * @brief Servo instance structure
 */
struct feetech_servo {
	uint8_t id;                     /* Servo ID on bus */
	float position_offset;          /* Calibration offset (radians) */
	float position_min;             /* Minimum position limit (radians) */
	float position_max;             /* Maximum position limit (radians) */
	uint16_t position_ticks_min;    /* Minimum position in ticks */
	uint16_t position_ticks_max;    /* Maximum position in ticks */
	bool reverse_direction;         /* Reverse sign for position */
};

/**
 * @brief Servo status information
 */
struct feetech_servo_status {
	float position_rad;             /* Current position (radians) */
	float speed_rad_s;              /* Current speed (rad/s) */
	int16_t load;                   /* Current load (-1000 to 1000) */
	uint8_t voltage;                /* Voltage (0.1V units, e.g., 120 = 12.0V) */
	uint8_t temperature;            /* Temperature (°C) */
	bool moving;                    /* True if servo is moving */
	uint8_t error_flags;            /* Error flags from servo */
};

/**
 * @brief Initialize servo instance
 *
 * @param servo Pointer to servo structure
 * @param id Servo ID (1-253)
 * @param position_min Minimum position limit (radians)
 * @param position_max Maximum position limit (radians)
 * @param reverse_direction True to reverse direction
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_init(struct feetech_servo *servo, uint8_t id,
                       float position_min, float position_max,
                       bool reverse_direction);

/**
 * @brief Ping servo to check if it's responding
 *
 * @param servo Servo instance
 * @return HAL_OK if servo responds, error code otherwise
 */
int feetech_servo_ping(const struct feetech_servo *servo);

/**
 * @brief Enable/disable servo torque
 *
 * @param servo Servo instance
 * @param enable True to enable, false to disable
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_set_torque_enable(const struct feetech_servo *servo, bool enable);

/**
 * @brief Set goal position
 *
 * @param servo Servo instance
 * @param position_rad Goal position in radians
 * @param time_ms Time to reach position (0 = max speed)
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_set_goal_position(const struct feetech_servo *servo,
                                    float position_rad, uint16_t time_ms);

/**
 * @brief Set goal speed
 *
 * @param servo Servo instance
 * @param speed_rad_s Goal speed in rad/s (0 = stop)
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_set_goal_speed(const struct feetech_servo *servo, float speed_rad_s);

/**
 * @brief Read present position
 *
 * @param servo Servo instance
 * @param position_rad Output position in radians
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_read_position(const struct feetech_servo *servo, float *position_rad);

/**
 * @brief Read present speed
 *
 * @param servo Servo instance
 * @param speed_rad_s Output speed in rad/s
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_read_speed(const struct feetech_servo *servo, float *speed_rad_s);

/**
 * @brief Read present load
 *
 * @param servo Servo instance
 * @param load Output load (-1000 to 1000, negative = CCW, positive = CW)
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_read_load(const struct feetech_servo *servo, int16_t *load);

/**
 * @brief Read present temperature
 *
 * @param servo Servo instance
 * @param temperature Output temperature in °C
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_read_temperature(const struct feetech_servo *servo, uint8_t *temperature);

/**
 * @brief Read present voltage
 *
 * @param servo Servo instance
 * @param voltage Output voltage in 0.1V units (e.g., 120 = 12.0V)
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_read_voltage(const struct feetech_servo *servo, uint8_t *voltage);

/**
 * @brief Read full status of servo
 *
 * @param servo Servo instance
 * @param status Output status structure
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_read_status(const struct feetech_servo *servo,
                              struct feetech_servo_status *status);

/**
 * @brief Check if servo is moving
 *
 * @param servo Servo instance
 * @param moving Output: true if moving
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_is_moving(const struct feetech_servo *servo, bool *moving);

/**
 * @brief Set position offset for calibration
 *
 * @param servo Servo instance
 * @param offset_rad Offset in radians (added to all position commands/reads)
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_set_offset(struct feetech_servo *servo, float offset_rad);

/**
 * @brief Sync write goal positions to multiple servos
 *
 * Sends position commands to all servos in a single transaction for
 * coordinated motion (critical for robot arm).
 *
 * @param servos Array of servo instances
 * @param positions Array of goal positions in radians
 * @param times Array of goal times in ms (0 = max speed)
 * @param count Number of servos
 * @return HAL_OK on success, error code otherwise
 */
int feetech_servo_sync_set_goal_position(const struct feetech_servo *servos,
                                         const float *positions,
                                         const uint16_t *times,
                                         size_t count);

/**
 * @brief Convert position from radians to servo ticks
 *
 * @param servo Servo instance
 * @param position_rad Position in radians
 * @return Position in servo ticks (0-4095)
 */
uint16_t feetech_servo_rad_to_ticks(const struct feetech_servo *servo, float position_rad);

/**
 * @brief Convert position from servo ticks to radians
 *
 * @param servo Servo instance
 * @param ticks Position in servo ticks (0-4095)
 * @return Position in radians
 */
float feetech_servo_ticks_to_rad(const struct feetech_servo *servo, uint16_t ticks);

#endif /* OCTROBOT_FEETECH_SERVO_H */
