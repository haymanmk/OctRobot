/*
 * OctroBot Robot Arm Firmware - Servo Control Header
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * High-level servo control API for Feetech SCS/STS servos.
 */

#ifndef OCTROBOT_SERVO_CONTROL_H
#define OCTROBOT_SERVO_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#include "feetech_servo.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enable or disable soft compliance mode for all servos
 * Soft compliance mode allows the servos to be back-driven with reduced resistance,
 * which can be useful for manual positioning or safety. When enabled, the driver will
 * set low P gains and max torque limits on all servos.
 * @note This is a global setting that affects all servos. It is not per-servo.
 * @param enable True to enable soft compliance, false to disable
 * @return 0 on success, negative error code on failure
 */
int servo_control_soft_comply(bool enable);

/**
 * @brief Check if soft compliance mode is currently enabled
 * @return True if soft compliance is enabled, false otherwise
 */
bool servo_control_is_soft_comply_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* OCTROBOT_SERVO_CONTROL_H */