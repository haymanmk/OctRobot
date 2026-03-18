/**
 * OctroBot Robot Arm Firmware - Servo Control Implementation
 * Copyright (c) 2026 OctroBot Project
 * 
 * SPDX-License-Identifier: Apache-2.0
 * 
 * This module implements high-level servo control functions for the Feetech SCS/STS servos used in the OctroBot robot arm.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "servo_control.h"

#define TORQUE_LIMIT_SOFT_COMPLY 100  /* Example torque limit for soft compliance mode (0.1%) */
#define PERIODIC_CHECK_INTERVAL_MS 5 // Interval for periodic compliance checks in soft comply mode
#define STACK_SIZE 2048
#define PRIORITY 5

LOG_MODULE_REGISTER(servo_control, LOG_LEVEL_DBG);

static void soft_comply_thread_fn(void *parameters);

// Register a thread for monitoring and enforcing soft compliance mode
K_KERNEL_THREAD_DEFINE(soft_comply_thread, STACK_SIZE,
                       soft_comply_thread_fn, NULL, NULL, NULL,
                       PRIORITY, 0, 0);

// Register a semaphore for signaling changes in soft compliance mode
K_SEM_DEFINE(soft_comply_sem, 0, 1);

static bool soft_comply_enabled = false;

static void soft_comply_thread_fn(void *parameters)
{
    while (1) {
        // Wait for a signal that soft compliance mode has been enabled
        LOG_DBG("Soft compliance thread waiting for signal...");
        k_sem_take(&soft_comply_sem, K_FOREVER);
        
        while (soft_comply_enabled) {
            for (uint8_t id = 1; id <= FEETECH_MAX_SERVOS; id++) {
                uint16_t pos;
                feetech_servo_read_position(id, &pos);

                // Set goal = current position
                float angle_deg = FEETECH_POS_TO_DEG(pos) * 0.96f; // Apply a small multiplier to allow some movement while still being compliant
                feetech_servo_set_goal_angle(id, angle_deg);
            }
            k_sleep(K_MSEC(PERIODIC_CHECK_INTERVAL_MS)); // Sleep to reduce CPU usage
        }
    }
}

int servo_control_soft_comply(bool enable)
{
    /* 
     * In a real implementation, we would send specific commands to each servo to adjust their compliance settings.
     * For example, we might set low P gains and max torque limits when enabling soft comply mode.
     * This is a placeholder to indicate where that logic would go.
     */
    
    LOG_INF("Soft compliance mode %s", enable ? "ENABLED" : "DISABLED");

    if (soft_comply_enabled == enable) return 0; // No change needed
    
    soft_comply_enabled = enable;

    if (enable) {
        /* Enable soft compliance - set low P gains and max torque limits on all servos */
        for (uint8_t id = 1; id <= FEETECH_MAX_SERVOS; id++) {
            // Example: feetech_servo_set_compliance(id, true);
            LOG_DBG("Setting servo %d to soft compliance mode", id);
            // Set torque limit to a low value for soft compliance
            int ret = feetech_servo_set_torque_limit(id, TORQUE_LIMIT_SOFT_COMPLY);
            if (ret != 0) {
                LOG_ERR("Failed to set torque limit for servo %d: %d", id, ret);
            }
        }
        // Signal the compliance thread to start monitoring this servo
        k_sem_give(&soft_comply_sem);
    } else {
        /* Disable soft compliance - restore normal P gains and torque limits */
        for (uint8_t id = 1; id <= FEETECH_MAX_SERVOS; id++) {
            // Example: feetech_servo_set_compliance(id, false);
            LOG_DBG("Restoring servo %d to normal compliance mode", id);
            // Restore torque limit to default
            int ret = feetech_servo_set_torque_limit(id, 1000); // 100% torque
            if (ret != 0) {
                LOG_ERR("Failed to restore torque limit for servo %d: %d", id, ret);
            }
        }
    }
    
    return 0;
}

bool servo_control_is_soft_comply_enabled(void)
{
    return soft_comply_enabled;
}