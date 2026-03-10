/*
 * OctroBot Robot Arm Firmware - Timer Utilities
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Provides microsecond-resolution timing for control loops.
 */

#ifndef OCTROBOT_HAL_TIMER_H
#define OCTROBOT_HAL_TIMER_H

#include "hal_types.h"

/**
 * @brief Get current time in microseconds
 *
 * Uses Zephyr k_cycle_get_32() and converts to microseconds.
 * Wraps around approximately every 71 minutes on 240MHz ESP32.
 *
 * @return Current time in microseconds
 */
uint64_t hal_timer_get_us(void);

/**
 * @brief Get current time in milliseconds
 *
 * @return Current time in milliseconds
 */
uint64_t hal_timer_get_ms(void);

/**
 * @brief Delay for specified microseconds (busy-wait)
 *
 * Use sparingly - prefer k_usleep() for longer delays to allow
 * other threads to run.
 *
 * @param us Microseconds to delay
 */
void hal_timer_delay_us(uint32_t us);

/**
 * @brief Delay for specified milliseconds
 *
 * Uses k_msleep() to yield to other threads.
 *
 * @param ms Milliseconds to delay
 */
void hal_timer_delay_ms(uint32_t ms);

/**
 * @brief Initialize timer subsystem
 *
 * @return HAL_OK on success, error code otherwise
 */
int hal_timer_init(void);

#endif /* OCTROBOT_HAL_TIMER_H */
