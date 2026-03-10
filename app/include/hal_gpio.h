/*
 * OctroBot Robot Arm Firmware - GPIO Utilities
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef OCTROBOT_HAL_GPIO_H
#define OCTROBOT_HAL_GPIO_H

#include "hal_types.h"

/**
 * @brief Initialize button GPIO (emergency stop)
 *
 * @return HAL_OK on success, error code otherwise
 */
int hal_gpio_button_init(void);

/**
 * @brief Read button state
 *
 * @return true if button is pressed, false otherwise
 */
bool hal_gpio_button_is_pressed(void);

/**
 * @brief Set button callback for emergency stop
 *
 * @param callback Function to call when button is pressed
 * @return HAL_OK on success, error code otherwise
 */
int hal_gpio_button_set_callback(void (*callback)(void));

/**
 * @brief Initialize status LED (WS2812B)
 *
 * @return HAL_OK on success, error code otherwise
 */
int hal_gpio_led_init(void);

/**
 * @brief Set LED state (simple on/off, WS2812B requires special driver for colors)
 *
 * @param on true to turn on, false to turn off
 * @return HAL_OK on success, error code otherwise
 */
int hal_gpio_led_set(bool on);

#endif /* OCTROBOT_HAL_GPIO_H */
