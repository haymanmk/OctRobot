/*
 * OctroBot Robot Arm Firmware - GPIO Utilities Implementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "hal_gpio.h"

LOG_MODULE_REGISTER(hal_gpio, LOG_LEVEL_INF);

/* Device tree nodes */
#define BUTTON_NODE DT_ALIAS(sw0)
#define LED_NODE    DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(BUTTON_NODE, okay)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static struct gpio_callback button_cb_data;
static void (*button_callback)(void) = NULL;
#endif

#if DT_NODE_HAS_STATUS(LED_NODE, okay)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
#endif

#if DT_NODE_HAS_STATUS(BUTTON_NODE, okay)
/* Button ISR handler */
static void button_pressed_isr(const struct device *dev, struct gpio_callback *cb,
                                uint32_t pins)
{
	LOG_WRN("Emergency stop button pressed!");
	
	if (button_callback) {
		button_callback();
	}
}
#endif

int hal_gpio_button_init(void)
{
#if DT_NODE_HAS_STATUS(BUTTON_NODE, okay)
	if (!gpio_is_ready_dt(&button)) {
		LOG_ERR("Button GPIO device not ready");
		return HAL_ERROR;
	}
	
	int ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Failed to configure button GPIO: %d", ret);
		return HAL_ERROR;
	}
	
	/* Configure interrupt for button press */
	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Failed to configure button interrupt: %d", ret);
		return HAL_ERROR;
	}
	
	gpio_init_callback(&button_cb_data, button_pressed_isr, BIT(button.pin));
	ret = gpio_add_callback(button.port, &button_cb_data);
	if (ret != 0) {
		LOG_ERR("Failed to add button callback: %d", ret);
		return HAL_ERROR;
	}
	
	LOG_INF("Button GPIO initialized (GPIO %d)", button.pin);
	
	return HAL_OK;
#else
	LOG_WRN("Button not defined in device tree");
	return HAL_ERROR;
#endif
}

bool hal_gpio_button_is_pressed(void)
{
#if DT_NODE_HAS_STATUS(BUTTON_NODE, okay)
	return gpio_pin_get_dt(&button) != 0;
#else
	return false;
#endif
}

int hal_gpio_button_set_callback(void (*callback)(void))
{
#if DT_NODE_HAS_STATUS(BUTTON_NODE, okay)
	button_callback = callback;
	LOG_INF("Button callback registered");
	return HAL_OK;
#else
	return HAL_ERROR;
#endif
}

int hal_gpio_led_init(void)
{
#if DT_NODE_HAS_STATUS(LED_NODE, okay)
	if (!gpio_is_ready_dt(&led)) {
		LOG_ERR("LED GPIO device not ready");
		return HAL_ERROR;
	}
	
	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("Failed to configure LED GPIO: %d", ret);
		return HAL_ERROR;
	}
	
	LOG_INF("LED GPIO initialized (GPIO %d)", led.pin);
	
	return HAL_OK;
#else
	LOG_WRN("LED not defined in device tree");
	return HAL_ERROR;
#endif
}

int hal_gpio_led_set(bool on)
{
#if DT_NODE_HAS_STATUS(LED_NODE, okay)
	return gpio_pin_set_dt(&led, on ? 1 : 0);
#else
	return HAL_ERROR;
#endif
}
