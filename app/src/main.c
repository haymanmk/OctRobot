/*
 * OctroBot Robot Arm Firmware
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/logging/log.h>

#include "half_duplex_uart.h"
#include "hal_gpio.h"
#include "hal_timer.h"
#include "feetech_servo.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Emergency stop flag */
static volatile bool emergency_stop_active = false;

/* Emergency stop callback */
static void emergency_stop_callback(void)
{
	emergency_stop_active = true;
	LOG_ERR("!!! EMERGENCY STOP ACTIVATED !!!");
	
	/* TODO Phase 6: Halt all motion, send stop commands to servos */
	/* For now, disable torque on all servos */
	for (uint8_t id = 1; id <= 6; id++) {
		feetech_servo_set_torque_enable(id, false);
	}
}

/* HAL initialization */
static int initialize_hal(void)
{
	int ret;
	
	LOG_INF("Initializing HAL layer...");
	
	/* Initialize timer subsystem */
	ret = hal_timer_init();
	if (ret != HAL_OK) {
		LOG_ERR("Failed to initialize timer");
		return ret;
	}
	
	/* Initialize GPIO (LED and button) */
	ret = hal_gpio_led_init();
	if (ret == HAL_OK) {
		/* Blink LED to indicate startup */
		for (int i = 0; i < 3; i++) {
			hal_gpio_led_set(true);
			hal_timer_delay_ms(100);
			hal_gpio_led_set(false);
			hal_timer_delay_ms(100);
		}
	}
	
	ret = hal_gpio_button_init();
	if (ret == HAL_OK) {
		hal_gpio_button_set_callback(emergency_stop_callback);
	}
	
	/* Initialize half-duplex UART for servo bus */
	struct half_duplex_uart_config uart_config = {
		.device_name = "UART_1",
		.baudrate = 1000000,  /* 1 Mbps for Feetech STS servos */
		.tx_timeout_ms = 100,
		.rx_timeout_ms = 50,
	};
	
	hal_uart_handle_t servo_uart = half_duplex_uart_init(&uart_config);
	if (!servo_uart) {
	/* Initialize servo driver */
	ret = feetech_servo_init(servo_uart);
	if (ret != HAL_OK) {
		LOG_ERR("Failed to initialize servo driver");
		return ret;
	}
	
	LOG_INF("HAL layer initialized successfully");
	return HAL_OK;
}

/* Test servo communication */
static void test_servos(void)
{
	LOG_INF("===============================3===========");
	LOG_INF("Testing Servo Communication");
	LOG_INF("===========================================");
	
	/* Expected servo IDs for 6-DOF arm */
	uint8_t servo_ids[] = {1, 2, 3, 4, 5, 6};
	uint8_t servo_count = 6;
	
	/* Ping each servo */
	LOG_INF("Pinging servos...");
	uint8_t detected_servos = 0;
	for (uint8_t i = 0; i < servo_count; i++) {
		int ret = feetech_servo_ping(servo_ids[i]);
		if (ret == HAL_OK) {
			LOG_INF("  Servo %d: OK", servo_ids[i]);
			detected_servos++;
		} else {
			LOG_WRN("  Servo %d: No response", servo_ids[i]);
		}
		k_msleep(50); /* Small delay between pings */
	}
	
	LOG_INF("Detected %d/%d servos", detected_servos, servo_count);
	
	if (detected_servos == 0) {
		LOG_ERR("No servos detected! Check wiring and power.");
		LOG_INF("  - Verify servos are powered (6-12V)");
		LOG_INF("  - Check UART connections (GPIO 26=TX, GPIO 32=RX)");
		LOG_INF("  - Confirm servo IDs are 1-6");
		LOG_INF("  - Verify baud rate is 1 Mbps");
		return;
	}
	
	/* Read initial positions */
	LOG_INF("Reading initial servo positions...");
	for (uint8_t i = 0; i < servo_count; i++) {
		uint16_t position;
		int ret = feetech_servo_read_position(servo_ids[i], &position);
		if (ret == HAL_OK) {
			float angle = FEETECH_POS_TO_RAD(position);
			LOG_INF("  Servo %d: position=%d (%.2f rad)", 
			        servo_ids[i], position, (double)angle);
		}
		k_msleep(50);
	}
	
	/* Read servo status */
	LOG_INF("Reading servo status...");
	for (uint8_t i = 0; i < servo_count; i++) {
		struct feetech_servo_state state;
		int ret = feetech_servo_read_state(servo_ids[i], &state);
		if (ret == HAL_OK) {
			LOG_INF("  Servo %d: temp=%d°C, voltage=%d.%dV, load=%d",
			        servo_ids[i],
			        state.present_temperature,
			        state.present_voltage / 10,
			        state.present_voltage % 10,
			        state.present_load);
		}
		k_msleep(100);
	}
	
	LOG_INF("===========================================");
	LOG_INF("Servo test complete");
	LOG_INF("===========================================")RROR;
	}
	
	LOG_INF("HAL layer initialized successfully");
	return HAL_OK;
}

/* Main entry point */
int main(void)
{
	int ret;

	LOG_INF("===========================================");
	LOG_INF("OctroBot Robot Arm Firmware v0.2.0");
	LOG_INF("===========================================");
	LOG_INF("MCU: ESP32-PICO-D4 (M5Stack Atom Lite)");
	LOG_INF("RTOS: Zephyr RTOS v%s", KERNEL_VERSION_STRING);
	LOG_INF("Build: %s %s", __DATE__, __TIME__);
	LOG_INF("===========================================");

#if defined(CONFIG_USB_DEVICE_STACK)
	/* Enable USB CDC ACM for console output */
	const struct device *usb_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	
	if (!device_is_ready(usb_dev)) {
		LOG_ERR("USB CDC ACM device not ready");
		return -1;
	}

	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB: %d", ret);
		return ret;
	}

	LOG_INF("USB CDC ACM console enabled");
	
	/* Give USB time to enumerate */
	k_sleep(K_MSEC(1000));
#endif

	/* Initialize HAL layer */
	ret = initialize_hal();
	if (ret != HAL_OK) {
		LOG_ERR("HAL initialization failed: %d", ret);
		return ret;
	}

	LOG_INF("System initialization complete");
	LOG_INF("6-DOF robot arm ready");
	LOG_INF("Press button (GPIO 39) for emergency stop");
	
	/* Test servo communication */
	k_sleep(K_MSEC(500)); /* Brief delay before testing */
	test_servos();

	/* Main loop */
	uint32_t counter = 0;
	uint64_t last_time_us = hal_timer_get_us();
	
	while (1) {
		/* Heartbeat with timing measurement */
		uint64_t current_time_us = hal_timer_get_us();
		uint64_t elapsed_us = current_time_us - last_time_us;
		last_time_us = current_time_us;
		
		LOG_INF("Heartbeat: %u sec (loop time: %llu us)", 
		        counter, elapsed_us);
		
		/* Blink LED as status indicator */
		hal_gpio_led_set(counter % 2);
		
		/* Check emergency stop status */
		if (emergency_stop_active) {
			LOG_WRN("System halted - emergency stop active");
			/* Keep running but indicate stopped state */
		}
		
		counter++;
		k_sleep(K_SECONDS(2));
	}

	return 0;
}
