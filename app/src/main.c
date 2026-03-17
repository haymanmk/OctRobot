/*
 * OctroBot Robot Arm Firmware
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/version.h>

#include "feetech_servo.h"
#include "hal_gpio.h"
#include "half_duplex_uart.h"
#include "uart_console.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static volatile bool emergency_stop_active = false;

static void emergency_stop_callback(void)
{
	emergency_stop_active = true;
	LOG_ERR("!!! EMERGENCY STOP ACTIVATED !!!");

	/* Disable torque on all configured joints. */
	for (uint8_t id = 1; id <= 6; id++) {
		(void)feetech_servo_set_torque_enable(id, false);
	}
}

static int initialize_hal(void)
{
	int ret;

	LOG_INF("Initializing HAL layer...");

	ret = hal_gpio_button_init();
	if (ret == HAL_OK) {
		ret = hal_gpio_button_set_callback(emergency_stop_callback);
		if (ret != HAL_OK) {
			LOG_WRN("Button callback setup failed: %d", ret);
		}
	} else {
		LOG_WRN("Button init failed: %d", ret);
	}

	struct half_duplex_uart_config uart_config = {
		.device_name = NULL,
		.baudrate = 1000000,
		.tx_timeout_ms = 100,
		.rx_timeout_ms = 50,
	};

	hal_uart_handle_t servo_uart = half_duplex_uart_init(&uart_config);
	if (!servo_uart) {
		LOG_ERR("Failed to initialize half-duplex UART");
		return HAL_ERROR;
	}

	ret = feetech_servo_init(servo_uart);
	if (ret != HAL_OK) {
		LOG_ERR("Failed to initialize servo driver: %d", ret);
		return ret;
	}

	LOG_INF("HAL layer initialized successfully");
	return HAL_OK;
}

static void test_servos(void)
{
	LOG_INF("===========================================");
	LOG_INF("Testing Servo Communication");
	LOG_INF("===========================================");

	uint8_t servo_ids[] = {1, 2, 3, 4, 5, 6};
	uint8_t servo_count = ARRAY_SIZE(servo_ids);
	uint8_t detected_servos = 0;

	LOG_INF("Pinging servos...");
	for (uint8_t i = 0; i < servo_count; i++) {
		int ret = feetech_servo_ping(servo_ids[i]);
		if (ret == HAL_OK) {
			LOG_INF("  Servo %d: OK", servo_ids[i]);
			detected_servos++;
		} else {
			LOG_WRN("  Servo %d: No response (%d)", servo_ids[i], ret);
		}
		k_msleep(50);
	}

	LOG_INF("Detected %d/%d servos", detected_servos, servo_count);
	if (detected_servos == 0) {
		LOG_ERR("No servos detected. Check wiring and power.");
		LOG_INF("  - Verify servos are powered (6-12V)");
		LOG_INF("  - Check UART connections (GPIO 21=TX, GPIO 25=RX)");
		LOG_INF("  - Confirm servo IDs are 1-6");
		LOG_INF("  - Verify baud rate is 1 Mbps");
		return;
	}

	LOG_INF("Reading initial servo positions...");
	for (uint8_t i = 0; i < servo_count; i++) {
		uint16_t position = 0;
		int ret = feetech_servo_read_position(servo_ids[i], &position);
		if (ret == HAL_OK) {
			float angle = FEETECH_POS_TO_DEG(position);
			LOG_INF("  Servo %d: position=%d (%.2f deg)", servo_ids[i], position,
				(double)angle);
		}
		k_msleep(50);
	}

	LOG_INF("Reading servo status...");
	for (uint8_t i = 0; i < servo_count; i++) {
		struct feetech_servo_state state;
		int ret = feetech_servo_read_state(servo_ids[i], &state);
		if (ret == HAL_OK) {
			LOG_INF("  Servo %d: temp=%dC, voltage=%d.%dV, load=%d", servo_ids[i],
				state.present_temperature, state.present_voltage / 10,
				state.present_voltage % 10, state.present_load);
		}
		k_msleep(100);
	}

	LOG_INF("===========================================");
	LOG_INF("Servo test complete");
	LOG_INF("===========================================");
}

int main(void)
{
	int ret;

	LOG_INF("===========================================");
	LOG_INF("OctroBot Robot Arm Firmware v0.3.0");
	LOG_INF("===========================================");
	LOG_INF("MCU: ESP32-PICO-D4 (M5Stack Atom Lite)");
	LOG_INF("RTOS: Zephyr RTOS v%s", KERNEL_VERSION_STRING);
	LOG_INF("Build: %s %s", __DATE__, __TIME__);
	LOG_INF("Console: UART0 via CH340 USB-UART bridge");
	LOG_INF("===========================================");

	ret = initialize_hal();
	if (ret != HAL_OK) {
		LOG_ERR("HAL initialization failed: %d", ret);
		return ret;
	}

	LOG_INF("System initialization complete");
	LOG_INF("6-DOF robot arm ready");
	LOG_INF("Press button (GPIO 39) for emergency stop");

	k_sleep(K_MSEC(500));
	test_servos();

	/* Initialize host communication */
	ret = uart_console_init();
	if (ret == 0) {
		LOG_INF("Host communication initialized");
		LOG_INF("Connect via USB serial to send commands");
	} else {
		LOG_WRN("Host communication init failed: %d", ret);
		LOG_INF("Continuing without host control...");
	}

	LOG_INF("");
	LOG_INF("===========================================");
	LOG_INF("Entering main loop - Phase 3b Manual Control");
	LOG_INF("===========================================");
	LOG_INF("Send commands via USB serial to control robot");
	LOG_INF("Available commands:");
	LOG_INF("  - jog: Move joints incrementally");
	LOG_INF("  - set: Direct position control");
	LOG_INF("  - read: Get current joint angles");
	LOG_INF("  - demo: Record and playback motions");
	LOG_INF("===========================================");
	LOG_INF("");

	while (1) {
		// Heartbeat message every 5 seconds
		LOG_DBG("Main loop heartbeat");
		k_sleep(K_MSEC(5000));
	}

	return 0;
}
