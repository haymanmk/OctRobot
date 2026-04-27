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
#include "hal_flash.h"
#include "half_duplex_uart.h"
#include "uart_console.h"
#include "robot_geometry.h"
#include "forward_kinematics_poe.h"

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

	/* Initialize flash storage for persistent data */
	ret = hal_flash_init();
	if (ret != HAL_OK) {
		LOG_ERR("Failed to initialize flash storage: %d", ret);
		/* Non-fatal - continue without persistent storage */
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

static void test_forward_kinematics(void)
{
	LOG_INF("");
	LOG_INF("===========================================");
	LOG_INF("Testing Forward Kinematics (POE)");
	LOG_INF("===========================================");
	
	/* Initialize robot geometry (loads from flash or uses factory defaults) */
	robot_geometry_init();
	const poe_robot_model_t *model = robot_geometry_get_model();
	
	LOG_INF("Robot model loaded:");
	LOG_INF("  - Screw axes: 6 joints");
	LOG_INF("  - Home config: M matrix");
	LOG_INF("  - Joint limits configured");
	
	/* Test 1: Zero configuration (home position) */
	LOG_INF("");
	LOG_INF("Test 1: Zero configuration (home position)");
	float joint_angles_zero[NUM_JOINTS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	mat4x4_t T_zero;
	
	if (forward_kinematics_compute(model, joint_angles_zero, &T_zero)) {
		vec3_t pos = mat4x4_get_translation(&T_zero);
		LOG_INF("  FK([0,0,0,0,0,0]):");
		LOG_INF("    Position: [%.4f, %.4f, %.4f] m", 
		        (double)pos.x, (double)pos.y, (double)pos.z);
		LOG_INF("    (Should match home configuration M)");
	} else {
		LOG_ERR("  FK computation failed!");
	}
	
	/* Test 2: Small joint movements */
	LOG_INF("");
	LOG_INF("Test 2: Small joint movements");
	float joint_angles_30deg[NUM_JOINTS] = {
		deg_to_rad(30.0f),   /* Base rotation 30° */
		deg_to_rad(0.0f),    /* Shoulder */
		deg_to_rad(0.0f),    /* Elbow */
		deg_to_rad(0.0f),    /* Wrist roll */
		deg_to_rad(0.0f),    /* Wrist pitch */
		deg_to_rad(0.0f)     /* Wrist roll */
	};
	mat4x4_t T_30deg;
	
	if (forward_kinematics_compute(model, joint_angles_30deg, &T_30deg)) {
		vec3_t pos = mat4x4_get_translation(&T_30deg);
		LOG_INF("  FK([30°,0,0,0,0,0]):");
		LOG_INF("    Position: [%.4f, %.4f, %.4f] m", 
		        (double)pos.x, (double)pos.y, (double)pos.z);
		LOG_INF("    (Base rotated 30°, end-effector should move in XY plane)");
	} else {
		LOG_ERR("  FK computation failed!");
	}
	
	/* Test 3: Multiple joint movements */
	LOG_INF("");
	LOG_INF("Test 3: Multiple joint movements");
	float joint_angles_multi[NUM_JOINTS] = {
		deg_to_rad(45.0f),   /* Base 45° */
		deg_to_rad(30.0f),   /* Shoulder 30° */
		deg_to_rad(-20.0f),  /* Elbow -20° */
		deg_to_rad(0.0f),    /* Wrist roll */
		deg_to_rad(15.0f),   /* Wrist pitch 15° */
		deg_to_rad(0.0f)     /* Wrist roll */
	};
	mat4x4_t T_multi;
	
	if (forward_kinematics_compute(model, joint_angles_multi, &T_multi)) {
		vec3_t pos = mat4x4_get_translation(&T_multi);
		mat3x3_t rot = mat4x4_get_rotation(&T_multi);
		LOG_INF("  FK([45°,30°,-20°,0°,15°,0°]):");
		LOG_INF("    Position: [%.4f, %.4f, %.4f] m", 
		        (double)pos.x, (double)pos.y, (double)pos.z);
		LOG_INF("    Rotation matrix:");
		LOG_INF("      [%.3f, %.3f, %.3f]", 
		        (double)rot.m[0][0], (double)rot.m[0][1], (double)rot.m[0][2]);
		LOG_INF("      [%.3f, %.3f, %.3f]", 
		        (double)rot.m[1][0], (double)rot.m[1][1], (double)rot.m[1][2]);
		LOG_INF("      [%.3f, %.3f, %.3f]", 
		        (double)rot.m[2][0], (double)rot.m[2][1], (double)rot.m[2][2]);
	} else {
		LOG_ERR("  FK computation failed!");
	}
	
	/* Test 4: Partial FK (intermediate link positions) */
	LOG_INF("");
	LOG_INF("Test 4: Partial FK (joint 2 position)");
	mat4x4_t T_partial;
	
	if (forward_kinematics_partial(model, joint_angles_multi, 2, &T_partial)) {
		vec3_t pos = mat4x4_get_translation(&T_partial);
		LOG_INF("  FK_partial([45°,30°,-20°,...], end_idx=2):");
		LOG_INF("    Joint 2 position: [%.4f, %.4f, %.4f] m", 
		        (double)pos.x, (double)pos.y, (double)pos.z);
		LOG_INF("    (Position after joints 0-2)");
	} else {
		LOG_ERR("  Partial FK computation failed!");
	}
	
	/* Test 5: Performance measurement */
	LOG_INF("");
	LOG_INF("Test 5: Performance measurement");
	uint32_t start_time = k_cycle_get_32();
	const int num_iterations = 100;
	
	for (int i = 0; i < num_iterations; i++) {
		mat4x4_t T_perf;
		forward_kinematics_compute(model, joint_angles_multi, &T_perf);
	}
	
	uint32_t end_time = k_cycle_get_32();
	uint32_t cycles = end_time - start_time;
	uint32_t cycles_per_fk = cycles / num_iterations;
	
	/* ESP32 @ 240MHz: 1 cycle = 1/240MHz ≈ 4.17 ns */
	float time_per_fk_us = (float)cycles_per_fk / 240.0f;  /* μs */
	
	LOG_INF("  FK computation time:");
	LOG_INF("    %d iterations: %u cycles", num_iterations, cycles);
	LOG_INF("    Average: %u cycles/FK (%.2f μs)", 
	        cycles_per_fk, (double)time_per_fk_us);
	LOG_INF("    Expected: 2000-5000 μs (2-5 ms)");
	
	if (time_per_fk_us < 10000.0f) {  /* Less than 10ms */
		LOG_INF("    ✓ Performance is good!");
	} else {
		LOG_WRN("    ⚠ Performance is slower than expected");
	}
	
	LOG_INF("");
	LOG_INF("===========================================");
	LOG_INF("Forward Kinematics test complete");
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
	
	/* Test forward kinematics (Phase 4) */
	k_sleep(K_MSEC(500));
	test_forward_kinematics();

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
