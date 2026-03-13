/*
 * OctroBot Robot Arm Firmware - Host Communication Implementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>

/* Define M_PI if not available */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "host_comms.h"
#include "packet_protocol.h"
#include "feetech_servo.h"

LOG_MODULE_REGISTER(host_comms, LOG_LEVEL_INF);

/* UART device (console via CH340 USB-UART bridge) */
static const struct device *uart_dev = NULL;

/* Manual control mode flag - when true, suppress system logging */
static bool manual_mode_active = false;

/* Receive buffer for packet assembly */
#define RX_BUF_SIZE 256
static uint8_t rx_buf[RX_BUF_SIZE];
static uint16_t rx_buf_pos = 0;

/* Read loop state */
static bool read_loop_active = false;

/* Demo recording state */
#define MAX_DEMO_WAYPOINTS 50
struct demo_waypoint {
	float joint_angles[6];
	uint32_t delay_ms;
};

static struct {
	bool is_recording;
	uint8_t demo_id;
	uint8_t waypoint_count;
	struct demo_waypoint waypoints[MAX_DEMO_WAYPOINTS];
} demo_recording;

/* Forward declarations of command handlers */
static int handle_jog_joint(const uint8_t *payload, uint8_t len);
static int handle_set_joint_direct(const uint8_t *payload, uint8_t len);
static int handle_read_state(void);
static int handle_start_read_loop(void);
static int handle_stop_read_loop(void);
static int handle_single_joint_test(const uint8_t *payload, uint8_t len);
static int handle_stop(void);
static int handle_start_demo_recording(const uint8_t *payload, uint8_t len);
static int handle_add_waypoint(const uint8_t *payload, uint8_t len);
static int handle_finish_demo_recording(void);
static int handle_play_demo(const uint8_t *payload, uint8_t len);
static int handle_clear_demo(const uint8_t *payload, uint8_t len);

int host_comms_init(void)
{
	LOG_INF("Initializing UART host communication...");
	
	/* Get console UART device (UART0 via CH340 USB-UART bridge) */
	uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("Console UART device not ready");
		return -ENODEV;
	}
	
	LOG_INF("UART host communication initialized successfully");
	LOG_INF("Host can now send commands via UART (USB-to-UART bridge)");
	
	/* Initialize demo recording state */
	demo_recording.is_recording = false;
	demo_recording.demo_id = 0;
	demo_recording.waypoint_count = 0;
	
	return 0;
}

void host_comms_set_manual_mode(bool enable)
{
	if (enable && !manual_mode_active) {
		/* Transitioning to manual mode - print final message before suppressing logs */
		LOG_INF("===========================================");
		LOG_INF("Manual Control Mode ENABLED");
		LOG_INF("System logging suppressed to prevent UART interference");
		LOG_INF("Send commands via host_test.py");
		LOG_INF("===========================================");
		k_sleep(K_MSEC(100)); /* Allow log to flush */
	} else if (!enable && manual_mode_active) {
		LOG_INF("===========================================");
		LOG_INF("Manual Control Mode DISABLED");
		LOG_INF("System logging resumed");
		LOG_INF("===========================================");
	}
	manual_mode_active = enable;
}

bool host_comms_is_manual_mode(void)
{
	return manual_mode_active;
}

int host_comms_get_robot_state(struct robot_state *state)
{
	if (!state) {
		return -EINVAL;
	}
	
	/* Query all 6 servos */
	uint8_t servo_ids[] = {1, 2, 3, 4, 5, 6};
	bool any_moving = false;
	
	for (uint8_t i = 0; i < 6; i++) {
		/* Read position */
		uint16_t position = 0;
		int ret = feetech_servo_read_position(servo_ids[i], &position);
		if (ret == 0) {
			state->joint_angles[i] = FEETECH_POS_TO_DEG(position);
		} else {
			state->joint_angles[i] = 0.0f;
		}
		
		/* Read full state */
		struct feetech_servo_state servo_state;
		ret = feetech_servo_read_state(servo_ids[i], &servo_state);
		if (ret == 0) {
			state->temperatures[i] = servo_state.present_temperature;
			state->voltages[i] = servo_state.present_voltage;
			state->loads[i] = servo_state.present_load;
			if (servo_state.is_moving) {
				any_moving = true;
			}
		} else {
			state->temperatures[i] = 0;
			state->voltages[i] = 0;
			state->loads[i] = 0;
		}
	}
	
	state->is_moving = any_moving;
	state->error_flags = 0; /* TODO: Implement error tracking */
	
	return 0;
}

int host_comms_send_status(const struct robot_state *state)
{
	if (!uart_dev || !state) {
		return -EINVAL;
	}
	
	struct packet pkt;
	packet_init(&pkt, CMD_STATUS_REPORT);
	
	/* Pack joint angles (6 × float32 = 24 bytes) */
	packet_add_payload(&pkt, (const uint8_t *)state->joint_angles, 24);
	
	/* Pack temperatures (6 bytes) */
	packet_add_payload(&pkt, state->temperatures, 6);
	
	/* Pack voltages (6 bytes) */
	packet_add_payload(&pkt, state->voltages, 6);
	
	/* Pack loads (6 × uint16_t = 12 bytes) */
	packet_add_payload(&pkt, (const uint8_t *)state->loads, 12);
	
	/* Pack flags */
	uint8_t flags = (state->is_moving ? 0x01 : 0x00) | (state->error_flags << 1);
	packet_add_payload(&pkt, &flags, 1);
	
	/* Finalize and send */
	packet_finalize(&pkt);
	
	uint8_t tx_buf[PACKET_MAX_SIZE];
	uint8_t tx_len;
	if (packet_serialize(&pkt, tx_buf, &tx_len) != 0) {
		return -EINVAL;
	}
	
	int sent = uart_fifo_fill(uart_dev, tx_buf, tx_len);
	if (sent != tx_len) {
		LOG_WRN("Incomplete status send: %d/%d bytes", sent, tx_len);
		return -EIO;
	}
	
	return 0;
}

int host_comms_send_demo_status(uint8_t demo_id, uint8_t waypoint_count,
                                 uint16_t bytes_remaining)
{
	if (!uart_dev) {
		return -EINVAL;
	}
	
	struct packet pkt;
	packet_init(&pkt, CMD_DEMO_RECORDING_STATUS);
	
	packet_add_payload(&pkt, &demo_id, 1);
	packet_add_payload(&pkt, &waypoint_count, 1);
	packet_add_payload(&pkt, (const uint8_t *)&bytes_remaining, 2);
	
	packet_finalize(&pkt);
	
	uint8_t tx_buf[PACKET_MAX_SIZE];
	uint8_t tx_len;
	if (packet_serialize(&pkt, tx_buf, &tx_len) != 0) {
		return -EINVAL;
	}
	
	int sent = uart_fifo_fill(uart_dev, tx_buf, tx_len);
	if (sent != tx_len) {
		return -EIO;
	}
	
	return 0;
}

int host_comms_process(uint32_t timeout_ms)
{
	if (!uart_dev) {
		return -ENODEV;
	}
	
	/* If read loop is active, send periodic status */
	if (read_loop_active) {
		struct robot_state state;
		if (host_comms_get_robot_state(&state) == 0) {
			host_comms_send_status(&state);
		}
		k_sleep(K_MSEC(10)); /* 100Hz update rate */
		return 0;
	}
	
	/* Non-blocking read */
	uint8_t byte;
	int ret = uart_fifo_read(uart_dev, &byte, 1);
	if (ret != 1) {
		/* No data available */
		if (timeout_ms > 0) {
			k_sleep(K_MSEC(timeout_ms));
		}
		return -EAGAIN;
	}
	
	/* Look for packet start marker */
	if (byte == PACKET_START_BYTE) {
		rx_buf_pos = 0;
	}
	
	/* Store byte in buffer */
	if (rx_buf_pos < RX_BUF_SIZE) {
		rx_buf[rx_buf_pos++] = byte;
	} else {
		/* Buffer overflow, reset */
		LOG_WRN("RX buffer overflow, resetting");
		rx_buf_pos = 0;
		return -ENOMEM;
	}
	
	/* Try to parse packet if we have enough bytes */
	if (rx_buf_pos >= PACKET_HEADER_SIZE) {
		/* We have header, check if we have complete packet */
		uint8_t payload_len = rx_buf[2];  /* Length field is at index 2 */
		uint16_t expected_total = PACKET_HEADER_SIZE + payload_len + PACKET_CRC_SIZE;
		
		if (rx_buf_pos >= expected_total) {
			/* We have complete packet, attempt to parse */
			struct packet pkt;
			if (packet_parse(rx_buf, rx_buf_pos, &pkt) == 0) {
				/* Valid packet received - enable manual mode automatically */
				if (!manual_mode_active) {
					host_comms_set_manual_mode(true);
				}
				
				/* Dispatch command (logging now suppressed) */
				int cmd_ret = 0;
				switch (pkt.cmd) {
			case CMD_JOG_JOINT:
				cmd_ret = handle_jog_joint(pkt.payload, pkt.payload_len);
				break;
			case CMD_SET_JOINT_DIRECT:
				cmd_ret = handle_set_joint_direct(pkt.payload, pkt.payload_len);
				break;
			case CMD_READ_STATE:
				cmd_ret = handle_read_state();
				break;
			case CMD_START_READ_LOOP:
				cmd_ret = handle_start_read_loop();
				break;
			case CMD_STOP_READ_LOOP:
				cmd_ret = handle_stop_read_loop();
				break;
			case CMD_SINGLE_JOINT_TEST:
				cmd_ret = handle_single_joint_test(pkt.payload, pkt.payload_len);
				break;
			case CMD_STOP:
				cmd_ret = handle_stop();
				break;
			case CMD_START_DEMO_RECORDING:
				cmd_ret = handle_start_demo_recording(pkt.payload, pkt.payload_len);
				break;
			case CMD_ADD_WAYPOINT:
				cmd_ret = handle_add_waypoint(pkt.payload, pkt.payload_len);
				break;
			case CMD_FINISH_DEMO_RECORDING:
				cmd_ret = handle_finish_demo_recording();
				break;
			case CMD_PLAY_DEMO:
				cmd_ret = handle_play_demo(pkt.payload, pkt.payload_len);
				break;
			case CMD_CLEAR_DEMO:
				cmd_ret = handle_clear_demo(pkt.payload, pkt.payload_len);
				break;
			default:
				LOG_WRN("Unknown command: 0x%02X", pkt.cmd);
				cmd_ret = -ENOTSUP;
				break;
			}
			
			if (cmd_ret != 0) {
				LOG_ERR("Command 0x%02X failed: %d", pkt.cmd, cmd_ret);
			}
			
			/* Reset buffer for next packet */
			rx_buf_pos = 0;
			return 0;
			}  /* End of packet_parse if */
		}  /* End of expected_total if */
	}  /* End of PACKET_HEADER_SIZE if */
	
	return -EAGAIN;
}

/* Command Handler Implementations */

static int handle_jog_joint(const uint8_t *payload, uint8_t len)
{
	/* Payload: [joint_id(1), direction(1), step_size_deg(1)] */
	if (len < 3) {
		return -EINVAL;
	}
	
	uint8_t joint_id = payload[0];
	int8_t direction = (int8_t)payload[1]; /* +1 or -1 */
	uint8_t step_deg = payload[2];
	
	/* Read current position */
	uint16_t current_pos;
	int ret = feetech_servo_read_position(joint_id, &current_pos);
	if (ret != 0) {
		return ret;
	}
	
	/* Convert to angle, apply jog, convert back */
	float current_angle = FEETECH_POS_TO_DEG(current_pos);
	float step_deg_float = (float)step_deg;
	float new_angle = current_angle + (direction * step_deg_float);
	
	/* Set new position */
	return feetech_servo_set_goal_angle(joint_id, new_angle);
}

static int handle_set_joint_direct(const uint8_t *payload, uint8_t len)
{
	/* Payload: [joint_id(1), angle_rad(4)] */
	if (len < 5) {
		return -EINVAL;
	}
	
	uint8_t joint_id = payload[0];
	float angle;
	memcpy(&angle, &payload[1], sizeof(float));
	
	return feetech_servo_set_goal_angle(joint_id, angle);
}

static int handle_read_state(void)
{
	struct robot_state state;
	int ret = host_comms_get_robot_state(&state);
	if (ret != 0) {
		return ret;
	}
	
	return host_comms_send_status(&state);
}

static int handle_start_read_loop(void)
{
	read_loop_active = true;
	return 0;
}

static int handle_stop_read_loop(void)
{
	read_loop_active = false;
	return 0;
}

static int handle_single_joint_test(const uint8_t *payload, uint8_t len)
{
	/* Payload: [joint_id(1), start_angle(4), end_angle(4), cycles(1)] */
	if (len < 10) {
		return -EINVAL;
	}
	
	uint8_t joint_id = payload[0];
	float start_angle, end_angle;
	memcpy(&start_angle, &payload[1], sizeof(float));
	memcpy(&end_angle, &payload[5], sizeof(float));
	uint8_t cycles = payload[9];
	
	for (uint8_t i = 0; i < cycles; i++) {
		/* Move to start */
		feetech_servo_set_goal_angle(joint_id, start_angle);
		k_sleep(K_MSEC(1000));
		
		/* Move to end */
		feetech_servo_set_goal_angle(joint_id, end_angle);
		k_sleep(K_MSEC(1000));
	}
	
	return 0;
}

static int handle_stop(void)
{
	LOG_WRN("Emergency stop via command");
	
	/* Disable torque on all servos */
	for (uint8_t id = 1; id <= 6; id++) {
		feetech_servo_set_torque_enable(id, false);
	}
	
	return 0;
}

static int handle_start_demo_recording(const uint8_t *payload, uint8_t len)
{
	if (len < 1) {
		return -EINVAL;
	}
	
	uint8_t demo_id = payload[0];
	if (demo_id > 2) {
		LOG_ERR("Invalid demo ID: %d (must be 0-2)", demo_id);
		return -EINVAL;
	}
	
	demo_recording.is_recording = true;
	demo_recording.demo_id = demo_id;
	demo_recording.waypoint_count = 0;
	
	return 0;
}

static int handle_add_waypoint(const uint8_t *payload, uint8_t len)
{
	/* Payload: [delay_ms(4)] */
	if (len < 4) {
		return -EINVAL;
	}
	
	if (!demo_recording.is_recording) {
		LOG_ERR("Cannot add waypoint: not recording");
		return -EINVAL;
	}
	
	if (demo_recording.waypoint_count >= MAX_DEMO_WAYPOINTS) {
		LOG_ERR("Cannot add waypoint: buffer full");
		return -ENOMEM;
	}
	
	uint32_t delay_ms;
	memcpy(&delay_ms, payload, sizeof(uint32_t));
	
	/* Read current joint angles */
	struct demo_waypoint *wp = &demo_recording.waypoints[demo_recording.waypoint_count];
	for (uint8_t i = 0; i < 6; i++) {
		uint16_t position;
		int ret = feetech_servo_read_position(i + 1, &position);
		if (ret == 0) {
			wp->joint_angles[i] = FEETECH_POS_TO_DEG(position);
		} else {
			wp->joint_angles[i] = 0.0f;
		}
	}
	wp->delay_ms = delay_ms;
	
	demo_recording.waypoint_count++;
	
	LOG_INF("Waypoint %d recorded (delay=%u ms)", 
	        demo_recording.waypoint_count, delay_ms);
	
	/* Send status update */
	uint16_t bytes_remaining = (MAX_DEMO_WAYPOINTS - demo_recording.waypoint_count) * 
	                            sizeof(struct demo_waypoint);
	return host_comms_send_demo_status(demo_recording.demo_id,
	                                    demo_recording.waypoint_count,
	                                    bytes_remaining);
}

static int handle_finish_demo_recording(void)
{
	if (!demo_recording.is_recording) {
		return -EINVAL;
	}
	
	LOG_INF("Finishing demo recording: %d waypoints saved to slot %d",
	        demo_recording.waypoint_count, demo_recording.demo_id);
	
	/* TODO: Write to flash in later phase */
	/* For now, just keep in RAM */
	
	demo_recording.is_recording = false;
	
	return 0;
}

static int handle_play_demo(const uint8_t *payload, uint8_t len)
{
	if (len < 1) {
		return -EINVAL;
	}
	
	uint8_t demo_id = payload[0];
	
	/* For now, only playback the demo in RAM if it matches */
	if (demo_id != demo_recording.demo_id || demo_recording.waypoint_count == 0) {
		LOG_ERR("Demo %d not found or empty", demo_id);
		return -ENOENT;
	}
	
	LOG_INF("Playing demo %d (%d waypoints)", demo_id, demo_recording.waypoint_count);
	
	for (uint8_t i = 0; i < demo_recording.waypoint_count; i++) {
		struct demo_waypoint *wp = &demo_recording.waypoints[i];
		
		/* Send sync write to all joints */
		uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
		int ret = feetech_servo_sync_write_angles(ids, wp->joint_angles, 6);
		if (ret != 0) {
			LOG_ERR("Failed to execute waypoint %d", i);
			return ret;
		}
		
		LOG_DBG("Waypoint %d/%d", i + 1, demo_recording.waypoint_count);
		
		/* Wait for specified delay */
		k_sleep(K_MSEC(wp->delay_ms));
	}
	
	LOG_INF("Demo playback complete");
	return 0;
}

static int handle_clear_demo(const uint8_t *payload, uint8_t len)
{
	if (len < 1) {
		return -EINVAL;
	}
	
	uint8_t demo_id = payload[0];
	
	if (demo_id == demo_recording.demo_id) {
		demo_recording.waypoint_count = 0;
		demo_recording.is_recording = false;
		LOG_INF("Cleared demo %d from RAM", demo_id);
	}
	
	/* TODO: Erase from flash in later phase */
	
	return 0;
}
