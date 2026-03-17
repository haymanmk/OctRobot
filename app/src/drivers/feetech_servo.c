/*
 * OctroBot Robot Arm Firmware - Feetech Servo Driver Implementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>

#include "feetech_servo.h"
#include "feetech_protocol.h"

LOG_MODULE_REGISTER(feetech_servo, LOG_LEVEL_INF);

/* Global UART handle for servo communication */
static hal_uart_handle_t servo_uart = NULL;

int feetech_servo_init(hal_uart_handle_t uart)
{
	if (!uart) {
		return HAL_INVALID;
	}
	
	servo_uart = uart;
	
	LOG_INF("Feetech servo driver initialized");
	
	return HAL_OK;
}

int feetech_servo_ping(uint8_t id)
{
	if (!servo_uart) {
		LOG_ERR("Servo driver not initialized");
		return HAL_ERROR;
	}
	
	return feetech_protocol_ping(servo_uart, id);
}

int feetech_servo_set_torque_enable(uint8_t id, bool enable)
{
	if (!servo_uart) {
		return HAL_ERROR;
	}
	
	uint8_t value = enable ? 1 : 0;
	int ret = feetech_protocol_write(servo_uart, id, FEETECH_REG_TORQUE_ENABLE, &value, 1);
	
	if (ret == HAL_OK) {
		LOG_DBG("Servo %d torque %s", id, enable ? "enabled" : "disabled");
	}
	
	return ret;
}

int feetech_servo_set_goal_position(uint8_t id, uint16_t position)
{
	if (!servo_uart) {
		return HAL_ERROR;
	}
	
	/* Clamp position to valid range */
	if (position > FEETECH_POS_MAX) {
		LOG_WRN("Position %d exceeds max, clamping to %d", position, FEETECH_POS_MAX);
		position = FEETECH_POS_MAX;
	}
	
	/* 
	 * According to Feetech protocol, write 6 consecutive bytes to 0x2A-0x2F:
	 * - Position (2 bytes at 0x2A-0x2B)
	 * - Time (2 bytes at 0x2C-0x2D) - set to 0 for immediate movement
	 * - Speed (2 bytes at 0x2E-0x2F) - default speed in steps/second
	 */
	uint8_t params[6];
	params[0] = position & 0xFF;                         /* Position low byte */
	params[1] = (position >> 8) & 0xFF;                  /* Position high byte */
	params[2] = 0;                                        /* Time low byte (0 = immediate) */
	params[3] = 0;                                        /* Time high byte */
	params[4] = FEETECH_DEFAULT_SPEED & 0xFF;            /* Speed low byte */
	params[5] = (FEETECH_DEFAULT_SPEED >> 8) & 0xFF;     /* Speed high byte */
	
	int ret = feetech_protocol_write(servo_uart, id, FEETECH_REG_GOAL_POSITION_L, params, 6);
	
	if (ret == HAL_OK) {
		LOG_DBG("Servo %d goal position: %d", id, position);
	}
	
	return ret;
}

int feetech_servo_set_goal_angle(uint8_t id, float angle)
{
	uint16_t position = FEETECH_DEG_TO_POS(angle);
	return feetech_servo_set_goal_position(id, position);
}

int feetech_servo_set_goal_speed(uint8_t id, uint16_t speed)
{
	if (!servo_uart) {
		return HAL_ERROR;
	}
	
	if (speed > FEETECH_VEL_MAX) {
		LOG_WRN("Speed %d exceeds max, clamping to %d", speed, FEETECH_VEL_MAX);
		speed = FEETECH_VEL_MAX;
	}
	
	return feetech_protocol_write_word(servo_uart, id, FEETECH_REG_GOAL_SPEED_L, speed);
}

int feetech_servo_set_goal_time(uint8_t id, uint16_t time_ms)
{
	if (!servo_uart) {
		return HAL_ERROR;
	}
	
	return feetech_protocol_write_word(servo_uart, id, FEETECH_REG_GOAL_TIME_L, time_ms);
}

int feetech_servo_set_acceleration(uint8_t id, uint8_t acceleration)
{
	if (!servo_uart) {
		return HAL_ERROR;
	}
	
	return feetech_protocol_write(servo_uart, id, FEETECH_REG_ACCELERATION, &acceleration, 1);
}

int feetech_servo_set_goal_position_ex(uint8_t id, uint16_t position, 
                                        uint16_t time_ms, uint16_t speed)
{
	if (!servo_uart) {
		return HAL_ERROR;
	}
	
	/* Clamp position to valid range */
	if (position > FEETECH_POS_MAX) {
		LOG_WRN("Position %d exceeds max, clamping to %d", position, FEETECH_POS_MAX);
		position = FEETECH_POS_MAX;
	}
	
	/* Clamp speed to valid range */
	if (speed > FEETECH_VEL_MAX) {
		LOG_WRN("Speed %d exceeds max, clamping to %d", speed, FEETECH_VEL_MAX);
		speed = FEETECH_VEL_MAX;
	}
	
	/* 
	 * Write 6 consecutive bytes to 0x2A-0x2F per Feetech protocol:
	 * - Position (2 bytes at 0x2A-0x2B)
	 * - Time (2 bytes at 0x2C-0x2D)
	 * - Speed (2 bytes at 0x2E-0x2F)
	 */
	uint8_t params[6];
	params[0] = position & 0xFF;        /* Position low byte */
	params[1] = (position >> 8) & 0xFF; /* Position high byte */
	params[2] = time_ms & 0xFF;         /* Time low byte */
	params[3] = (time_ms >> 8) & 0xFF;  /* Time high byte */
	params[4] = speed & 0xFF;           /* Speed low byte */
	params[5] = (speed >> 8) & 0xFF;    /* Speed high byte */
	
	int ret = feetech_protocol_write(servo_uart, id, FEETECH_REG_GOAL_POSITION_L, params, 6);
	
	if (ret == HAL_OK) {
		LOG_DBG("Servo %d goal: pos=%d, time=%dms, speed=%d", id, position, time_ms, speed);
	}
	
	return ret;
}

int feetech_servo_read_position(uint8_t id, uint16_t *position)
{
	if (!servo_uart || !position) {
		return HAL_INVALID;
	}
	
	return feetech_protocol_read_word(servo_uart, id, FEETECH_REG_PRESENT_POSITION_L, position);
}

int feetech_servo_read_angle(uint8_t id, float *angle)
{
	if (!angle) {
		return HAL_INVALID;
	}
	
	uint16_t position;
	int ret = feetech_servo_read_position(id, &position);
	
	if (ret == HAL_OK) {
		*angle = FEETECH_POS_TO_DEG(position);
	}
	
	return ret;
}

int feetech_servo_read_load(uint8_t id, uint16_t *load)
{
	if (!servo_uart || !load) {
		return HAL_INVALID;
	}
	
	return feetech_protocol_read_word(servo_uart, id, FEETECH_REG_PRESENT_LOAD_L, load);
}

int feetech_servo_read_temperature(uint8_t id, uint8_t *temperature)
{
	if (!servo_uart || !temperature) {
		return HAL_INVALID;
	}
	
	return feetech_protocol_read(servo_uart, id, FEETECH_REG_PRESENT_TEMPERATURE, 1, temperature);
}

int feetech_servo_read_voltage(uint8_t id, uint8_t *voltage)
{
	if (!servo_uart || !voltage) {
		return HAL_INVALID;
	}
	
	return feetech_protocol_read(servo_uart, id, FEETECH_REG_PRESENT_VOLTAGE, 1, voltage);
}

int feetech_servo_read_state(uint8_t id, struct feetech_servo_state *state)
{
	if (!servo_uart || !state) {
		return HAL_INVALID;
	}
	
	int ret;
	
	/* Read position */
	ret = feetech_servo_read_position(id, &state->present_position);
	if (ret != HAL_OK) {
		return ret;
	}
	
	/* Read speed */
	ret = feetech_protocol_read_word(servo_uart, id, FEETECH_REG_PRESENT_SPEED_L,
	                        &state->present_speed);
	if (ret != HAL_OK) {
		return ret;
	}
	
	/* Read load */
	ret = feetech_servo_read_load(id, &state->present_load);
	if (ret != HAL_OK) {
		return ret;
	}
	
	/* Read voltage */
	ret = feetech_servo_read_voltage(id, &state->present_voltage);
	if (ret != HAL_OK) {
		return ret;
	}
	
	/* Read temperature */
	ret = feetech_servo_read_temperature(id, &state->present_temperature);
	if (ret != HAL_OK) {
		return ret;
	}
	
	/* Read moving status */
	uint8_t moving;
	ret = feetech_protocol_read(servo_uart, id, FEETECH_REG_MOVING, 1, &moving);
	if (ret != HAL_OK) {
		return ret;
	}
	state->is_moving = (moving != 0);
	
	state->error = 0; /* Error flags would be in response packets */
	
	LOG_DBG("Servo %d state: pos=%d, temp=%d°C, voltage=%d.%dV",
	        id, state->present_position, state->present_temperature,
	        state->present_voltage / 10, state->present_voltage % 10);
	
	return HAL_OK;
}

int feetech_servo_read_multi_states(const uint8_t *ids, struct feetech_servo_state *states,
									uint8_t count)
{
	if (!servo_uart || !ids || !states) {
		return HAL_INVALID;
	}
	
	if (count == 0 || count > FEETECH_MAX_SERVOS) {
		return HAL_INVALID;
	}
	
	for (uint8_t i = 0; i < count; i++) {
		int ret = feetech_servo_read_state(ids[i], &states[i]);
		if (ret != HAL_OK) {
			LOG_ERR("Failed to read state from servo %d", ids[i]);
			return ret;
		}
	}
	
	return HAL_OK;
}

int feetech_servo_sync_write_positions(const uint8_t *ids, const uint16_t *positions,
                                        uint8_t count)
{
	if (!servo_uart || !ids || !positions) {
		return HAL_INVALID;
	}
	
	if (count == 0 || count > FEETECH_MAX_SERVOS) {
		LOG_ERR("Invalid servo count: %d", count);
		return HAL_INVALID;
	}
	
	/* 
	 * Pack 6 bytes per servo per Feetech protocol (0x2A-0x2F):
	 * - Position (2 bytes)
	 * - Time (2 bytes, 0 = immediate)
	 * - Speed (2 bytes, default speed in steps/second)
	 */
	uint8_t data[FEETECH_MAX_SERVOS * 6];
	for (uint8_t i = 0; i < count; i++) {
		data[i * 6 + 0] = (uint8_t)(positions[i] & 0xFF);                /* Position low */
		data[i * 6 + 1] = (uint8_t)((positions[i] >> 8) & 0xFF);         /* Position high */
		data[i * 6 + 2] = 0;                                              /* Time low (0 = immediate) */
		data[i * 6 + 3] = 0;                                              /* Time high */
		data[i * 6 + 4] = FEETECH_DEFAULT_SPEED & 0xFF;                  /* Speed low */
		data[i * 6 + 5] = (FEETECH_DEFAULT_SPEED >> 8) & 0xFF;           /* Speed high */
	}
	
	int ret = feetech_protocol_sync_write(servo_uart, FEETECH_REG_GOAL_POSITION_L, 6,
	                              ids, data, count);
	
	if (ret == HAL_OK) {
		LOG_DBG("Sync write %d servo positions", count);
	}
	
	return ret;
}

int feetech_servo_sync_write_angles(const uint8_t *ids, const float *angles,
                                     uint8_t count)
{
	if (!ids || !angles) {
		return HAL_INVALID;
	}
	
	if (count == 0 || count > FEETECH_MAX_SERVOS) {
		return HAL_INVALID;
	}
	
	/* Convert angles to positions */
	uint16_t positions[FEETECH_MAX_SERVOS];
	for (uint8_t i = 0; i < count; i++) {
		positions[i] = FEETECH_DEG_TO_POS(angles[i]);
	}
	
	return feetech_servo_sync_write_positions(ids, positions, count);
}

int feetech_servo_read_multi_positions(const uint8_t *ids, uint16_t *positions,
                                        uint8_t count)
{
	if (!servo_uart || !ids || !positions) {
		return HAL_INVALID;
	}
	
	if (count == 0 || count > FEETECH_MAX_SERVOS) {
		return HAL_INVALID;
	}
	
	/* Read each servo individually (no sync read in basic protocol) */
	for (uint8_t i = 0; i < count; i++) {
		int ret = feetech_servo_read_position(ids[i], &positions[i]);
		if (ret != HAL_OK) {
			LOG_ERR("Failed to read position from servo %d", ids[i]);
			return ret;
		}
	}
	
	return HAL_OK;
}
