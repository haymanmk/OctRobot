/*
 * OctroBot Robot Arm Firmware - Feetech Protocol Implementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "feetech_protocol.h"

LOG_MODULE_REGISTER(feetech_protocol, LOG_LEVEL_DBG);

uint8_t feetech_calculate_checksum(uint8_t id, uint8_t length, uint8_t instruction,
                                    const uint8_t *params, uint8_t param_len)
{
	uint8_t sum = id + length + instruction;
	
	for (uint8_t i = 0; i < param_len; i++) {
		sum += params[i];
	}
	
	return ~sum & 0xFF;
}

int feetech_build_packet(const struct feetech_packet *packet, uint8_t *buffer,
                         size_t buffer_size)
{
	if (!packet || !buffer) {
		return HAL_INVALID;
	}
	
	/* Calculate required buffer size */
	size_t packet_len = FEETECH_MIN_PACKET_SIZE + packet->param_length;
	
	if (buffer_size < packet_len) {
		LOG_ERR("Buffer too small: need %zu, have %zu", packet_len, buffer_size);
		return HAL_INVALID;
	}
	
	/* Build packet */
	uint8_t idx = 0;
	buffer[idx++] = FEETECH_HEADER_1;
	buffer[idx++] = FEETECH_HEADER_2;
	buffer[idx++] = packet->id;
	buffer[idx++] = packet->param_length + 2; /* length = params + instruction + checksum */
	buffer[idx++] = packet->instruction;
	
	/* Copy parameters */
	if (packet->param_length > 0) {
		memcpy(&buffer[idx], packet->parameters, packet->param_length);
		idx += packet->param_length;
	}
	
	/* Calculate and append checksum */
	uint8_t checksum = feetech_calculate_checksum(packet->id,
	                                               packet->param_length + 2,
	                                               packet->instruction,
	                                               packet->parameters,
	                                               packet->param_length);
	buffer[idx++] = checksum;
	
	LOG_DBG("Built packet: ID=%d, INST=0x%02X, LEN=%d, CHKSUM=0x%02X",
	        packet->id, packet->instruction, packet->param_length, checksum);
	
	return (int)packet_len;
}

int feetech_parse_packet(const uint8_t *buffer, size_t buffer_len,
                         struct feetech_packet *packet)
{
	if (!buffer || !packet || buffer_len < FEETECH_MIN_PACKET_SIZE) {
		return HAL_INVALID;
	}
	
	/* Verify headers */
	if (buffer[0] != FEETECH_HEADER_1 || buffer[1] != FEETECH_HEADER_2) {
		LOG_ERR("Invalid packet headers: 0x%02X 0x%02X", buffer[0], buffer[1]);
		return HAL_ERROR;
	}
	
	/* Extract packet fields */
	packet->id = buffer[2];
	uint8_t length = buffer[3];
	packet->error = buffer[4];
	
	/* Validate length */
	if (length < 2) {
		LOG_ERR("Invalid packet length: %d", length);
		return HAL_ERROR;
	}
	
	packet->param_length = length - 2; /* length includes error + checksum */
	
	/* Check if we have complete packet */
	size_t expected_len = 4 + length; /* headers + id + length + payload */
	if (buffer_len < expected_len) {
		LOG_ERR("Incomplete packet: got %zu, need %zu", buffer_len, expected_len);
		return HAL_ERROR;
	}
	
	/* Copy parameters */
	if (packet->param_length > 0) {
		if (packet->param_length > FEETECH_MAX_PACKET_SIZE) {
			LOG_ERR("Parameter length too large: %d", packet->param_length);
			return HAL_ERROR;
		}
		memcpy(packet->parameters, &buffer[5], packet->param_length);
	}
	
	/* Verify checksum */
	/* Total packet bytes = 4 + length, checksum is the last byte. */
	uint8_t received_checksum = buffer[expected_len - 1];
	uint8_t calculated_checksum = feetech_calculate_checksum(packet->id, length,
	                                                          packet->error,
	                                                          packet->parameters,
	                                                          packet->param_length);
	
	if (received_checksum != calculated_checksum) {
		LOG_ERR("Checksum mismatch: got 0x%02X, calc 0x%02X",
		        received_checksum, calculated_checksum);
		return HAL_ERROR;
	}
	
	/* Check for error flags */
	if (packet->error != 0) {
		LOG_WRN("Servo error flags: 0x%02X", packet->error);
	}
	
	LOG_DBG("Parsed packet: ID=%d, ERR=0x%02X, PARAMS=%d",
	        packet->id, packet->error, packet->param_length);
	
	return HAL_OK;
}

int feetech_ping(hal_uart_handle_t uart, uint8_t id)
{
	if (!uart) {
		return HAL_INVALID;
	}
	
	LOG_DBG("Pinging servo ID %d", id);
	
	/* Build ping packet */
	struct feetech_packet tx_packet = {
		.id = id,
		.instruction = FEETECH_INST_PING,
		.param_length = 0,
	};
	
	uint8_t tx_buffer[FEETECH_MIN_PACKET_SIZE];
	int len = feetech_build_packet(&tx_packet, tx_buffer, sizeof(tx_buffer));
	if (len < 0) {
		return len;
	}
	
	/* Transmit ping */
	int ret = half_duplex_uart_transmit(uart, tx_buffer, len);
	if (ret < 0) {
		LOG_ERR("Failed to transmit ping: %d", ret);
		return ret;
	}
	
	/* Wait for response (only if not broadcast) */
	if (id == FEETECH_BROADCAST_ID) {
		return HAL_OK; /* No response expected */
	}
	
	uint8_t rx_buffer[FEETECH_MIN_PACKET_SIZE];
	ret = half_duplex_uart_receive(uart, rx_buffer, FEETECH_MIN_PACKET_SIZE, 50);
	if (ret < 0) {
		LOG_ERR("No response from servo %d", id);
		return ret;
	}
	
	/* Parse response */
	struct feetech_packet rx_packet;
	ret = feetech_parse_packet(rx_buffer, ret, &rx_packet);
	if (ret != HAL_OK) {
		return ret;
	}
	
	if (rx_packet.id != id) {
		LOG_ERR("Response ID mismatch: expected %d, got %d", id, rx_packet.id);
		return HAL_ERROR;
	}
	
	LOG_INF("Servo %d responded to ping", id);
	return HAL_OK;
}

int feetech_read(hal_uart_handle_t uart, uint8_t id, uint8_t reg_addr,
                 uint8_t length, uint8_t *data)
{
	if (!uart || !data) {
		return HAL_INVALID;
	}
	
	LOG_DBG("Reading %d bytes from servo %d reg 0x%02X", length, id, reg_addr);
	
	/* Build read packet */
	struct feetech_packet tx_packet = {
		.id = id,
		.instruction = FEETECH_INST_READ,
		.param_length = 2,
	};
	tx_packet.parameters[0] = reg_addr;
	tx_packet.parameters[1] = length;
	
	uint8_t tx_buffer[FEETECH_MIN_PACKET_SIZE + 2];
	int len = feetech_build_packet(&tx_packet, tx_buffer, sizeof(tx_buffer));
	if (len < 0) {
		return len;
	}
	
	/* Transmit read request */
	int ret = half_duplex_uart_transmit(uart, tx_buffer, len);
	if (ret < 0) {
		return ret;
	}
	
	/* Wait for response */
	uint8_t rx_buffer[FEETECH_MAX_PACKET_SIZE];
	size_t expected_response_len = FEETECH_MIN_PACKET_SIZE + length;
	ret = half_duplex_uart_receive(uart, rx_buffer, expected_response_len, 50);
	if (ret < 0) {
		LOG_ERR("No read response from servo %d", id);
		return ret;
	}
	
	/* Parse response */
	struct feetech_packet rx_packet;
	ret = feetech_parse_packet(rx_buffer, ret, &rx_packet);
	if (ret != HAL_OK) {
		return ret;
	}
	
	if (rx_packet.id != id) {
		LOG_ERR("Response ID mismatch");
		return HAL_ERROR;
	}
	
	if (rx_packet.param_length != length) {
		LOG_ERR("Response length mismatch: expected %d, got %d",
		        length, rx_packet.param_length);
		return HAL_ERROR;
	}
	
	/* Copy data */
	memcpy(data, rx_packet.parameters, length);
	
	return HAL_OK;
}

int feetech_write(hal_uart_handle_t uart, uint8_t id, uint8_t reg_addr,
                  const uint8_t *data, uint8_t length)
{
	if (!uart || !data) {
		return HAL_INVALID;
	}
	
	LOG_DBG("Writing %d bytes to servo %d reg 0x%02X", length, id, reg_addr);
	
	/* Build write packet */
	struct feetech_packet tx_packet = {
		.id = id,
		.instruction = FEETECH_INST_WRITE,
		.param_length = (uint8_t)(1 + length),
	};
	tx_packet.parameters[0] = reg_addr;
	memcpy(&tx_packet.parameters[1], data, length);
	
	uint8_t tx_buffer[FEETECH_MAX_PACKET_SIZE];
	int len = feetech_build_packet(&tx_packet, tx_buffer, sizeof(tx_buffer));
	if (len < 0) {
		return len;
	}
	
	/* Transmit write command */
	int ret = half_duplex_uart_transmit(uart, tx_buffer, len);
	if (ret < 0) {
		return ret;
	}
	
	/* Wait for response (only if not broadcast) */
	if (id == FEETECH_BROADCAST_ID) {
		return HAL_OK;
	}
	
	uint8_t rx_buffer[FEETECH_MIN_PACKET_SIZE];
	ret = half_duplex_uart_receive(uart, rx_buffer, FEETECH_MIN_PACKET_SIZE, 50);
	if (ret < 0) {
		LOG_WRN("No write response from servo %d (may be expected)", id);
		return HAL_OK; /* Some servos don't respond to writes */
	}
	
	/* Parse response */
	struct feetech_packet rx_packet;
	ret = feetech_parse_packet(rx_buffer, ret, &rx_packet);
	if (ret != HAL_OK) {
		return ret;
	}
	
	if (rx_packet.error != 0) {
		LOG_ERR("Write error: 0x%02X", rx_packet.error);
		return HAL_ERROR;
	}
	
	return HAL_OK;
}

int feetech_sync_write(hal_uart_handle_t uart, uint8_t reg_addr, uint8_t data_len,
                       const uint8_t *servo_ids, const uint8_t *servo_data,
                       uint8_t servo_count)
{
	if (!uart || !servo_ids || !servo_data || servo_count == 0) {
		return HAL_INVALID;
	}
	
	LOG_DBG("Sync write to %d servos at reg 0x%02X (%d bytes each)",
	        servo_count, reg_addr, data_len);
	
	/* Build sync write packet */
	struct feetech_packet tx_packet = {
		.id = FEETECH_BROADCAST_ID,
		.instruction = FEETECH_INST_SYNC_WRITE,
		.param_length = (uint8_t)(2 + servo_count * (1 + data_len)),
	};
	
	/* First two params are register address and data length */
	tx_packet.parameters[0] = reg_addr;
	tx_packet.parameters[1] = data_len;
	
	/* Pack servo ID and data for each servo */
	uint8_t param_idx = 2;
	for (uint8_t i = 0; i < servo_count; i++) {
		tx_packet.parameters[param_idx++] = servo_ids[i];
		memcpy(&tx_packet.parameters[param_idx], &servo_data[i * data_len], data_len);
		param_idx += data_len;
	}
	
	uint8_t tx_buffer[FEETECH_MAX_PACKET_SIZE];
	int len = feetech_build_packet(&tx_packet, tx_buffer, sizeof(tx_buffer));
	if (len < 0) {
		return len;
	}
	
	/* Transmit sync write (no response expected for broadcast) */
	int ret = half_duplex_uart_transmit(uart, tx_buffer, len);
	if (ret < 0) {
		return ret;
	}
	
	LOG_DBG("Sync write complete");
	
	return HAL_OK;
}

int feetech_read_word(hal_uart_handle_t uart, uint8_t id, uint8_t reg_addr,
                      uint16_t *value)
{
	if (!value) {
		return HAL_INVALID;
	}
	
	uint8_t data[2];
	int ret = feetech_read(uart, id, reg_addr, 2, data);
	if (ret != HAL_OK) {
		return ret;
	}
	
	/* Little-endian */
	*value = (uint16_t)(data[0] | (data[1] << 8));
	
	return HAL_OK;
}

int feetech_write_word(hal_uart_handle_t uart, uint8_t id, uint8_t reg_addr,
                       uint16_t value)
{
	/* Little-endian */
	uint8_t data[2] = {
		(uint8_t)(value & 0xFF),
		(uint8_t)((value >> 8) & 0xFF),
	};
	
	return feetech_write(uart, id, reg_addr, data, 2);
}
