/*
 * OctroBot Robot Arm Firmware - Packet Protocol Implementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "packet_protocol.h"

/**
 * CRC8 Dallas/Maxim polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
 * 
 * Educational Note:
 * This polynomial is optimal for detecting errors in byte-oriented protocols.
 * The algorithm processes each bit and updates the CRC register using XOR
 * operations. No lookup table is used to minimize code size.
 */
uint8_t crc8_compute(const uint8_t *data, uint8_t len)
{
	uint8_t crc = 0x00; /* Initial value */
	
	for (uint8_t i = 0; i < len; i++) {
		crc ^= data[i]; /* XOR byte into CRC */
		
		/* Process each bit */
		for (uint8_t bit = 0; bit < 8; bit++) {
			if (crc & 0x80) {
				/* If MSB is 1, shift and XOR with polynomial */
				crc = (crc << 1) ^ 0x31;
			} else {
				/* Otherwise just shift */
				crc = (crc << 1);
			}
		}
	}
	
	return crc;
}

void packet_init(struct packet *pkt, uint8_t cmd)
{
	if (!pkt) {
		return;
	}
	
	pkt->start = PACKET_START_BYTE;
	pkt->cmd = cmd;
	pkt->payload_len = 0;
	pkt->crc = 0;
	
	/* Zero out payload */
	memset(pkt->payload, 0, PACKET_MAX_PAYLOAD);
}

int packet_add_payload(struct packet *pkt, const uint8_t *data, uint8_t len)
{
	if (!pkt || !data) {
		return -1;
	}
	
	/* Check if adding this data would exceed max payload size */
	if (pkt->payload_len + len > PACKET_MAX_PAYLOAD) {
		return -1;
	}
	
	/* Copy data to payload buffer */
	memcpy(&pkt->payload[pkt->payload_len], data, len);
	pkt->payload_len += len;
	
	return 0;
}

void packet_finalize(struct packet *pkt)
{
	if (!pkt) {
		return;
	}
	
	/* CRC is computed over: CMD + PAYLOAD_LEN + PAYLOAD */
	uint8_t crc_data[2 + PACKET_MAX_PAYLOAD];
	crc_data[0] = pkt->cmd;
	crc_data[1] = pkt->payload_len;
	
	if (pkt->payload_len > 0) {
		memcpy(&crc_data[2], pkt->payload, pkt->payload_len);
	}
	
	pkt->crc = crc8_compute(crc_data, 2 + pkt->payload_len);
}

bool packet_verify_crc(const struct packet *pkt)
{
	if (!pkt) {
		return false;
	}
	
	/* Recompute CRC and compare */
	uint8_t crc_data[2 + PACKET_MAX_PAYLOAD];
	crc_data[0] = pkt->cmd;
	crc_data[1] = pkt->payload_len;
	
	if (pkt->payload_len > 0) {
		memcpy(&crc_data[2], pkt->payload, pkt->payload_len);
	}
	
	uint8_t computed_crc = crc8_compute(crc_data, 2 + pkt->payload_len);
	
	return (computed_crc == pkt->crc);
}

int packet_serialize(const struct packet *pkt, uint8_t *buf, uint8_t *out_len)
{
	if (!pkt || !buf || !out_len) {
		return -1;
	}
	
	/* Total packet size: START + CMD + LEN + PAYLOAD + CRC */
	uint8_t total_len = PACKET_HEADER_SIZE + pkt->payload_len + PACKET_CRC_SIZE;
	
	if (total_len > PACKET_MAX_SIZE) {
		return -1;
	}
	
	/* Build packet byte by byte */
	buf[0] = pkt->start;
	buf[1] = pkt->cmd;
	buf[2] = pkt->payload_len;
	
	if (pkt->payload_len > 0) {
		memcpy(&buf[3], pkt->payload, pkt->payload_len);
	}
	
	buf[3 + pkt->payload_len] = pkt->crc;
	
	*out_len = total_len;
	return 0;
}

int packet_parse(const uint8_t *buf, uint8_t len, struct packet *pkt)
{
	if (!buf || !pkt || len < PACKET_HEADER_SIZE + PACKET_CRC_SIZE) {
		return -1; /* Buffer too small for minimum packet */
	}
	
	/* Check start byte */
	if (buf[0] != PACKET_START_BYTE) {
		return -1;
	}
	
	/* Extract fields */
	pkt->start = buf[0];
	pkt->cmd = buf[1];
	pkt->payload_len = buf[2];
	
	/* Verify total length */
	uint8_t expected_len = PACKET_HEADER_SIZE + pkt->payload_len + PACKET_CRC_SIZE;
	if (len < expected_len) {
		return -1; /* Incomplete packet */
	}
	
	/* Check payload length */
	if (pkt->payload_len > PACKET_MAX_PAYLOAD) {
		return -1;
	}
	
	/* Copy payload */
	if (pkt->payload_len > 0) {
		memcpy(pkt->payload, &buf[3], pkt->payload_len);
	}
	
	/* Extract CRC */
	pkt->crc = buf[3 + pkt->payload_len];
	
	/* Verify CRC */
	if (!packet_verify_crc(pkt)) {
		return -1; /* CRC mismatch */
	}
	
	return 0;
}
