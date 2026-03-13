/*
 * OctroBot Robot Arm Firmware - Packet Protocol Header
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Binary packet protocol for USB-CDC host communication.
 * Format: [0xAA] [CMD] [PAYLOAD_LEN] [PAYLOAD...] [CRC8]
 *
 * Educational Note:
 * - 0xAA is the packet start marker (easily distinguishable from data)
 * - CMD byte identifies the command type
 * - PAYLOAD_LEN is the number of payload bytes (0-255)
 * - CRC8 uses Dallas/Maxim polynomial for error detection
 */

#ifndef PACKET_PROTOCOL_H
#define PACKET_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Protocol Constants
 */
#define PACKET_START_BYTE 0xAA
#define PACKET_HEADER_SIZE 3  /* START + CMD + LEN */
#define PACKET_CRC_SIZE 1
#define PACKET_MAX_PAYLOAD 128
#define PACKET_MAX_SIZE (PACKET_HEADER_SIZE + PACKET_MAX_PAYLOAD + PACKET_CRC_SIZE)

/*
 * Core Commands (0x01-0x0F)
 */
#define CMD_MOVE_JOINTS 0x01       /* Move joints with trajectory planning */
#define CMD_MOVE_CARTESIAN 0x02    /* Move end-effector in Cartesian space */
#define CMD_READ_STATE 0x03        /* Request current robot state */
#define CMD_STOP 0x04              /* Emergency stop */
#define CMD_HOME 0x05              /* Move to home position */
#define CMD_SET_PARAMS 0x10        /* Update motion parameters */

/*
 * Manual/Debug Mode Commands (0x20-0x2F)
 */
#define CMD_JOG_JOINT 0x20         /* Incremental joint movement */
#define CMD_SET_JOINT_DIRECT 0x21  /* Direct position command (bypass trajectory) */
#define CMD_START_READ_LOOP 0x22   /* Start continuous position streaming */
#define CMD_STOP_READ_LOOP 0x23    /* Stop streaming */
#define CMD_SINGLE_JOINT_TEST 0x24 /* Oscillate single joint for testing */

/*
 * Demo Recording & Playback Commands (0x30-0x3F)
 */
#define CMD_PLAY_DEMO 0x30              /* Play recorded demo sequence */
#define CMD_START_DEMO_RECORDING 0x31   /* Begin recording waypoints */
#define CMD_ADD_WAYPOINT 0x32           /* Capture current joint angles */
#define CMD_FINISH_DEMO_RECORDING 0x33  /* Save demo to flash */
#define CMD_CLEAR_DEMO 0x34             /* Erase stored demo */

/*
 * Status Reports (0xF0-0xFF)
 */
#define CMD_STATUS_REPORT 0xF0           /* MCU→host: periodic status */
#define CMD_DEMO_RECORDING_STATUS 0xF2   /* MCU→host: recording progress */

/*
 * Packet Structure
 */
struct packet {
	uint8_t start;         /* Always PACKET_START_BYTE */
	uint8_t cmd;           /* Command byte */
	uint8_t payload_len;   /* Number of payload bytes */
	uint8_t payload[PACKET_MAX_PAYLOAD];
	uint8_t crc;           /* CRC8 checksum */
};

/*
 * Packet Builder/Parser Functions
 */

/**
 * @brief Initialize a packet with start byte and command.
 * 
 * @param pkt Pointer to packet structure
 * @param cmd Command byte
 */
void packet_init(struct packet *pkt, uint8_t cmd);

/**
 * @brief Add payload data to a packet.
 * 
 * @param pkt Pointer to packet structure
 * @param data Pointer to data buffer
 * @param len Number of bytes to add
 * @return 0 on success, -1 if payload would exceed max size
 */
int packet_add_payload(struct packet *pkt, const uint8_t *data, uint8_t len);

/**
 * @brief Finalize packet by computing and setting CRC8.
 * 
 * Educational Note:
 * CRC8 (Dallas/Maxim polynomial: x^8 + x^5 + x^4 + 1) detects:
 * - All single-bit errors
 * - All double-bit errors
 * - All burst errors up to 8 bits
 * 
 * @param pkt Pointer to packet structure
 */
void packet_finalize(struct packet *pkt);

/**
 * @brief Verify packet CRC.
 * 
 * @param pkt Pointer to packet structure
 * @return true if CRC is valid, false otherwise
 */
bool packet_verify_crc(const struct packet *pkt);

/**
 * @brief Serialize packet to byte buffer for transmission.
 * 
 * @param pkt Pointer to packet structure
 * @param buf Output buffer (must be at least PACKET_MAX_SIZE bytes)
 * @param out_len Pointer to store actual packet length
 * @return 0 on success, -1 on error
 */
int packet_serialize(const struct packet *pkt, uint8_t *buf, uint8_t *out_len);

/**
 * @brief Parse received bytes into packet structure.
 * 
 * @param buf Input buffer containing received packet
 * @param len Number of bytes in buffer
 * @param pkt Output packet structure
 * @return 0 on success, -1 if invalid packet or CRC mismatch
 */
int packet_parse(const uint8_t *buf, uint8_t len, struct packet *pkt);

/**
 * @brief Compute CRC8 checksum using Dallas/Maxim polynomial.
 * 
 * Implementation note: This is a table-less implementation optimized
 * for code size. For high-throughput applications, a 256-byte lookup
 * table would be faster.
 * 
 * @param data Pointer to data buffer
 * @param len Number of bytes
 * @return CRC8 checksum
 */
uint8_t crc8_compute(const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* PACKET_PROTOCOL_H */
