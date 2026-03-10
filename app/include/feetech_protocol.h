/*
 * OctroBot Robot Arm Firmware - Feetech SCS/STS Protocol
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Feetech Serial Communication Servo (SCS) and Serial Transfer Servo (STS)
 * protocol implementation.
 *
 * Protocol format: [0xFF] [0xFF] [ID] [LENGTH] [INSTRUCTION] [PARAMS...] [CHECKSUM]
 */

#ifndef OCTROBOT_FEETECH_PROTOCOL_H
#define OCTROBOT_FEETECH_PROTOCOL_H

#include "hal_types.h"
#include "half_duplex_uart.h"

/* Protocol constants */
#define FEETECH_HEADER_BYTE_1           0xFF
#define FEETECH_HEADER_BYTE_2           0xFF
#define FEETECH_BROADCAST_ID            0xFE
#define FEETECH_MIN_PACKET_SIZE         6    /* Header(2) + ID + Len + Instr + Checksum */
#define FEETECH_MAX_PACKET_SIZE         256
#define FEETECH_MAX_PARAMS              250

/* Instruction set */
#define FEETECH_INST_PING               0x01
#define FEETECH_INST_READ               0x02
#define FEETECH_INST_WRITE              0x03
#define FEETECH_INST_REG_WRITE          0x04
#define FEETECH_INST_ACTION             0x05
#define FEETECH_INST_SYNC_WRITE         0x83
#define FEETECH_INST_RESET              0x06

/* Control table addresses (common across SCS/STS series) */
#define FEETECH_ADDR_MODEL_NUMBER_L     0x00
#define FEETECH_ADDR_MODEL_NUMBER_H     0x01
#define FEETECH_ADDR_FIRMWARE_VERSION   0x02
#define FEETECH_ADDR_ID                 0x05
#define FEETECH_ADDR_BAUD_RATE          0x06
#define FEETECH_ADDR_RETURN_DELAY       0x07
#define FEETECH_ADDR_RESPONSE_STATUS    0x08
#define FEETECH_ADDR_MIN_ANGLE_LIMIT_L  0x09
#define FEETECH_ADDR_MIN_ANGLE_LIMIT_H  0x0A
#define FEETECH_ADDR_MAX_ANGLE_LIMIT_L  0x0B
#define FEETECH_ADDR_MAX_ANGLE_LIMIT_H  0x0C
#define FEETECH_ADDR_MAX_TEMP_LIMIT     0x0D
#define FEETECH_ADDR_MAX_VOLTAGE_LIMIT  0x0E
#define FEETECH_ADDR_MIN_VOLTAGE_LIMIT  0x0F
#define FEETECH_ADDR_MAX_TORQUE_L       0x10
#define FEETECH_ADDR_MAX_TORQUE_H       0x11
#define FEETECH_ADDR_UNLOAD_CONDITION   0x13
#define FEETECH_ADDR_LED_ALARM          0x14
#define FEETECH_ADDR_P_COEF             0x15
#define FEETECH_ADDR_D_COEF             0x16
#define FEETECH_ADDR_I_COEF             0x17
#define FEETECH_ADDR_MIN_PWM_L          0x18
#define FEETECH_ADDR_MIN_PWM_H          0x19
#define FEETECH_ADDR_CW_DEAD_ZONE       0x1A
#define FEETECH_ADDR_CCW_DEAD_ZONE      0x1B
#define FEETECH_ADDR_OVERCURRENT_PROTECT_L 0x1C
#define FEETECH_ADDR_OVERCURRENT_PROTECT_H 0x1D
#define FEETECH_ADDR_ANGLE_OFFSET_L     0x1E
#define FEETECH_ADDR_ANGLE_OFFSET_H     0x1F
#define FEETECH_ADDR_OVERLOAD_CURRENT   0x20

/* RAM area (volatile, reset on power cycle) */
#define FEETECH_ADDR_TORQUE_ENABLE      0x28
#define FEETECH_ADDR_LED                0x29
#define FEETECH_ADDR_GOAL_POSITION_L    0x2A
#define FEETECH_ADDR_GOAL_POSITION_H    0x2B
#define FEETECH_ADDR_GOAL_TIME_L        0x2C
#define FEETECH_ADDR_GOAL_TIME_H        0x2D
#define FEETECH_ADDR_GOAL_SPEED_L       0x2E
#define FEETECH_ADDR_GOAL_SPEED_H       0x2F
#define FEETECH_ADDR_LOCK               0x30
#define FEETECH_ADDR_PRESENT_POSITION_L 0x38
#define FEETECH_ADDR_PRESENT_POSITION_H 0x39
#define FEETECH_ADDR_PRESENT_SPEED_L    0x3A
#define FEETECH_ADDR_PRESENT_SPEED_H    0x3B
#define FEETECH_ADDR_PRESENT_LOAD_L     0x3C
#define FEETECH_ADDR_PRESENT_LOAD_H     0x3D
#define FEETECH_ADDR_PRESENT_VOLTAGE    0x3E
#define FEETECH_ADDR_PRESENT_TEMP       0x3F
#define FEETECH_ADDR_REGISTERED         0x40
#define FEETECH_ADDR_MOVING             0x42
#define FEETECH_ADDR_PRESENT_CURRENT_L  0x45
#define FEETECH_ADDR_PRESENT_CURRENT_H  0x46

/* Error flags (in status packet) */
#define FEETECH_ERROR_VOLTAGE           0x01
#define FEETECH_ERROR_ANGLE_LIMIT       0x02
#define FEETECH_ERROR_OVERHEAT          0x04
#define FEETECH_ERROR_RANGE             0x08
#define FEETECH_ERROR_CHECKSUM          0x10
#define FEETECH_ERROR_OVERLOAD          0x20
#define FEETECH_ERROR_INSTRUCTION       0x40

/**
 * @brief Feetech protocol packet structure
 */
struct feetech_packet {
	uint8_t id;                             /* Servo ID (0-253, 254=broadcast) */
	uint8_t instruction;                    /* Instruction code */
	uint8_t params[FEETECH_MAX_PARAMS];     /* Parameters */
	size_t param_len;                       /* Number of parameter bytes */
	uint8_t error;                          /* Error flags (status packets only) */
};

/**
 * @brief Initialize Feetech protocol handler
 *
 * @param uart_handle Handle to initialized half-duplex UART
 * @return HAL_OK on success, error code otherwise
 */
int feetech_protocol_init(hal_uart_handle_t uart_handle);

/**
 * @brief Build a Feetech protocol packet
 *
 * @param packet Pointer to packet structure to encode
 * @param buffer Output buffer for encoded packet
 * @param buffer_size Size of output buffer
 * @return Number of bytes written to buffer, or negative error code
 */
int feetech_packet_build(const struct feetech_packet *packet, uint8_t *buffer,
                         size_t buffer_size);

/**
 * @brief Parse a received Feetech protocol packet
 *
 * @param buffer Input buffer containing packet data
 * @param buffer_len Length of input buffer
 * @param packet Output packet structure
 * @return HAL_OK on success, error code otherwise
 */
int feetech_packet_parse(const uint8_t *buffer, size_t buffer_len,
                         struct feetech_packet *packet);

/**
 * @brief Calculate Feetech checksum
 *
 * @param data Pointer to data (starting from ID byte)
 * @param len Length of data
 * @return Calculated checksum byte
 */
uint8_t feetech_calculate_checksum(const uint8_t *data, size_t len);

/**
 * @brief Send PING command to servo
 *
 * @param servo_id Servo ID (1-253)
 * @return HAL_OK on success, error code otherwise
 */
int feetech_ping(uint8_t servo_id);

/**
 * @brief Read data from servo control table
 *
 * @param servo_id Servo ID
 * @param address Control table address
 * @param data Output buffer for read data
 * @param length Number of bytes to read
 * @return Number of bytes read, or negative error code
 */
int feetech_read(uint8_t servo_id, uint8_t address, uint8_t *data, size_t length);

/**
 * @brief Write data to servo control table
 *
 * @param servo_id Servo ID
 * @param address Control table address
 * @param data Data to write
 * @param length Number of bytes to write
 * @return HAL_OK on success, error code otherwise
 */
int feetech_write(uint8_t servo_id, uint8_t address, const uint8_t *data, size_t length);

/**
 * @brief Sync write to multiple servos (broadcast)
 *
 * Writes the same parameter to multiple servos in a single transaction.
 * Critical for coordinated multi-joint motion.
 *
 * @param address Control table address
 * @param data_length Length of data per servo
 * @param servo_ids Array of servo IDs
 * @param servo_data Array of data for each servo (servo_count * data_length bytes)
 * @param servo_count Number of servos
 * @return HAL_OK on success, error code otherwise
 */
int feetech_sync_write(uint8_t address, size_t data_length,
                       const uint8_t *servo_ids, const uint8_t *servo_data,
                       size_t servo_count);

/**
 * @brief Read 16-bit value from control table
 *
 * Helper function for reading 2-byte values (positions, speeds, etc.)
 *
 * @param servo_id Servo ID
 * @param address Control table address (must be L byte)
 * @param value Output value
 * @return HAL_OK on success, error code otherwise
 */
int feetech_read_word(uint8_t servo_id, uint8_t address, uint16_t *value);

/**
 * @brief Write 16-bit value to control table
 *
 * Helper function for writing 2-byte values.
 *
 * @param servo_id Servo ID
 * @param address Control table address (must be L byte)
 * @param value Value to write
 * @return HAL_OK on success, error code otherwise
 */
int feetech_write_word(uint8_t servo_id, uint8_t address, uint16_t value);

#endif /* OCTROBOT_FEETECH_PROTOCOL_H */
