/*
 * OctroBot Robot Arm Firmware - Feetech SCS/STS Protocol
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Protocol implementation for Feetech Serial Control System (SCS) and
 * Serial Torque Servo (STS) servos.
 *
 * Packet Format:
 *   [0xFF] [0xFF] [ID] [LENGTH] [INSTRUCTION] [PARAMS...] [CHECKSUM]
 *
 * Checksum: ~(ID + LENGTH + INSTRUCTION + PARAMS) & 0xFF
 */

#ifndef OCTROBOT_FEETECH_PROTOCOL_H
#define OCTROBOT_FEETECH_PROTOCOL_H

#include "hal_types.h"
#include "half_duplex_uart.h"

/* Protocol constants */
#define FEETECH_HEADER_1          0xFF
#define FEETECH_HEADER_2          0xFF
#define FEETECH_BROADCAST_ID      0xFE
#define FEETECH_MIN_PACKET_SIZE   6

/* Instruction set */
#define FEETECH_INST_PING         0x01
#define FEETECH_INST_READ         0x02
#define FEETECH_INST_WRITE        0x03
#define FEETECH_INST_REG_WRITE    0x04
#define FEETECH_INST_ACTION       0x05
#define FEETECH_INST_SYNC_WRITE   0x83
#define FEETECH_INST_SYNC_READ    0x84

/* Common register addresses (STS series) */
#define FEETECH_REG_ID                    0x05
#define FEETECH_REG_BAUD_RATE             0x06
#define FEETECH_REG_RETURN_DELAY          0x07
#define FEETECH_REG_RESPONSE_STATUS       0x08
#define FEETECH_REG_MIN_ANGLE_LIMIT_L     0x09
#define FEETECH_REG_MIN_ANGLE_LIMIT_H     0x0A
#define FEETECH_REG_MAX_ANGLE_LIMIT_L     0x0B
#define FEETECH_REG_MAX_ANGLE_LIMIT_H     0x0C
#define FEETECH_REG_MAX_TEMPERATURE       0x0D
#define FEETECH_REG_MAX_VOLTAGE           0x0E
#define FEETECH_REG_MIN_VOLTAGE           0x0F
#define FEETECH_REG_MAX_TORQUE_L          0x10
#define FEETECH_REG_MAX_TORQUE_H          0x11
#define FEETECH_REG_UNLOAD_CONDITION      0x13
#define FEETECH_REG_LED_ALARM             0x14
#define FEETECH_REG_P_COEFFICIENT         0x15
#define FEETECH_REG_D_COEFFICIENT         0x16
#define FEETECH_REG_I_COEFFICIENT         0x17
#define FEETECH_REG_MINIMUM_STARTUP_FORCE_L 0x18
#define FEETECH_REG_MINIMUM_STARTUP_FORCE_H 0x19
#define FEETECH_REG_TORQUE_ENABLE         0x28
#define FEETECH_REG_ACCELERATION          0x29
#define FEETECH_REG_GOAL_POSITION_L       0x2A
#define FEETECH_REG_GOAL_POSITION_H       0x2B
#define FEETECH_REG_GOAL_TIME_L           0x2C
#define FEETECH_REG_GOAL_TIME_H           0x2D
#define FEETECH_REG_GOAL_SPEED_L          0x2E
#define FEETECH_REG_GOAL_SPEED_H          0x2F
#define FEETECH_REG_LOCK                  0x37
#define FEETECH_REG_PRESENT_POSITION_L    0x38
#define FEETECH_REG_PRESENT_POSITION_H    0x39
#define FEETECH_REG_PRESENT_SPEED_L       0x3A
#define FEETECH_REG_PRESENT_SPEED_H       0x3B
#define FEETECH_REG_PRESENT_LOAD_L        0x3C
#define FEETECH_REG_PRESENT_LOAD_H        0x3D
#define FEETECH_REG_PRESENT_VOLTAGE       0x3E
#define FEETECH_REG_PRESENT_TEMPERATURE   0x3F
#define FEETECH_REG_REGISTERED            0x40
#define FEETECH_REG_MOVING                0x42
#define FEETECH_REG_VIR_POSITION_L        0x46
#define FEETECH_REG_VIR_POSITION_H        0x47
#define FEETECH_REG_CURRENT_L             0x45
#define FEETECH_REG_CURRENT_H             0x69

/* Error codes in response packets */
#define FEETECH_ERROR_VOLTAGE       (1 << 0)
#define FEETECH_ERROR_ANGLE_LIMIT   (1 << 1)
#define FEETECH_ERROR_OVERHEAT      (1 << 2)
#define FEETECH_ERROR_RANGE         (1 << 3)
#define FEETECH_ERROR_CHECKSUM      (1 << 4)
#define FEETECH_ERROR_OVERLOAD      (1 << 5)
#define FEETECH_ERROR_INSTRUCTION   (1 << 6)

/* Maximum packet size */
#define FEETECH_MAX_PACKET_SIZE   256

/**
 * @brief Feetech packet structure
 */
struct feetech_packet {
	uint8_t id;
	uint8_t instruction;
	uint8_t error;
	uint8_t parameters[FEETECH_MAX_PACKET_SIZE];
	uint8_t param_length;
};

/**
 * @brief Calculate checksum for Feetech packet
 *
 * @param id Servo ID
 * @param length Packet length (instruction + params + 1)
 * @param instruction Instruction byte
 * @param params Parameter bytes
 * @param param_len Number of parameter bytes
 * @return Calculated checksum
 */
uint8_t feetech_protocol_calculate_checksum(uint8_t id, uint8_t length, uint8_t instruction,
                                    const uint8_t *params, uint8_t param_len);

/**
 * @brief Build a Feetech instruction packet
 *
 * @param packet Pointer to packet structure
 * @param buffer Output buffer for raw packet bytes
 * @param buffer_size Size of output buffer
 * @return Number of bytes written to buffer, or negative error code
 */
int feetech_protocol_build_packet(const struct feetech_packet *packet, uint8_t *buffer,
                         size_t buffer_size);

/**
 * @brief Parse a Feetech response packet
 *
 * @param buffer Input buffer containing raw packet bytes
 * @param buffer_len Length of input buffer
 * @param packet Pointer to packet structure to fill
 * @return HAL_OK on success, error code otherwise
 */
int feetech_protocol_parse_packet(const uint8_t *buffer, size_t buffer_len,
                         struct feetech_packet *packet);

/**
 * @brief Send a ping to servo and wait for response
 *
 * @param uart UART handle
 * @param id Servo ID
 * @return HAL_OK if servo responds, error code otherwise
 */
int feetech_protocol_ping(hal_uart_handle_t uart, uint8_t id);

/**
 * @brief Read data from servo register
 *
 * @param uart UART handle
 * @param id Servo ID
 * @param reg_addr Register address
 * @param length Number of bytes to read
 * @param data Output buffer for read data
 * @return HAL_OK on success, error code otherwise
 */
int feetech_protocol_read(hal_uart_handle_t uart, uint8_t id, uint8_t reg_addr,
                 uint8_t length, uint8_t *data);

/**
 * @brief Write data to servo register
 *
 * @param uart UART handle
 * @param id Servo ID
 * @param reg_addr Register address
 * @param data Data to write
 * @param length Number of bytes to write
 * @return HAL_OK on success, error code otherwise
 */
int feetech_protocol_write(hal_uart_handle_t uart, uint8_t id, uint8_t reg_addr,
                  const uint8_t *data, uint8_t length);

/**
 * @brief Sync write to multiple servos (single register)
 *
 * @param uart UART handle
 * @param reg_addr Starting register address
 * @param data_len Length of data per servo
 * @param servo_ids Array of servo IDs
 * @param servo_data Array of data for each servo (servo_count * data_len bytes)
 * @param servo_count Number of servos
 * @return HAL_OK on success, error code otherwise
 */
int feetech_protocol_sync_write(hal_uart_handle_t uart, uint8_t reg_addr, uint8_t data_len,
                       const uint8_t *servo_ids, const uint8_t *servo_data,
                       uint8_t servo_count);

/**
 * @brief Read 16-bit value from servo (little-endian)
 *
 * @param uart UART handle
 * @param id Servo ID
 * @param reg_addr Register address (low byte)
 * @param value Pointer to store 16-bit value
 * @return HAL_OK on success, error code otherwise
 */
int feetech_protocol_read_word(hal_uart_handle_t uart, uint8_t id, uint8_t reg_addr,
                      uint16_t *value);

/**
 * @brief Write 16-bit value to servo (little-endian)
 *
 * @param uart UART handle
 * @param id Servo ID
 * @param reg_addr Register address (low byte)
 * @param value 16-bit value to write
 * @return HAL_OK on success, error code otherwise
 */
int feetech_protocol_write_word(hal_uart_handle_t uart, uint8_t id, uint8_t reg_addr,
                       uint16_t value);

#endif /* OCTROBOT_FEETECH_PROTOCOL_H */
