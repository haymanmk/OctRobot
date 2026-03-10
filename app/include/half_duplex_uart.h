/*
 * OctroBot Robot Arm Firmware - Half-Duplex UART Driver
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Half-duplex UART driver for Feetech servo bus communication.
 * Handles TX/RX direction switching for RS-485-style communication.
 */

#ifndef OCTROBOT_HALF_DUPLEX_UART_H
#define OCTROBOT_HALF_DUPLEX_UART_H

#include "hal_types.h"

/**
 * @brief Half-duplex UART configuration
 */
struct half_duplex_uart_config {
	const char *device_name;        /* Optional Zephyr UART device name (e.g., "UART_1") */
	uint32_t baudrate;              /* Baud rate (e.g., 1000000 for 1 Mbps) */
	uint32_t tx_timeout_ms;         /* Transmit timeout in milliseconds */
	uint32_t rx_timeout_ms;         /* Receive timeout in milliseconds */
};

/**
 * @brief Initialize half-duplex UART
 *
 * @param config Pointer to configuration structure
 * @return hal_uart_handle_t Handle to UART instance, or NULL on error
 */
hal_uart_handle_t half_duplex_uart_init(const struct half_duplex_uart_config *config);

/**
 * @brief Deinitialize half-duplex UART
 *
 * @param handle UART handle
 * @return HAL_OK on success, error code otherwise
 */
int half_duplex_uart_deinit(hal_uart_handle_t handle);

/**
 * @brief Transmit data (blocking with timeout)
 *
 * Automatically switches to TX mode, sends data, waits for completion,
 * then switches back to RX mode.
 *
 * @param handle UART handle
 * @param data Pointer to data buffer
 * @param len Length of data in bytes
 * @return Number of bytes transmitted, or negative error code
 */
int half_duplex_uart_transmit(hal_uart_handle_t handle, const uint8_t *data, size_t len);

/**
 * @brief Receive data (blocking with timeout)
 *
 * @param handle UART handle
 * @param data Pointer to receive buffer
 * @param len Maximum length to receive
 * @param timeout_ms Timeout in milliseconds (0 = use default)
 * @return Number of bytes received, or negative error code
 */
int half_duplex_uart_receive(hal_uart_handle_t handle, uint8_t *data, size_t len,
                              uint32_t timeout_ms);

/**
 * @brief Flush RX buffer
 *
 * @param handle UART handle
 * @return HAL_OK on success, error code otherwise
 */
int half_duplex_uart_flush_rx(hal_uart_handle_t handle);

/**
 * @brief Get number of bytes available in RX buffer
 *
 * @param handle UART handle
 * @return Number of bytes available, or negative error code
 */
int half_duplex_uart_rx_available(hal_uart_handle_t handle);

#endif /* OCTROBOT_HALF_DUPLEX_UART_H */
