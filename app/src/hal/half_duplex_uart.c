/*
 * OctroBot Robot Arm Firmware - Half-Duplex UART Driver Implementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "half_duplex_uart.h"

LOG_MODULE_REGISTER(half_duplex_uart, LOG_LEVEL_WRN);

#define RX_RING_BUFFER_SIZE 256

/* Half-duplex UART instance structure */
struct hal_uart_handle_s {
	const struct device *dev;
	uint32_t baudrate;
	uint32_t tx_timeout_ms;
	uint32_t rx_timeout_ms;
	
	/* RX ring buffer */
	uint8_t rx_buffer[RX_RING_BUFFER_SIZE];
	volatile size_t rx_head;
	volatile size_t rx_tail;
	
	/* Synchronization */
	struct k_sem tx_done_sem;
	struct k_mutex tx_mutex;
	
	bool initialized;
};

/* Static instance (single UART for servos) */
static struct hal_uart_handle_s uart_instance;

static const struct device *resolve_uart_device(const char *device_name)
{
	const struct device *dev = NULL;

	if (device_name) {
		dev = device_get_binding(device_name);
		if (dev) {
			return dev;
		}
		LOG_WRN("UART device '%s' not found, trying devicetree fallbacks", device_name);
	}

#if DT_HAS_ALIAS(servo_uart) && DT_NODE_HAS_STATUS(DT_ALIAS(servo_uart), okay)
	dev = DEVICE_DT_GET(DT_ALIAS(servo_uart));
	if (device_is_ready(dev)) {
		return dev;
	}
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart1), okay)
	dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
	if (device_is_ready(dev)) {
		return dev;
	}
#endif

	return NULL;
}

/* UART callback for interrupt-driven RX */
static void uart_isr_callback(const struct device *dev, void *user_data)
{
	struct hal_uart_handle_s *handle = (struct hal_uart_handle_s *)user_data;
	
	if (!uart_irq_update(dev)) {
		return;
	}
	
	/* Handle RX data */
	if (uart_irq_rx_ready(dev)) {
		uint8_t byte;
		while (uart_fifo_read(dev, &byte, 1) == 1) {
			size_t next_head = (handle->rx_head + 1) % RX_RING_BUFFER_SIZE;
			
			/* Drop byte if buffer full */
			if (next_head != handle->rx_tail) {
				handle->rx_buffer[handle->rx_head] = byte;
				handle->rx_head = next_head;
			} else {
				LOG_WRN("RX buffer overflow, dropping byte: 0x%02X", byte);
			}
		}
	}
	
	/* Handle TX complete */
	if (uart_irq_tx_complete(dev)) {
		uart_irq_tx_disable(dev);
		k_sem_give(&handle->tx_done_sem);
	}
}

hal_uart_handle_t half_duplex_uart_init(const struct half_duplex_uart_config *config)
{
	if (!config) {
		LOG_ERR("Invalid configuration");
		return NULL;
	}
	
	struct hal_uart_handle_s *handle = &uart_instance;
	
	if (handle->initialized) {
		LOG_WRN("UART already initialized");
		return handle;
	}
	
	/* Resolve UART from explicit device name or devicetree fallback. */
	handle->dev = resolve_uart_device(config->device_name);
	if (!handle->dev) {
		LOG_ERR("No usable UART device found for half-duplex bus");
		return NULL;
	}
	
	if (!device_is_ready(handle->dev)) {
		LOG_ERR("UART device not ready");
		return NULL;
	}
	
	/* Configure UART */
	struct uart_config uart_cfg = {
		.baudrate = config->baudrate,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};
	
	int ret = uart_configure(handle->dev, &uart_cfg);
	if (ret != 0) {
		LOG_ERR("Failed to configure UART: %d", ret);
		return NULL;
	}
	
	/* Initialize synchronization primitives */
	k_sem_init(&handle->tx_done_sem, 0, 1);
	k_mutex_init(&handle->tx_mutex);
	
	/* Initialize RX ring buffer */
	handle->rx_head = 0;
	handle->rx_tail = 0;
	
	/* Store configuration */
	handle->baudrate = config->baudrate;
	handle->tx_timeout_ms = config->tx_timeout_ms;
	handle->rx_timeout_ms = config->rx_timeout_ms;
	
	/* Set up interrupt-driven RX */
	uart_irq_callback_user_data_set(handle->dev, uart_isr_callback, handle);
	uart_irq_rx_enable(handle->dev);
	
	handle->initialized = true;
	
	LOG_INF("Half-duplex UART initialized: %s @ %u baud",
		handle->dev->name, config->baudrate);
	
	return handle;
}

int half_duplex_uart_deinit(hal_uart_handle_t handle)
{
	if (!handle || !handle->initialized) {
		return HAL_INVALID;
	}
	
	uart_irq_rx_disable(handle->dev);
	uart_irq_tx_disable(handle->dev);
	
	handle->initialized = false;
	
	LOG_INF("Half-duplex UART deinitialized");
	
	return HAL_OK;
}

int half_duplex_uart_transmit(hal_uart_handle_t handle, const uint8_t *data, size_t len)
{
	if (!handle || !handle->initialized || !data || len == 0) {
		return HAL_INVALID;
	}
	
	/* Lock TX to prevent concurrent transmissions */
	k_mutex_lock(&handle->tx_mutex, K_FOREVER);
	
	/* Clear RX buffer before transmission (echoed bytes on half-duplex bus) */
	half_duplex_uart_flush_rx(handle);
	
	/* Transmit data using polling mode (simpler for short packets) */
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(handle->dev, data[i]);
	}
	
	/* Wait for TX to complete (hardware FIFO empty) */
	/* Note: ESP32 UART doesn't have reliable TX-complete IRQ in all cases,
	 * so we add a small delay based on baud rate */
	uint32_t bit_time_us = (1000000 * 10) / handle->baudrate; /* 10 bits per byte */
	uint32_t tx_time_us = bit_time_us * len + 100; /* Add 100us margin */
	k_usleep(tx_time_us);
	
	k_mutex_unlock(&handle->tx_mutex);
	
	LOG_DBG("TX %zu bytes", len);
	
	return (int)len;
}

int half_duplex_uart_receive(hal_uart_handle_t handle, uint8_t *data, size_t len,
                              uint32_t timeout_ms)
{
	if (!handle || !handle->initialized || !data || len == 0) {
		return HAL_INVALID;
	}
	
	if (timeout_ms == 0) {
		timeout_ms = handle->rx_timeout_ms;
	}
	
	size_t received = 0;
	int64_t deadline = k_uptime_get() + timeout_ms;
	
	while (received < len) {
		/* Check for bytes in ring buffer */
		if (handle->rx_head != handle->rx_tail) {
			data[received] = handle->rx_buffer[handle->rx_tail];
			handle->rx_tail = (handle->rx_tail + 1) % RX_RING_BUFFER_SIZE;
			received++;
		} else {
			/* No data available, check timeout */
			if (k_uptime_get() >= deadline) {
				if (received > 0) {
					LOG_DBG("RX timeout, received %zu/%zu bytes", received, len);
					return (int)received;
				} else {
					return HAL_TIMEOUT;
				}
			}
			
			/* Yield to allow ISR to process incoming data */
			k_usleep(100);
		}
	}
	
	LOG_DBG("RX %zu bytes", received);
	
	return (int)received;
}

int half_duplex_uart_flush_rx(hal_uart_handle_t handle)
{
	if (!handle || !handle->initialized) {
		return HAL_INVALID;
	}
	
	/* Reset ring buffer pointers */
	handle->rx_head = 0;
	handle->rx_tail = 0;
	
	/* Drain hardware FIFO */
	uint8_t dummy;
	while (uart_fifo_read(handle->dev, &dummy, 1) == 1) {
		/* Discard */
	}
	
	return HAL_OK;
}

int half_duplex_uart_rx_available(hal_uart_handle_t handle)
{
	if (!handle || !handle->initialized) {
		return HAL_INVALID;
	}
	
	size_t head = handle->rx_head;
	size_t tail = handle->rx_tail;
	
	if (head >= tail) {
		return (int)(head - tail);
	} else {
		return (int)(RX_RING_BUFFER_SIZE - tail + head);
	}
}
