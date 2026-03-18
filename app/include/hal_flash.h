/*
 * OctroBot Robot Arm Firmware - Flash HAL
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Flash storage abstraction layer for persistent data.
 * Uses Zephyr NVS (Non-Volatile Storage) subsystem.
 */

#ifndef OCTROBOT_HAL_FLASH_H
#define OCTROBOT_HAL_FLASH_H

#include "hal_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize flash storage subsystem
 * 
 * This function initializes the NVS subsystem and mounts the storage partition.
 * Should be called once during system initialization.
 * 
 * @return HAL_OK on success, error code otherwise
 */
int hal_flash_init(void);

/**
 * @brief Write data to flash storage
 * 
 * Writes data associated with a key to non-volatile storage.
 * If the key already exists, it will be overwritten.
 * 
 * Educational Note:
 * Flash wear leveling is handled automatically by the NVS subsystem.
 * The ESP32 flash has ~100k erase cycles per sector.
 * 
 * @param key Unique identifier for the data (e.g., "demo0", "demo1")
 * @param data Pointer to data buffer to write
 * @param len Length of data in bytes (max 8KB recommended)
 * @return HAL_OK on success, error code otherwise
 */
int hal_flash_write(const char *key, const void *data, size_t len);

/**
 * @brief Read data from flash storage
 * 
 * Reads data associated with a key from non-volatile storage.
 * 
 * @param key Unique identifier for the data
 * @param data Pointer to buffer to receive data
 * @param len Length of buffer (must be >= stored data size)
 * @return Number of bytes read on success, error code otherwise
 */
int hal_flash_read(const char *key, void *data, size_t len);

/**
 * @brief Delete data from flash storage
 * 
 * Removes the data associated with a key from storage.
 * 
 * @param key Unique identifier for the data
 * @return HAL_OK on success, HAL_ERROR if key not found
 */
int hal_flash_delete(const char *key);

/**
 * @brief Check if a key exists in flash storage
 * 
 * @param key Unique identifier to check
 * @return true if key exists, false otherwise
 */
bool hal_flash_exists(const char *key);

/**
 * @brief Get the size of stored data
 * 
 * Returns the size of data associated with a key without reading it.
 * Useful for allocating buffers before reading.
 * 
 * @param key Unique identifier for the data
 * @return Size in bytes if key exists, HAL_ERROR otherwise
 */
int hal_flash_get_size(const char *key);

/**
 * @brief Get free space available in flash storage
 * 
 * Returns the approximate free space available in the NVS partition.
 * Note: Actual usable space may be less due to wear leveling overhead.
 * 
 * @return Free space in bytes, or HAL_ERROR on failure
 */
int hal_flash_get_free_space(void);

#ifdef __cplusplus
}
#endif

#endif /* OCTROBOT_HAL_FLASH_H */
