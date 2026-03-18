/*
 * OctroBot Robot Arm Firmware - Flash HAL Implementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "hal_flash.h"

LOG_MODULE_REGISTER(hal_flash, LOG_LEVEL_INF);

/* NVS file system instance */
static struct nvs_fs nvs;

/* Initialization flag */
static bool initialized = false;

/* Storage partition device */
#define STORAGE_PARTITION		storage_partition
#define STORAGE_PARTITION_ID	FIXED_PARTITION_ID(STORAGE_PARTITION)

int hal_flash_init(void)
{
	int ret;
	struct flash_pages_info info;
	
	if (initialized) {
		LOG_WRN("Flash already initialized");
		return HAL_OK;
	}
	
	LOG_INF("Initializing flash storage...");
	
	/* Get flash device from storage partition */
	nvs.flash_device = FIXED_PARTITION_DEVICE(STORAGE_PARTITION);
	if (!device_is_ready(nvs.flash_device)) {
		LOG_ERR("Flash device not ready");
		return HAL_ERROR;
	}
	
	/* Get partition offset and size */
	nvs.offset = FIXED_PARTITION_OFFSET(STORAGE_PARTITION);
	
	/* Get flash page/sector info */
	ret = flash_get_page_info_by_offs(nvs.flash_device, nvs.offset, &info);
	if (ret) {
		LOG_ERR("Failed to get flash page info: %d", ret);
		return HAL_ERROR;
	}
	
	nvs.sector_size = info.size;
	nvs.sector_count = FIXED_PARTITION_SIZE(STORAGE_PARTITION) / info.size;
	
	LOG_INF("NVS partition: offset=0x%x, size=%u bytes, sector_size=%u, sector_count=%u",
	        (unsigned int)nvs.offset,
	        (unsigned int)FIXED_PARTITION_SIZE(STORAGE_PARTITION),
	        nvs.sector_size,
	        nvs.sector_count);
	
	/* Mount NVS file system */
	ret = nvs_mount(&nvs);
	if (ret) {
		LOG_ERR("Failed to mount NVS: %d", ret);
		return HAL_ERROR;
	}
	
	initialized = true;
	LOG_INF("Flash storage initialized successfully");
	
	return HAL_OK;
}

int hal_flash_write(const char *key, const void *data, size_t len)
{
	int ret;
	
	if (!initialized) {
		LOG_ERR("Flash not initialized");
		return HAL_ERROR;
	}
	
	if (!key || !data || len == 0) {
		return HAL_INVALID;
	}
	
	/* NVS requires numeric IDs, so we hash the string key */
	uint16_t id = 0;
	for (size_t i = 0; key[i] != '\0'; i++) {
		id = (id << 1) ^ key[i];
	}
	
	LOG_DBG("Writing %zu bytes to key '%s' (id=%u)", len, key, id);
	
	ret = nvs_write(&nvs, id, data, len);
	if (ret < 0) {
		LOG_ERR("Failed to write to flash: %d", ret);
		return HAL_ERROR;
	}
	
	LOG_DBG("Successfully wrote %d bytes", ret);
	return HAL_OK;
}

int hal_flash_read(const char *key, void *data, size_t len)
{
	int ret;
	
	if (!initialized) {
		LOG_ERR("Flash not initialized");
		return HAL_ERROR;
	}
	
	if (!key || !data || len == 0) {
		return HAL_INVALID;
	}
	
	/* Hash string key to numeric ID */
	uint16_t id = 0;
	for (size_t i = 0; key[i] != '\0'; i++) {
		id = (id << 1) ^ key[i];
	}
	
	LOG_DBG("Reading from key '%s' (id=%u)", key, id);
	
	ret = nvs_read(&nvs, id, data, len);
	if (ret < 0) {
		if (ret == -ENOENT) {
			LOG_DBG("Key '%s' not found", key);
		} else {
			LOG_ERR("Failed to read from flash: %d", ret);
		}
		return HAL_ERROR;
	}
	
	LOG_DBG("Successfully read %d bytes", ret);
	return ret;
}

int hal_flash_delete(const char *key)
{
	int ret;
	
	if (!initialized) {
		LOG_ERR("Flash not initialized");
		return HAL_ERROR;
	}
	
	if (!key) {
		return HAL_INVALID;
	}
	
	/* Hash string key to numeric ID */
	uint16_t id = 0;
	for (size_t i = 0; key[i] != '\0'; i++) {
		id = (id << 1) ^ key[i];
	}
	
	LOG_DBG("Deleting key '%s' (id=%u)", key, id);
	
	ret = nvs_delete(&nvs, id);
	if (ret < 0) {
		if (ret == -ENOENT) {
			LOG_DBG("Key '%s' not found", key);
		} else {
			LOG_ERR("Failed to delete from flash: %d", ret);
		}
		return HAL_ERROR;
	}
	
	LOG_DBG("Successfully deleted key");
	return HAL_OK;
}

bool hal_flash_exists(const char *key)
{
	uint8_t dummy;
	int ret = hal_flash_read(key, &dummy, 1);
	return (ret >= 0);
}

int hal_flash_get_size(const char *key)
{
	if (!initialized) {
		LOG_ERR("Flash not initialized");
		return HAL_ERROR;
	}
	
	if (!key) {
		return HAL_INVALID;
	}
	
	/* Hash string key to numeric ID */
	uint16_t id = 0;
	for (size_t i = 0; key[i] != '\0'; i++) {
		id = (id << 1) ^ key[i];
	}
	
	/* Read with NULL buffer to get size */
	int ret = nvs_read(&nvs, id, NULL, 0);
	if (ret < 0) {
		return HAL_ERROR;
	}
	
	return ret;
}

int hal_flash_get_free_space(void)
{
	if (!initialized) {
		LOG_ERR("Flash not initialized");
		return HAL_ERROR;
	}
	
	/* NVS doesn't provide a direct API for free space */
	/* Return partition size as approximate free space */
	/* In practice, wear leveling reduces usable space */
	return nvs.sector_count * nvs.sector_size / 2;  /* Estimate ~50% usable */
}
