/*
 * OctroBot - HAL Flash Stub for Unit Tests
 * SPDX-License-Identifier: Apache-2.0
 *
 * Provides stub implementations of hal_flash functions so that
 * robot_geometry.c can link on native_posix without NVS.
 * All reads return errors, forcing robot_geometry_load_from_flash()
 * to fall back to factory defaults.
 */

#include "hal_flash.h"

int hal_flash_init(void)
{
	return HAL_OK;
}

int hal_flash_write(const char *key, const void *data, size_t len)
{
	(void)key;
	(void)data;
	(void)len;
	return HAL_ERROR;
}

int hal_flash_read(const char *key, void *data, size_t len)
{
	(void)key;
	(void)data;
	(void)len;
	return HAL_ERROR;  /* Forces factory defaults fallback */
}

int hal_flash_delete(const char *key)
{
	(void)key;
	return HAL_ERROR;
}

bool hal_flash_exists(const char *key)
{
	(void)key;
	return false;
}
