/*
 * OctroBot Robot Arm Firmware - Timer Utilities Implementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "hal_timer.h"

LOG_MODULE_REGISTER(hal_timer, LOG_LEVEL_INF);

/* Cached cycle frequency for faster conversion */
static uint32_t cycles_per_us = 0;
static uint32_t cycles_per_ms = 0;

int hal_timer_init(void)
{
	uint32_t freq_hz = sys_clock_hw_cycles_per_sec();
	
	cycles_per_us = freq_hz / 1000000;
	cycles_per_ms = freq_hz / 1000;
	
	LOG_INF("Timer initialized: %u Hz (%u cycles/us, %u cycles/ms)",
	        freq_hz, cycles_per_us, cycles_per_ms);
	
	return HAL_OK;
}

uint64_t hal_timer_get_us(void)
{
	if (cycles_per_us == 0) {
		/* Fallback if not initialized */
		cycles_per_us = sys_clock_hw_cycles_per_sec() / 1000000;
	}
	
	uint64_t cycles = k_cycle_get_64();
	return cycles / cycles_per_us;
}

uint64_t hal_timer_get_ms(void)
{
	return k_uptime_get();
}

void hal_timer_delay_us(uint32_t us)
{
	if (us == 0) {
		return;
	}
	
	if (cycles_per_us == 0) {
		cycles_per_us = sys_clock_hw_cycles_per_sec() / 1000000;
	}
	
	uint64_t start = k_cycle_get_64();
	uint64_t target_cycles = (uint64_t)us * cycles_per_us;
	
	while ((k_cycle_get_64() - start) < target_cycles) {
		/* Busy wait */
	}
}

void hal_timer_delay_ms(uint32_t ms)
{
	k_msleep(ms);
}
