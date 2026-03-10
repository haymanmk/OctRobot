/*
 * OctroBot Robot Arm Firmware
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Main entry point */
int main(void)
{
	int ret;

	LOG_INF("===========================================");
	LOG_INF("OctroBot Robot Arm Firmware v0.1.0");
	LOG_INF("===========================================");
	LOG_INF("MCU: ESP32-PICO-D4 (M5Stack Atom Lite)");
	LOG_INF("RTOS: Zephyr RTOS v%s", KERNEL_VERSION_STRING);
	LOG_INF("Build: %s %s", __DATE__, __TIME__);
	LOG_INF("===========================================");

#if defined(CONFIG_USB_DEVICE_STACK)
	/* Enable USB CDC ACM for console output */
	const struct device *usb_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	
	if (!device_is_ready(usb_dev)) {
		LOG_ERR("USB CDC ACM device not ready");
		return -1;
	}

	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB: %d", ret);
		return ret;
	}

	LOG_INF("USB CDC ACM console enabled");
	
	/* Give USB time to enumerate */
	k_sleep(K_MSEC(1000));
#endif

	LOG_INF("System initialization complete");
	LOG_INF("6-DOF robot arm ready");

	/* Main loop - placeholder for future control loop */
	uint32_t counter = 0;
	while (1) {
		LOG_INF("Heartbeat: %u seconds", counter);
		counter++;
		k_sleep(K_SECONDS(5));
	}

	return 0;
}
