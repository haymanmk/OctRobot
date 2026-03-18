/*
 * OctroBot Robot Arm Firmware - UART ConsoleImplementation
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * @note This module implements a simple command protocol over the USB-UART console (CH340).
 * The command format is:
 *   $CMD [ARG1] [ARG2] ... \n
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>

#include "utils.h"
#include "uart_console.h"
#include "servo_control.h"
#include "hal_flash.h"

LOG_MODULE_REGISTER(uart_console, CONFIG_UART_CONSOLE_LOG_LEVEL);

#define RX_BUF_SIZE 256
#define TX_BUF_SIZE 256
#define STACK_SIZE 2048
#define PRIORITY 5
#define MAX_ARGS 10

// UART device (console via CH340 USB-UART bridge)
static const struct device *uart_dev = NULL;

// RX ring buffer
UTILS_DECLARE_RING_BUFFER(rx, uint8_t)
static rx_ring_buffer_t rx_rb;

// Initialize a thread for processing received commands
static void uart_console_thread_fn(void *parameters);
K_KERNEL_THREAD_DEFINE(uart_console_thread,
                        STACK_SIZE,
                        uart_console_thread_fn,
                        NULL, NULL, NULL,
                        PRIORITY, 0, 0);

// Initialize semaphore for new line received
K_SEM_DEFINE(new_line_sem, 0, 10);

/* Demo waypoint recording */
#define MAX_WAYPOINTS 4  /* 0-3 waypoint slots */

struct waypoint_data {
	bool valid;           /* Is this waypoint recorded? */
	float angles[6];      /* Joint angles in degrees */
};

/* On-disk demo format */
struct demo_storage {
	uint8_t waypoint_count;
	uint8_t reserved[3];       /* Padding for alignment */
	struct waypoint_data waypoints[MAX_WAYPOINTS];
};

/* RAM storage for active waypoints */
static struct demo_storage active_demo = {0};

/**
 * @brief Reply to host with a formatted message over UART.
 * @param fmt printf-style format string
 * @param ... Arguments for format string
 * @return 0 on success, negative error code on failure
 */
int uart_console_reply(const char *fmt, ...)
{
	if (!fmt || !uart_dev) {
		return -EINVAL;
	}

	char buffer[TX_BUF_SIZE];
	
	va_list args;
	va_start(args, fmt);
	int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	if (len < 0) {
		return -EINVAL;
	}
	
	/* Check for truncation */
	if (len >= sizeof(buffer)) {
		LOG_WRN("Reply message truncated: %d bytes needed, %zu available",
		        len, sizeof(buffer));
		len = sizeof(buffer) - 1;
	}

	/* Send formatted message directly via UART FIFO */
	for (int i = 0; i < len; i++) {
        uart_poll_out(uart_dev, buffer[i]);
    }

	return 0;
}

static void uart_console_callback(const struct device *dev, void *user_data)
{
    // Start processing incoming data in ISR context
    if (!uart_irq_update(dev)) {
        return;
    }

    // Check if RX data is ready
    if (!uart_irq_rx_ready(dev)) {
        return;
    }

    LOG_DBG("UART RX interrupt triggered");

    // Read bytes into ring buffer
    uint8_t byte;
    while (uart_fifo_read(dev, &byte, 1) == 1) {
        if (!rx_ring_buffer_append(&rx_rb, byte)) {
            LOG_WRN("RX ring buffer overflow, dropping oldest bytes to make room");
        }

        // Check for packet terminator
        if (byte == UART_CONSOLE_TERMINATOR) {
            // Signal main thread to process complete packet
            k_sem_give(&new_line_sem);
            // Log message for debugging
            LOG_DBG("Received complete command packet");
        }
    }
}

/**
 * @brief Split a command buffer into arguments based on whitespace.
 * @param cmd_buf Pointer to the command buffer (null-terminated)
 * @param cmd_len Length of the command buffer
 * @param argv Array of pointers to hold argument strings
 * @param max_args Maximum number of arguments to parse
 * @return Number of arguments parsed
 * @note This function modifies the input command buffer by inserting null terminators.
 */
int uart_console_split_command(const uint8_t *cmd_buf, size_t cmd_len, char *argv[], size_t max_args)
{
    size_t arg_count = 0;
    size_t i = 0;

    // Skip leading whitespace and command prefix ('$')
    while (i < cmd_len && (cmd_buf[i] == ' ' || cmd_buf[i] == '\t' || cmd_buf[i] == '$')) {
        i++;
    }

    while (i < cmd_len && arg_count < max_args) {
        // Start of argument
        argv[arg_count++] = (char *)&cmd_buf[i];

        // Find end of argument
        while (i < cmd_len && cmd_buf[i] != ' ' && cmd_buf[i] != '\t') {
            i++;
        }

        // Null-terminate the argument
        // Add terminator in place of whitespace
        ((char *)cmd_buf)[i++] = '\0';
    }

    // Replace the last argument's terminator with null in case the command is sent without a parameter (e.g., "$read\n")
    if (arg_count > 0 && argv[arg_count - 1][0] != '\0') {
        ((char *)cmd_buf)[cmd_len - 1] = '\0';
    }

    return arg_count;
}

/**
 * @brief Process a received command packet from the UART console.
 * @param cmd_buf Pointer to the command buffer (excluding terminator)
 * @param cmd_len Length of the command buffer
 * @return 0 on success, negative error code on failure
 * @note Available commands and their payload formats are shown below:
 *       - $power [1/0] \n
 *         Power on/off the robot arm (enables/disables torque on all joints)
 *         Example: $power 1 \n (Power on)
 *         Example: $power 0 \n (Power off)
 * 
 *       - $jog [JOINT_ID] [DIRECTION] [DEGREES] \n
 *         JOINT_ID: 1-6 (joint number)
 *         DIRECTION: '+' for positive, '-' for negative
 *         DEGREES: 1-180 (step size in degrees)
 *         Example: $jog 1 + 50 \n (Jog joint 1 in positive direction at 50 degrees per step)
 * 
 *       - $read [JOINT_ID]\n
 *         JOINT_ID: 1-6 (joint number, optional)
 *         Example: $read \n (Request current joint angles and status)
 *         Example: $read 2 \n (Request status of joint 2)
 * 
 *       - $set [JOINT_ID] [DEGREES] \n
 *         JOINT_ID: 0 (all joints), 1-6 (joint number)
 *         DEGREES: -180 to +180 (desired angle in degrees)
 *         Example: $set 2 90 \n (Set joint 2 to 90 degrees)
 *         Example: $set 0 45 \n (Set all joints to 45 degrees)
 * 
 *       - $set_angles [DEG1] [DEG2] [DEG3] [DEG4] [DEG5] [DEG6] \n
 *         DEG1-6: Desired angles for joints 1-6 in degrees (-180 to +180)
 *         Example: $set_angles 45 30 0 -30 -45 90 \n (Set joints to specified angles)
 * 
 *       ############################## 
 *       # Teaching and Demo Commands #
 *       ##############################
 *       During teach mode, setting joint positions via command will be forbidden to allow safe manual manipulation.
 *       - $teach [1/0] \n
 *         Enter teach mode where the user can manually move the robot arm by hand.
 *         During teach mode, a position-based soft compliance is enabled with low P gains
 *         and max torque limits to allow safe manual manipulation.
 *         Example: $teach 1 \n (Enter teach mode)
 *         Example: $teach 0 \n (Exit teach mode)
 * 
 *       - $record [POINT_ID] \n
 *         POINT_ID: 0-3 (waypoint slot to record)
 *         Example: $record 1 \n (Record current joint angles as waypoint 1)
 * 
 *       - $demo \n
 *         Playback a recorded demo sequence (Record by sending multiple $record commands, then $demo to play back)
 * @warning This function will modify the input command buffer.
 */
int uart_console_process_command(const uint8_t *cmd_buf, size_t cmd_len)
{
    // Check if command is prefixed with '$'.
    if (cmd_buf[0] != '$') {
        LOG_WRN("Invalid command format: missing '$' prefix");
        return -EINVAL;
    }

    // Parse command and payload
    // Split command into tokens (command and arguments)
    char *argv[MAX_ARGS];
    // NOTE: The uart_console_split_command function will modify the cmd_buf.
    int argc = uart_console_split_command(cmd_buf, cmd_len, argv, MAX_ARGS);
    if (argc < 1) {
        LOG_WRN("Empty command received");
        return -EINVAL;
    }
    const char *command = argv[0];
    LOG_DBG("Parsed command: %s, argc: %d", command, argc);
    int ret;
    // Handle commands
    /* Power command */
    if (strcmp(command, "power") == 0) {
        if (argc != 2) {
            LOG_WRN("Invalid 'power' command format. Expected: $power [1/0]");
            return -EINVAL;
        }
        bool power_on = !!atoi(argv[1]);
        LOG_DBG("Power command received: %s", power_on ? "ON" : "OFF");
        
        // Enable/disable torque on all joints
        for (uint8_t id = 1; id <= 6; id++) {
            ret = feetech_servo_set_torque_enable(id, power_on);
            if (ret != 0) {
                LOG_ERR("Failed to set torque for joint %d: %d", id, ret);
                return ret;
            }
        }
    }
    /* Jog command */
    else if (strcmp(command, "jog") == 0) {
        if (argc != 4) {
            LOG_WRN("Invalid 'jog' command format. Expected: $jog [JOINT_ID] [DIRECTION] [SPEED]");
            return -EINVAL;
        }
        int joint_id = atoi(argv[1]);
        char direction = argv[2][0];
        int degrees = atoi(argv[3]);
        LOG_DBG("Jog command - Joint ID: %d, Direction: %c, Degrees: %d", joint_id, direction, degrees);
        ret = uart_console_jog_joint(joint_id, direction, degrees);
        if (ret != 0) {
            LOG_ERR("Failed to jog joint: %d", ret);
            return ret;
        }
    }
    /* Read command */
    else if ((ret = strcmp(command, "read")) == 0) {
        if (argc > 2) {
            LOG_WRN("Invalid 'read' command format. Expected: $read [JOINT_ID]");
            return -EINVAL;
        }
        LOG_DBG("Read command received");

        if (argc == 2) {
            int joint_id = atoi(argv[1]);
            // Check if joint_id is valid
            if (joint_id < 1 || joint_id > 6) {
                LOG_WRN("Invalid joint ID for 'read' command: %d", joint_id);
                return -EINVAL;
            }
            LOG_DBG("Read command - Joint ID: %d", joint_id);
            struct feetech_servo_state state;
            ret = uart_console_get_servo_state(joint_id, &state);
            if (ret != 0) {
                LOG_ERR("Failed to get servo state: %d", ret);
                return ret;
            }
            ret = uart_console_send_servo_state((uint8_t)joint_id, &state);
            if (ret != 0) {
                LOG_ERR("Failed to send servo state: %d", ret);
                return ret;
            }
        } else {
            ret = uart_console_get_servo_states();
            if (ret != 0) {
                LOG_ERR("Failed to get robot state: %d", ret);
                return ret;
            }
        }
    }
    /* Set command */
    else if (strcmp(command, "set") == 0) {
        if (argc != 3) {
            LOG_WRN("Invalid 'set' command format. Expected: $set [JOINT_ID] [DEGREES]");
            return -EINVAL;
        }
        int joint_id = atoi(argv[1]);
        float angle_deg = atof(argv[2]);
        LOG_DBG("Set command - Joint ID: %d, Degrees: %.2f", joint_id, (double)angle_deg);

        if (joint_id == 0) {
            // Set all joints to the specified angle
            for (uint8_t id = 1; id <= 6; id++) {
                ret = uart_console_set_joint_position(id, angle_deg);
                if (ret != 0) {
                    LOG_ERR("Failed to set joint %d position: %d", id, ret);
                    return ret;
                }
            }
        } else {
            // Set specific joint to the specified angle
            ret = uart_console_set_joint_position(joint_id, angle_deg);
            if (ret != 0) {
                LOG_ERR("Failed to set joint position: %d", ret);
                return ret;
            }
        }
    }
    /* Set angles command */
    else if (strcmp(command, "set_angles") == 0) {
        if (argc != 7) {
            LOG_WRN("Invalid 'set_angles' command format. Expected: $set_angles [DEG1] [DEG2] [DEG3] [DEG4] [DEG5] [DEG6]");
            return -EINVAL;
        }
        for (uint8_t i = 0; i < 6; i++) {
            float angle_deg = atof(argv[i + 1]);
            LOG_DBG("Set angles command - Joint %d: %.2f degrees", i + 1, (double)angle_deg);
            ret = uart_console_set_joint_position(i + 1, angle_deg);
            if (ret != 0) {
                LOG_ERR("Failed to set joint %d position: %d", i + 1, ret);
                return ret;
            }
        }
    }
    /* Teach command */
    else if (strcmp(command, "teach") == 0) {
        if (argc != 2) {
            LOG_WRN("Invalid 'teach' command format. Expected: $teach [1/0]");
            return -EINVAL;
        }
        bool enable = !!atoi(argv[1]);
        LOG_DBG("Teach command received");
        servo_control_soft_comply(enable);
    }
    /* Record command */
    else if (strcmp(command, "record") == 0) {
        if (argc != 2) {
            LOG_WRN("Invalid 'record' command format. Expected: $record [POINT_ID]");
            return -EINVAL;
        }
        int point_id = atoi(argv[1]);
        
        if (point_id < 0 || point_id >= MAX_WAYPOINTS) {
            LOG_ERR("Invalid waypoint ID: %d (must be 0-%d)", point_id, MAX_WAYPOINTS - 1);
            uart_console_reply("ERROR: Invalid waypoint ID\n");
            return -EINVAL;
        }
        
        LOG_INF("Recording waypoint %d", point_id);
        
        /* Read current joint angles */
        for (uint8_t id = 1; id <= 6; id++) {
            uint16_t position;
            ret = feetech_servo_read_position(id, &position);
            if (ret != 0) {
                LOG_ERR("Failed to read position for joint %d: %d", id, ret);
                uart_console_reply("ERROR: Failed to read joint %d\n", id);
                return ret;
            }
            active_demo.waypoints[point_id].angles[id - 1] = FEETECH_POS_TO_DEG(position);
            LOG_DBG("  Joint %d: %.2f deg", id, (double)active_demo.waypoints[point_id].angles[id - 1]);
        }
        
        active_demo.waypoints[point_id].valid = true;
        
        /* Count total valid waypoints */
        uint8_t count = 0;
        for (int i = 0; i < MAX_WAYPOINTS; i++) {
            if (active_demo.waypoints[i].valid) {
                count++;
            }
        }
        active_demo.waypoint_count = count;
        
        /* Save to flash */
        ret = hal_flash_write("demo", &active_demo, sizeof(active_demo));
        if (ret != HAL_OK) {
            LOG_ERR("Failed to save waypoint to flash: %d", ret);
            uart_console_reply("ERROR: Flash write failed\n");
            return ret;
        }
        
        LOG_INF("Waypoint %d recorded successfully (%d total waypoints)", point_id, count);
        uart_console_reply("Waypoint %d recorded (total: %d)\n", point_id, count);
    }
    /* Demo command */
    else if (strcmp(command, "demo") == 0) {
        LOG_INF("Playing demo sequence");
        
        /* Load demo from flash */
        struct demo_storage flash_demo;
        ret = hal_flash_read("demo", &flash_demo, sizeof(flash_demo));
        if (ret < 0) {
            LOG_ERR("No demo found in flash");
            uart_console_reply("ERROR: No demo recorded\n");
            return -ENOENT;
        }
        
        if (flash_demo.waypoint_count == 0) {
            LOG_WRN("Demo has no waypoints");
            uart_console_reply("ERROR: No waypoints recorded\n");
            return -EINVAL;
        }
        
        LOG_INF("Playing %d waypoints", flash_demo.waypoint_count);
        uart_console_reply("Playing demo (%d waypoints)...\n", flash_demo.waypoint_count);
        
        /* Play each valid waypoint in sequence */
        for (int i = 0; i < MAX_WAYPOINTS; i++) {
            if (!flash_demo.waypoints[i].valid) {
                continue;
            }
            
            LOG_INF("Executing waypoint %d", i);
            
            /* Set all joint positions */
            uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
            ret = feetech_servo_sync_write_angles(ids, flash_demo.waypoints[i].angles, 6);
            if (ret != 0) {
                LOG_ERR("Failed to execute waypoint %d: %d", i, ret);
                uart_console_reply("ERROR: Failed at waypoint %d\n", i);
                return ret;
            }
            
            /* Wait for movement to complete */
            k_sleep(K_MSEC(1500));  /* 1.5 second between waypoints */
        }
        
        LOG_INF("Demo playback complete");
        uart_console_reply("Demo complete\n");
    }
    else {
        LOG_WRN("Unknown command: %s, size: %zu, ret: %d", command, sizeof(command), ret);
        // Hexdump command buffer for debugging
        LOG_HEXDUMP_DBG(cmd_buf, cmd_len, "Received command buffer");
        return -EINVAL;
    }

    return 0;
}

void uart_console_thread_fn(void *parameters)
{
    while (1) {
        // Wait for a complete command packet to be received (terminated by UART_CONSOLE_TERMINATOR)
        k_sem_take(&new_line_sem, K_FOREVER);
        // Check if we have a complete command packet in the ring buffer
        if (!rx_ring_buffer_contains(&rx_rb, UART_CONSOLE_TERMINATOR)) {
            // No complete packet yet, release CPU and wait for more data
            k_sleep(K_MSEC(10));
            continue;
        }
        // Extract command packet from ring buffer
        uint8_t cmd_buf[RX_BUF_SIZE];
        // Extract bytes until the terminator is found where the terminator is included in the output buffer
        size_t cmd_len = rx_ring_buffer_extract_until(&rx_rb, cmd_buf,
                            sizeof(cmd_buf), UART_CONSOLE_TERMINATOR);
        if (cmd_len == 0) {
            LOG_WRN("Received empty command packet");
            continue;
        }

        LOG_DBG("Processing command of length %zu", cmd_len);
        LOG_DBG("Received command: %.*s", (int)cmd_len, cmd_buf);

        uart_console_process_command(cmd_buf, cmd_len);

        k_sleep(K_MSEC(10)); // Sleep to yield CPU
    }
}

int uart_console_init(void)
{
    LOG_INF("Initializing UART console...");

    /* Get console UART device (UART0 via CH340 USB-UART bridge) */
    uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("Console UART device not ready");
        return -ENODEV;
    }

    // Set up interrupt-driven RX
    if (uart_irq_callback_user_data_set(uart_dev, &uart_console_callback, NULL) < 0) {
        LOG_ERR("Failed to set UART callback");
        return -EIO;
    }

    // Initialize RX ring buffer
    rx_ring_buffer_init(&rx_rb);

    // Enable RX interrupts
    uart_irq_rx_enable(uart_dev);

    LOG_INF("UART console initialized successfully");
    LOG_INF("System logging is active on this UART");

    return 0;
}

int uart_console_get_servo_state(uint8_t joint_id, struct feetech_servo_state *state)
{
    if (!state) {
        return -EINVAL;
    }

    return feetech_servo_read_state(joint_id, state);
}

int uart_console_send_servo_state(uint8_t joint_id, const struct feetech_servo_state *state)
{
    if (!state) {
        return -EINVAL;
    }

    // Format the servo state into a packet and send it over UART
    // For simplicity, we will send a CSV line in the format (split by whitespace):
    // "$read JOINT_ID TEMP_C VOLTAGE_V LOAD_PCT ANGLE_DEG IS_MOVING IS_TORQUE_ENABLED ERROR\n"
    // Convert ticks to degrees for angle reporting
    float angle_deg = FEETECH_POS_TO_DEG(state->present_position);

    return uart_console_reply("$read %d %d %d.%d %d %d %d %d %d\n",
                              joint_id,
                              state->present_temperature,
                              state->present_voltage / 10,
                              state->present_voltage % 10,
                              state->present_load,
                              (int)(angle_deg * 100), // Send angle in hundredths of degrees
                              state->is_moving,
                              state->is_torque_enabled,
                              state->error);
}

int uart_console_get_servo_states()
{
    // Loop through all joints, get their states, and report back to host
    for (uint8_t i = 0; i < 6; i++) {
        struct feetech_servo_state state;
        int ret = uart_console_get_servo_state(i + 1, &state);
        if (ret != 0) {
            LOG_ERR("Failed to get state for joint %d: %d", i +1, ret);
            return ret;
        }
        ret = uart_console_send_servo_state(i + 1, &state);
        if (ret != 0) {
            LOG_ERR("Failed to send state for joint %d: %d", i +1, ret);
            return ret;
        }
    }
    return 0;
}

int uart_console_set_joint_position(uint8_t joint_id, float angle_deg)
{
    if (joint_id < 1 || joint_id > 6) {
        LOG_WRN("Invalid joint ID: %d (must be 1-6)", joint_id);
        return -EINVAL;
    }
    if (angle_deg < -180.0f || angle_deg > 180.0f) {
        LOG_WRN("Invalid angle: %.2f (must be -180 to +180)", (double)angle_deg);
        return -EINVAL;
    }

    // Convert desired angle to servo ticks
    uint16_t target_pos = FEETECH_DEG_TO_POS(angle_deg);

    // Send command to servo to move to target position
    int ret = feetech_servo_set_goal_position(joint_id, target_pos);
    if (ret != 0) {
        LOG_ERR("Failed to set position for joint %d: %d", joint_id, ret);
        return ret;
    }

    LOG_DBG("Set joint %d to %.2f degrees (ticks: %d)", joint_id, (double)angle_deg, target_pos);
    return 0;
}

int uart_console_jog_joint(uint8_t joint_id, char direction, uint8_t step_deg)
{
    if (joint_id < 1 || joint_id > 6) {
        LOG_WRN("Invalid joint ID: %d (must be 1-6)", joint_id);
        return -EINVAL;
    }
    if (direction != '+' && direction != '-') {
        LOG_WRN("Invalid direction: %c (must be '+' or '-')", direction);
        return -EINVAL;
    }
    if (step_deg < 1 || step_deg > 180) {
        LOG_WRN("Invalid step size: %d (must be 1-180)", step_deg);
        return -EINVAL;
    }

    int8_t dir_multiplier = (direction == '+') ? 1 : -1;

    // Read current position of the joint
    uint16_t current_pos;
    int ret = feetech_servo_read_position(joint_id, &current_pos);
    if (ret != 0) {
        LOG_ERR("Failed to read current position for joint %d: %d", joint_id, ret);
        return ret;
    }
    float current_angle = FEETECH_POS_TO_DEG(current_pos);
    float new_angle = current_angle + (dir_multiplier * step_deg);
    if (new_angle < -180.0f) {
        new_angle = -180.0f;
    } else if (new_angle > 180.0f) {
        new_angle = 180.0f;
    }

    return uart_console_set_joint_position(joint_id, new_angle);
}