/*
 * OctroBot Robot Arm Firmware - HAL Common Types
 * Copyright (c) 2026 OctroBot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef OCTROBOT_HAL_TYPES_H
#define OCTROBOT_HAL_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Return codes */
#define HAL_OK           0
#define HAL_ERROR       -1
#define HAL_TIMEOUT     -2
#define HAL_BUSY        -3
#define HAL_INVALID     -4

/* Opaque handle types for HAL modules */
typedef struct hal_uart_handle_s *hal_uart_handle_t;

#endif /* OCTROBOT_HAL_TYPES_H */
