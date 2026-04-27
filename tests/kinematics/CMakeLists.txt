# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(test_kinematics LANGUAGES C)

# Path to the main application source
set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../app)

# Include application headers
target_include_directories(app PRIVATE
  ${APP_SRC_DIR}/include
)

# Application kinematics sources under test
target_sources(app PRIVATE
  ${APP_SRC_DIR}/src/kinematics/kinematics_math.c
  ${APP_SRC_DIR}/src/kinematics/matrix_exp.c
  ${APP_SRC_DIR}/src/kinematics/robot_geometry.c
  ${APP_SRC_DIR}/src/kinematics/forward_kinematics_poe.c
)

# HAL flash stub (replaces real NVS-based hal_flash.c for test builds)
target_sources(app PRIVATE
  src/hal_flash_stub.c
)

# Test sources
target_sources(app PRIVATE
  src/test_kinematics_math.c
  src/test_matrix_exp.c
  src/test_forward_kinematics.c
)
