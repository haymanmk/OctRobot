# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Find Zephyr. This also loads Zephyr's build system.
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(octrobot_firmware VERSION 0.3.0 LANGUAGES C)

# Add application source files
target_sources(app PRIVATE
  src/main.c
)

# Add include directories
target_include_directories(app PRIVATE
  include
)

# HAL layer (Phase 2)
target_sources(app PRIVATE
  src/hal/half_duplex_uart.c
  src/hal/hal_gpio.c
  src/hal/hal_flash.c
)

# Servo driver (Phase 3)
target_sources(app PRIVATE
  src/drivers/feetech_protocol.c
  src/drivers/feetech_servo.c
)

# Host communications (Phase 3b)
target_sources(app PRIVATE
  src/comms/packet_protocol.c
  src/comms/host_comms.c
  src/comms/uart_console.c
)

# Kinematics (Phase 4 - POE implementation)
target_sources(app PRIVATE
  src/kinematics/kinematics_math.c
  src/kinematics/matrix_exp.c
  src/kinematics/robot_geometry.c
  src/kinematics/forward_kinematics_poe.c
)

# Trajectory planner (to be added in Phase 5)
# target_sources(app PRIVATE
#   src/trajectory/joint_trajectory.c
# )

# Motion controller (to be added in Phase 6)
target_sources(app PRIVATE
  src/controller/servo_control.c
)

# Host communications (to be added in Phase 7)
# target_sources(app PRIVATE
#   src/comms/host_comms.c
# )
