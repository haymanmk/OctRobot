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
  src/hal/hal_timer.c
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
)

# Kinematics (to be added in Phase 4)
# target_sources(app PRIVATE
#   src/kinematics/math.c
#   src/kinematics/forward_kinematics.c
#   src/kinematics/inverse_kinematics.c
# )

# Trajectory planner (to be added in Phase 5)
# target_sources(app PRIVATE
#   src/trajectory/joint_trajectory.c
# )

# Motion controller (to be added in Phase 6)
# target_sources(app PRIVATE
#   src/controller/motion_controller.c
# )

# Host communications (to be added in Phase 7)
# target_sources(app PRIVATE
#   src/comms/host_comms.c
# )
