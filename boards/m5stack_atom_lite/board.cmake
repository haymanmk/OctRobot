# SPDX-License-Identifier: Apache-2.0

board_runner_args(esp32 "--esp-boot-mode=default_reset")
board_runner_args(esp32 "--esp-tool=esptool.py")

include(${ZEPHYR_BASE}/boards/common/esp32.board.cmake)
