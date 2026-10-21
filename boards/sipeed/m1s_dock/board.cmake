# Copyright The Zephyr Project Contributors
#
# SPDX-License-Identifier: Apache-2.0

board_runner_args(openocd --cmd-pre-init "source [find bl808.cfg]")

board_runner_args(openocd --file-type=elf --no-load)
board_runner_args(openocd --gdb-init "set print asm-demangle on")
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)

board_runner_args(bflb_mcu_tool --chipname bl808)
include(${ZEPHYR_BASE}/boards/common/bflb_mcu_tool.board.cmake)

board_set_flasher(bflb_mcu_tool)
