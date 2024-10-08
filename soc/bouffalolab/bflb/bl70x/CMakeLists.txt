# Copyright (c) 2024 MASSDRIVER EI (massdriver.space)
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_compile_definitions(
  ARCH_RISCV
  __LINUX_ERRNO_EXTENSIONS__
)


zephyr_compile_definitions(bl702)
zephyr_compile_definitions(BL702)

zephyr_include_directories(.)
zephyr_sources(soc.c)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")

zephyr_code_relocate_ifdef(CONFIG_UART_BFLB LIBRARY drivers__serial LOCATION ITCM)
zephyr_code_relocate_ifdef(CONFIG_RISCV_MACHINE_TIMER LIBRARY drivers__timer LOCATION ITCM)
zephyr_code_relocate_ifdef(CONFIG_PINCTRL_BFLB LIBRARY drivers__pinctrl LOCATION ITCM)
zephyr_code_relocate_ifdef(CONFIG_I2C_BFLB LIBRARY drivers__i2c LOCATION ITCM)
zephyr_code_relocate_ifdef(CONFIG_GPIO_BFLB LIBRARY drivers__gpio LOCATION ITCM)
zephyr_code_relocate_ifdef(CONFIG_SOC_FLASH_BFLB FILES ${ZEPHYR_BASE}/drivers/flash/flash_bflb.c
LOCATION ITCM NOKEEP)
