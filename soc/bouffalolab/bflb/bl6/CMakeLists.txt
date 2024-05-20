# Copyright (c) 2021, ATL Electronics
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_compile_definitions(
  ARCH_RISCV
  __LINUX_ERRNO_EXTENSIONS__
)


string(TOUPPER bl602 CONFIG_SOC_SUB_SERIES_SDK)
zephyr_compile_definitions(bl602)
zephyr_compile_definitions(${CONFIG_SOC_SUB_SERIES_SDK})

zephyr_include_directories(.)
zephyr_sources(soc.c)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")
