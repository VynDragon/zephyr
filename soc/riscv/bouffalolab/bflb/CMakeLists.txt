# Copyright (c) 2021, ATL Electronics
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_compile_definitions(${CONFIG_SOC_SUB_SERIES})


zephyr_include_directories(.)
zephyr_include_directories(
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/bsp/board/${CONFIG_SOC_SUB_SERIES}dk/
)
zephyr_sources(
  soc.c
)

# zephyr_linker_sources_ifdef(CONFIG_SOC_FAMILY_BFLB RODATA rodata.ld)

set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/../${CONFIG_SOC_SUB_SERIES}/linker.ld CACHE INTERNAL "")
