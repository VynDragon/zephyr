# Copyright (c) 2024 MASSDRIVER EI (massdriver.space)
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_compile_definitions(
  ARCH_RISCV
  __LINUX_ERRNO_EXTENSIONS__
)


if (CONFIG_SOC_SERIES_BL60X)
zephyr_compile_definitions(bl602)
zephyr_compile_definitions(BL602)
elseif (CONFIG_SOC_SERIES_BL70X)
zephyr_compile_definitions(bl702)
zephyr_compile_definitions(BL702)
endif()

zephyr_include_directories(${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include)
zephyr_include_directories(${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include/bouffalolab/${SOC_SERIES})
