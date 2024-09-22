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
elseif (CONFIG_SOC_SERIES_BL61X)
zephyr_compile_definitions(bl616)
zephyr_compile_definitions(BL616)
endif()

zephyr_include_directories(${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include)
zephyr_include_directories(${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include/bouffalolab/${SOC_SERIES})

if (CONFIG_WIFI_BFLB OR CONFIG_BT_BFLB)
zephyr_include_directories(${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/blob/${SOC_SERIES}/include)
zephyr_include_directories(${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/blob/${SOC_SERIES}/hal)
zephyr_link_libraries(${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/blob/${SOC_SERIES}/lib/libphyrf.a)
zephyr_link_libraries(${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/blob/${SOC_SERIES}/lib/libwifi6_fixed.a)
endif()
