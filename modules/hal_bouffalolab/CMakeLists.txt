# Copyright (c) 2021-2025 Gerson Fernando Budke <nandojve@gmail.com>
# Copyright (c) 2024-2025 MASSDRIVER EI (massdriver.space)
#
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_FAMILY_BFLB)
  zephyr_include_directories(${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include)
  zephyr_include_directories(${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include/bouffalolab/${SOC_SERIES})

  add_subdirectory_ifdef(CONFIG_SOC_SERIES_BL70XL src/bl70xl)
endif() # SOC_FAMILY_BFLB
