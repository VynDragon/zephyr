# Copyright (c) 2026 MASSDRIVER EI (massdriver.space)
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_LORA_PAN30XX_NATIVE
  pan30xx.c
  pan30xx_hal.c
)
