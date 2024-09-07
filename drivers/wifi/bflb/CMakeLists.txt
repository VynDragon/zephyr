# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_WIFI_BFLB
  src/bflb_wifi_hal.c
  src/bflb_wifi_drv.c
)
