# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_WIFI_BFLB_BL60X
  bl60x/bflb_wifi_hal.c
  bl60x/bflb_wifi_drv.c
)

zephyr_library_sources_ifdef(CONFIG_WIFI_BFLB_BL61X
  bl61x/bflb_wifi_hal.c
  bl61x/bflb_wifi_drv.c
)

zephyr_include_directories_ifdef(CONFIG_WIFI_BFLB_BL61X
  bl61x/
)
