# Copyright (c) 2021, ATL Electronics
#
# SPDX-License-Identifier: Apache-2.0
#

add_subdirectory(common)
add_subdirectory(${SOC_SERIES})

zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/${CONFIG_SOC_SUB_SERIES})
