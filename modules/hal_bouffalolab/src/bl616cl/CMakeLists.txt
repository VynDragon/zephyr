# Copyright The Zephyr Project Contributors
#
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BT_BFLB_BL616CL)
  zephyr_include_directories(
    ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include/bouffalolab/${SOC_SERIES}/ble)

  # Select controller variant
  if(CONFIG_BFLB_BL616CL_BLE_M2S1)
    set(BLE_VARIANT_LIB libbtblecontroller_bl616cl_m2s1.a)
  elseif(CONFIG_BFLB_BL616CL_BLE_M0B1)
    set(BLE_VARIANT_LIB libbtblecontroller_bl616cl_m0b1.a)
  endif()

  message(STATUS "BL616CL BLE controller: ${BLE_VARIANT_LIB}")
  set(BLE_BLOBS_DIR ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/zephyr/blobs/lib/${SOC_SERIES})

  set(PHYRF_LIB libbl616cl_phyrf.a)

  if(NOT CONFIG_BUILD_ONLY_NO_BLOBS)
    zephyr_link_libraries(
      ${BLE_BLOBS_DIR}/${BLE_VARIANT_LIB}
      ${BLE_BLOBS_DIR}/${PHYRF_LIB}
    )
  endif()

endif()
