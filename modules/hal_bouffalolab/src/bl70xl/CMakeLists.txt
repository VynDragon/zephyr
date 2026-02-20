# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BT_BFLB_BL70XL)
  zephyr_include_directories(
    ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include/bouffalolab/${SOC_SERIES}/ble)

  # Select controller library and per-variant parameters
  if(CONFIG_BFLB_BL70XL_BLE_M0S1P)
    set(BLE_VARIANT_LIB libbtblecontroller_702l_m0s1p.a)
    set(BLE_CFG_CON 1)
    set(BLE_TX_BUFF_DATA 2)
  elseif(CONFIG_BFLB_BL70XL_BLE_M0S1RP)
    set(BLE_VARIANT_LIB libbtblecontroller_702l_m0s1rp.a)
    set(BLE_CFG_CON 1)
    set(BLE_TX_BUFF_DATA 5)
  elseif(CONFIG_BFLB_BL70XL_BLE_M0S1SP)
    set(BLE_VARIANT_LIB libbtblecontroller_702l_m0s1sp.a)
    set(BLE_CFG_CON 1)
    set(BLE_TX_BUFF_DATA 2)
  elseif(CONFIG_BFLB_BL70XL_BLE_M0S2P)
    set(BLE_VARIANT_LIB libbtblecontroller_702l_m0s2p.a)
    set(BLE_CFG_CON 2)
    set(BLE_TX_BUFF_DATA 4)
  elseif(CONFIG_BFLB_BL70XL_BLE_M0S4P)
    set(BLE_VARIANT_LIB libbtblecontroller_702l_m0s4p.a)
    set(BLE_CFG_CON 4)
    set(BLE_TX_BUFF_DATA 6)
  elseif(CONFIG_BFLB_BL70XL_BLE_M1S1P)
    set(BLE_VARIANT_LIB libbtblecontroller_702l_m1s1p.a)
    set(BLE_CFG_CON 1)
    set(BLE_TX_BUFF_DATA 2)
  elseif(CONFIG_BFLB_BL70XL_BLE_M2S1P)
    set(BLE_VARIANT_LIB libbtblecontroller_702l_m2s1p.a)
    set(BLE_CFG_CON 2)
    set(BLE_TX_BUFF_DATA 4)
  elseif(CONFIG_BFLB_BL70XL_BLE_M4S1P)
    set(BLE_VARIANT_LIB libbtblecontroller_702l_m4s1p.a)
    set(BLE_CFG_CON 4)
    set(BLE_TX_BUFF_DATA 6)
  elseif(CONFIG_BFLB_BL70XL_BLE_M8S1P)
    set(BLE_VARIANT_LIB libbtblecontroller_702l_m8s1p.a)
    set(BLE_CFG_CON 8)
    set(BLE_TX_BUFF_DATA 8)
  endif()

  zephyr_compile_definitions(
    BL702L
    CFG_BLE_ENABLE
    BFLB_BLE
    CFG_BLE
    CFG_CON=${BLE_CFG_CON}
    CFG_BLE_TX_BUFF_DATA=${BLE_TX_BUFF_DATA}
    CONFIG_HW_SEC_ENG_DISABLE
    ARCH_RISCV
  )

  if(NOT CONFIG_BUILD_ONLY_NO_BLOBS)
    if(CONFIG_BFLB_BL70XL_REV_A1)
      set(BL70XL_ROM_LIB libbl702l_rom_a1.a)
    else()
      set(BL70XL_ROM_LIB libbl702l_rom_a0.a)
    endif()

    zephyr_link_libraries(
      ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/lib/${SOC_SERIES}/${BLE_VARIANT_LIB}
      ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/lib/${SOC_SERIES}/libbl702l_rf.a
      ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/lib/${SOC_SERIES}/${BL70XL_ROM_LIB}
    )
  endif()
endif()
