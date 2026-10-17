# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

include("../../cmake/blob_objcopy.cmake")

set(BLOBS_DIR ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/zephyr/blobs/lib/bl808x)

# Replace the blobs' printf with blob_printf
set(BLOBS_OBJCOPY_ARGS --redefine-sym printf=blob_printf)

if(CONFIG_BT_BFLB_BL808)
  zephyr_include_directories(
    ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include/bouffalolab/${CONFIG_SOC_SERIES}/ble)

  message(STATUS "BL808 BLE: on-chip controller blobs")

  if(NOT CONFIG_BUILD_ONLY_NO_BLOBS)
    blob_objcopy(btblecontroller.a
      "${BLOBS_DIR}/libbl606p_btblecontroller_prebuild.a" ${BLOBS_OBJCOPY_ARGS})
  endif()
endif()

if(CONFIG_WIFI_BFLB AND NOT CONFIG_BUILD_ONLY_NO_BLOBS)
  blob_objcopy(wifi_firmware.a "${BLOBS_DIR}/libbl606p_firmware_prebuild.a"
    ${BLOBS_OBJCOPY_ARGS}
    --redefine-sym puts=blob_puts
    --redefine-sym putchar=blob_putchar
  )
  # The wraps intercept calls and chain to __real_, which objcopy cannot
  # express since it also renames the archive's internal references
  zephyr_link_libraries(
    -Wl,--wrap=ipc_emb_wait
    -Wl,--wrap=ipc_emb_notify
    -Wl,--wrap=assert_err
    -Wl,--wrap=bl_sleep_schedule
    -Wl,--wrap=rc_update_counters
    -Wl,--wrap=sta_mgmt_unregister
  )
endif()


if(CONFIG_IEEE802154_BFLB)
  if(NOT CONFIG_BUILD_ONLY_NO_BLOBS)
    # There are no ieee802.15.4 blobs for BL808, however the BL61x one can make it work because the
    # RF-handling/main E907 CPU is setup almost entirely the same between these chips.
    # Mostly 4 register offsets have moved, as BL808 uses the older BL702 layout:
    #                    BL61x   BL808
    # TX length          +0x27c  +0x2fc
    # TX rate            +0x280  +0x300
    # RX status          +0x47c  +0x4fc
    # RX rate            +0x480  +0x500
    # Frame-pending mode +0x504  ?
    #
    # Few functions use those, we strip out/wrap those and re-implement then.
    blob_objcopy(
      liblmac154.a
      "${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/zephyr/blobs/lib/bl61x/liblmac154_bl616.a"
      # Force LTO out manually as we need to operate on a LTO-less library
      --remove-section='.gnu.lto_*'
      --remove-section='.gnu.debuglto_*'
      --remove-section='.gnu.lto_.lto.*'
      # Turn the local symbols into weak globals so ours override them: wrap cannot apply here
      --globalize-symbol=LMAC154_SET_MPDU
      --weaken-symbol=LMAC154_SET_MPDU
      --globalize-symbol=LMAC154_GET_MPDU
      --weaken-symbol=LMAC154_GET_MPDU
      --weaken-symbol=lmac154_setTxDataRate
      --weaken-symbol=lmac154_getRxLength
      --weaken-symbol=lmac154_readRxCrc
      --weaken-symbol=lmac154_getRxDataRate
      # --weaken-symbol=lmac154_triggerTx
      # --weaken-symbol=lmac154_setFramePendingMode
      # --weaken-symbol=lmac154_framePendingResult
      # Warn about an orphan section if there was an problem tweaking the library
      --rename-section=.text.LMAC154_SET_MPDU=.lmac154_override_required
      --rename-section=.text.LMAC154_GET_MPDU=.lmac154_override_required
      --rename-section=.text.lmac154_setTxDataRate=.lmac154_override_required
      --rename-section=.text.lmac154_getRxLength=.lmac154_override_required
      --rename-section=.text.lmac154_readRxCrc=.lmac154_override_required
      --rename-section=.text.lmac154_getRxDataRate=.lmac154_override_required
      # --rename-section=.text.lmac154_triggerTx=.lmac154_override_required
      # --rename-section=.text.lmac154_framePendingResult=.lmac154_override_required
      ${BLOBS_OBJCOPY_ARGS}
    )
  endif()
endif()

if(CONFIG_BFLB_PHYRF AND NOT CONFIG_BUILD_ONLY_NO_BLOBS)
  blob_objcopy(phyrf.a "${BLOBS_DIR}/libbl606p_phyrf.a" ${BLOBS_OBJCOPY_ARGS})
  # zephyr_link_libraries(-Wl,--wrap=rfc_init)
endif()
