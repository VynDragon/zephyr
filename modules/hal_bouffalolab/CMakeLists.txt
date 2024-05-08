
if(CONFIG_SOC_FAMILY_BFLB)

# we need to define the proper sections names, defaults dont match zephyr LD scripts
add_compile_options(-include ${CMAKE_CURRENT_SOURCE_DIR}/include/bl_ld_sections.h)


zephyr_library_named(hal_bouffalolab)

zephyr_compile_definitions(
  ARCH_RISCV
  __LINUX_ERRNO_EXTENSIONS__
  BFLB_USE_HAL_DRIVER
)


string(TOUPPER ${CONFIG_SOC_SUB_SERIES} CONFIG_SOC_SUB_SERIES_SDK)
zephyr_compile_definitions(${CONFIG_SOC_SUB_SERIES})
zephyr_compile_definitions(${CONFIG_SOC_SUB_SERIES_SDK})


zephyr_library_sources(${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_common.c)

zephyr_library_sources(
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_adc.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_acomp.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_cks.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_ef_ctrl.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_gpio.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_i2c.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_dma.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_rtc.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_sec_aes.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_sec_sha.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_sec_trng.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_spi.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_timer.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_uart.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_wdg.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_flash.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/flash/bflb_sf_cfg.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/flash/bflb_xip_sflash.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/flash/bflb_sflash.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/flash/bflb_sf_ctrl.c
)

if("${CONFIG_SOC_SUB_SERIES}" STREQUAL "bl602")
zephyr_library_sources(
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_dac.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_ir.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_pwm_v1.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_sdio2.c
)
elseif("${CONFIG_SOC_SUB_SERIES}" STREQUAL "bl702")
zephyr_library_sources(
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_dac.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_emac.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_ir.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_pwm_v1.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_cam.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_spi_psram.c
)
elseif("${CONFIG_SOC_SUB_SERIES}" STREQUAL "bl702l")
zephyr_library_sources(
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_kys.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_pwm_v1.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_pwm_v2.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_spi_psram.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_auadc.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_ir.c
)
elseif("${CONFIG_SOC_SUB_SERIES}" STREQUAL "bl616")
zephyr_library_sources(
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_dac.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_emac.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_ir.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_mjpeg.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_pwm_v2.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_cam.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_sdio2.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_i2s.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_dbi.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_audac.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_auadc.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_platform_dma.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_wo.c
)
elseif("${CONFIG_SOC_SUB_SERIES}" STREQUAL "bl628")
zephyr_library_sources(
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_dac.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_emac.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_clock.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_pwm_v2.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_wo.c
)
elseif("${CONFIG_SOC_SUB_SERIES}" STREQUAL "bl808")
zephyr_library_sources(
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_dac.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_emac.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_ir.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_mjpeg.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_pwm_v2.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_cam.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_csi.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_i2s.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_wo.c
)
endif()

#if("${CONFIG_SOC_SUB_SERIES}" STREQUAL "bl702")
#zephyr_library_sources(${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_usb_v1.c)
#endif()

zephyr_library_sources(
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_irq.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_l1c.c
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/bflb_mtimer.c
)

zephyr_include_directories(
  include/${CONFIG_SOC_SUB_SERIES}
  # this MUST be before /drivers/lhal/include/arch, there is a interface for nonmatching risc-v
  # and riscv paths so as to not redefine risc-v related stuffs (the defines are the same)
  include

  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/include
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/include/arch
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/config/${CONFIG_SOC_SUB_SERIES}
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/src/flash
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include
)

if(NOT (("${CONFIG_SOC_SUB_SERIES}" STREQUAL "bl702") OR
("${CONFIG_SOC_SUB_SERIES}" STREQUAL "bl602") OR ("${CONFIG_SOC_SUB_SERIES}" STREQUAL "bl702l")))
zephyr_include_directories(
  include
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/include/arch/risc-v/t-head
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/include/arch/risc-v/t-head/Core/Include
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/include/arch/risc-v/t-head/csi_dsp/include
)
zephyr_library_sources(
include/arch/risc-v/t-head/rv_hart.c
include/arch/risc-v/t-head/rv_pmp.c
)
endif()

zephyr_library_sources(
${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal/config/${CONFIG_SOC_SUB_SERIES}/device_table.c
)

include(${CONFIG_SOC_SUB_SERIES}.cmake)
#include(newlib.cmake)
include(easyflash4.cmake)

target_link_libraries(zephyr_interface INTERFACE -lc)
endif()
