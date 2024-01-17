
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_FAMILY_BFLB)

zephyr_library_named(hal_bouffalolab)

zephyr_compile_definitions(
  ARCH_RISCV
)

if (EXISTS ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR})
  set(CONFIG_HAS_BFLB_HAL TRUE CACHE BOOL)
endif()

zephyr_library_compile_definitions(
  BFLB_USE_HAL_DRIVER
  BFLB_USE_CUSTOM_LD_SECTIONS
)

set(bflb_soc            ${CONFIG_SOC_SUB_SERIES})
set(bflb_drv_dir        ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/soc/${bflb_soc}/std)
set(bflb_drv_src_dir    ${bflb_drv_dir}/src)
set(bflb_components_dir ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/components)
set(bflb_lhal_dir       ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/lhal)

zephyr_include_directories(
  include

  ${bflb_drv_dir}/include
  ${bflb_drv_dir}/startup
  ${bflb_lhal_dir}/include
)


# probably not right
zephyr_library_include_directories(
  ${bflb_components_dir}/utils/math/include
)

zephyr_library_sources(
  ${bflb_drv_src_dir}/${bflb_soc}_aon.c
  ${bflb_drv_src_dir}/${bflb_soc}_ef_ctrl.c
  ${bflb_drv_src_dir}/${bflb_soc}_glb.c
  ${bflb_drv_src_dir}/${bflb_soc}_hbn.c
  ${bflb_drv_src_dir}/${bflb_soc}_l1c.c
  ${bflb_drv_src_dir}/${bflb_soc}_pds.c
  ${bflb_drv_src_dir}/${bflb_soc}_romapi.c

  ${bflb_components_dir}/utils/math/src/utils_crc.c
)



zephyr_library_sources_ifdef(CONFIG_USE_BFLB_ACOMP ${bflb_drv_src_dir}/${bflb_soc}_acomp.c)
zephyr_library_sources_ifdef(CONFIG_USE_BFLB_ADC   ${bflb_drv_src_dir}/${bflb_soc}_adc.c)
zephyr_library_sources_ifdef(CONFIG_USE_BFLB_DAC   ${bflb_drv_src_dir}/${bflb_soc}_dac.c)
zephyr_library_sources_ifdef(CONFIG_USE_BFLB_DMA   ${bflb_drv_src_dir}/${bflb_soc}_dma.c)
zephyr_library_sources_ifdef(CONFIG_USE_BFLB_I2C   ${bflb_drv_src_dir}/${bflb_soc}_i2c.c)
zephyr_library_sources_ifdef(CONFIG_USE_BFLB_IR    ${bflb_drv_src_dir}/${bflb_soc}_ir.c)
zephyr_library_sources_ifdef(CONFIG_USE_BFLB_PWM   ${bflb_drv_src_dir}/${bflb_soc}_pwm.c)
zephyr_library_sources_ifdef(CONFIG_USE_BFLB_SPI   ${bflb_drv_src_dir}/${bflb_soc}_spi.c)
zephyr_library_sources_ifdef(CONFIG_USE_BFLB_UART  ${bflb_drv_src_dir}/${bflb_soc}_uart.c)

endif()
