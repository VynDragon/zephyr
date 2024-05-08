zephyr_library_named(hal_${CONFIG_SOC_SUB_SERIES})

set(bflb_soc_path_prefix ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/drivers/soc/${CONFIG_SOC_SUB_SERIES}/std)

#zephyr_library_sources(
#${bflb_soc_path_prefix}/startup/start.S
#${bflb_soc_path_prefix}/startup/start_load.c
#${bflb_soc_path_prefix}/startup/system_bl602.c
#${bflb_soc_path_prefix}/startup/interrupt.c
#)

if(CONFIG_ROMAPI)
zephyr_library_sources(${bflb_soc_path_prefix}/src/bl702_romapi.c)
zephyr_compile_definitions(BFLB_USE_ROM_DRIVER)
endif()


zephyr_library_sources(
${bflb_soc_path_prefix}/src/bl702_aon.c
${bflb_soc_path_prefix}/src/bl702_common.c
${bflb_soc_path_prefix}/src/bl702_clock.c
${bflb_soc_path_prefix}/src/bl702_ef_cfg.c
${bflb_soc_path_prefix}/src/bl702_glb.c
${bflb_soc_path_prefix}/src/bl702_hbn.c
${bflb_soc_path_prefix}/src/bl702_l1c.c
${bflb_soc_path_prefix}/src/bl702_pds.c

${bflb_soc_path_prefix}/src/bl702_sflash_ext.c
${bflb_soc_path_prefix}/src/bl702_xip_sflash_ext.c
${bflb_soc_path_prefix}/src/bl702_sf_cfg_ext.c

${bflb_soc_path_prefix}/src/bl702_pm.c
${bflb_soc_path_prefix}/port/bl702_clock.c
)


zephyr_include_directories(
${bflb_soc_path_prefix}/include
${bflb_soc_path_prefix}/include/hardware
)

SET(MCPU "riscv-e24")
SET(MARCH "rv32imafc")
SET(MABI "ilp32f")

zephyr_compile_definitions(CONFIG_IRQ_NUM=80)


# bsp section

#zephyr_library_sources(
#${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/bsp/board/${CONFIG_SOC_SUB_SERIES}dk/board.c
#${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/bsp/board/${CONFIG_SOC_SUB_SERIES}dk/fw_header.c

#${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/components/mm/mem.c
#${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/components/mm/tlsf/tlsf.c
#${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/components/mm/tlsf/bflb_tlsf.c
#)

#zephyr_include_directories(
#${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/bsp/board/${CONFIG_SOC_SUB_SERIES}dk/

#${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/components/mm/
#)

#zephyr_compile_definitions(
#CONFIG_TLSF
#)
