zephyr_library_named(hal_bouffalolab_easyflash4)

set(bflb_easyflash4_prefix ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/components/easyflash4)
set(bflb_utils_prefix ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/components/utils)

zephyr_library_sources(
${bflb_easyflash4_prefix}/src/easyflash.c
${bflb_easyflash4_prefix}/src/ef_env.c
${bflb_easyflash4_prefix}/src/ef_env_legacy_wl.c
${bflb_easyflash4_prefix}/src/ef_env_legacy.c
${bflb_easyflash4_prefix}/src/ef_port.c
${bflb_easyflash4_prefix}/src/ef_utils.c
${bflb_easyflash4_prefix}/src/easyflash_cli.c

${bflb_utils_prefix}/log/log.c
${bflb_utils_prefix}/bflb_mtd/bflb_mtd.c
)

zephyr_include_directories(
${bflb_easyflash4_prefix}/inc
${bflb_utils_prefix}/log
${bflb_utils_prefix}/bflb_mtd/include
)
zephyr_compile_definitions(
CONFIG_EASYFLASH4
)
