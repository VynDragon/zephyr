zephyr_library_named(easyflash4)

zephyr_library_include_directories(
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/components/easyflash4/inc
)

zephyr_library_sources(
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/components/easyflash4/src/easyflash.c
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/components/easyflash4/src/ef_env.c
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/components/easyflash4/src/ef_env_legacy_wl.c
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/components/easyflash4/src/ef_env_legacy.c
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/components/easyflash4/src/ef_port.c src/ef_utils.c
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/components/easyflash4/src/easyflash_cli.c
)

if(CONFIG_CLI_CMD_ENABLE)
zephyr_compile_definitions(CONFIG_CLI_CMD_ENABLE)
endif()
zephyr_compile_definitions(CONFIG_EASYFLASH4)
