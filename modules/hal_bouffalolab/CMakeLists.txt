
if(CONFIG_SOC_FAMILY_BFLB)

zephyr_library_named(hal_bouffalolab)

zephyr_compile_definitions(
  ARCH_RISCV
  __LINUX_ERRNO_EXTENSIONS__
  BFLB_USE_HAL_DRIVER
)


string(TOUPPER ${CONFIG_SOC_SUB_SERIES} CONFIG_SOC_SUB_SERIES_SDK)
zephyr_compile_definitions(${CONFIG_SOC_SUB_SERIES})
zephyr_compile_definitions(${CONFIG_SOC_SUB_SERIES_SDK})


zephyr_include_directories(
  ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/include
  include/${CONFIG_SOC_SUB_SERIES}
  include/common
)

include(${CONFIG_SOC_SUB_SERIES}.cmake)
target_link_libraries(zephyr_interface INTERFACE -lc)
endif()
