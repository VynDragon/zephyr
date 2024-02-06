zephyr_library_named(hal_bouffalolab_newlib)

set(bflb_newlib_prefix ${ZEPHYR_HAL_BOUFFALOLAB_MODULE_DIR}/components/libc/newlib)

zephyr_library_sources(
${bflb_newlib_prefix}/syscalls.c
${bflb_newlib_prefix}/port_tty.c
${bflb_newlib_prefix}/port_time.c
${bflb_newlib_prefix}/port_init_fini.c
${bflb_newlib_prefix}/port_file_nosys.c
)
