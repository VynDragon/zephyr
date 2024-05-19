zephyr_library_named(hal_${CONFIG_SOC_SUB_SERIES})

SET(MCPU "riscv-e24")
SET(MARCH "rv32imafc")
SET(MABI "ilp32f")

zephyr_compile_definitions(CONFIG_IRQ_NUM=80)
