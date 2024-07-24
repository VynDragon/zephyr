# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

zephyr_sources_ifdef(CONFIG_RISCV_PRIVILEGED
  soc_common_irq_privileged.c
  )
