/*
 * Copyright (c) 2024 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Board configuration macros
 *
 * This header file is used to specify and describe board-level aspects
 */

#ifndef _SOC__H_
#define _SOC__H_

#include <zephyr/sys/util.h>

#ifndef _ASMLANGUAGE

/* Add include for DTS generated information */
#include <zephyr/devicetree.h>

/* Addresses */

#include <bouffalolab/bl61x/bflb_soc.h>
#include <bouffalolab/bl61x/aon_reg.h>
#include <bouffalolab/bl61x/glb_reg.h>
#include <bouffalolab/bl61x/hbn_reg.h>
#include <bouffalolab/bl61x/pds_reg.h>
#include <bouffalolab/bl61x/tzc_sec_reg.h>
#include <bouffalolab/bl61x/ef_ctrl_reg.h>
#include <bouffalolab/bl61x/sf_ctrl_reg.h>
#include <bouffalolab/bl61x/mcu_misc_reg.h>

#define RISCV_MSIP			0xE0000000

/* RISC-V Machine Timer configuration */
#define RISCV_MTIMECMP_BASE		0xE0004000
#define RISCV_MTIME_BASE		0xE000BFF8

/* lib-c hooks required RAM defined variables */
#define RISCV_RAM_BASE               DT_SRAM_BASE_ADDRESS
#define RISCV_RAM_SIZE               KB(DT_SRAM_SIZE)

#define SOC_BOUFFALOLAB_BL_HCLK_FREQ_HZ	\
	DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency)

#endif /* !_ASMLANGUAGE */

#endif /* _SOC__H_ */
