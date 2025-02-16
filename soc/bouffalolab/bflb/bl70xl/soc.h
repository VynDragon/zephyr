/*
 * Copyright (c) 2025 MASSDRIVER EI (massdriver.space)
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

#include <bouffalolab/bl70xl/bflb_soc.h>
#include <bouffalolab/bl70xl/aon_reg.h>
#include <bouffalolab/bl70xl/glb_reg.h>
#include <bouffalolab/bl70xl/hbn_reg.h>
#include <bouffalolab/bl70xl/l1c_reg.h>
#include <bouffalolab/bl70xl/pds_reg.h>
#include <bouffalolab/bl70xl/tzc_sec_reg.h>
#include <bouffalolab/bl70xl/ef_ctrl_reg.h>
#include <bouffalolab/bl70xl/sf_ctrl_reg.h>
#include <bouffalolab/bl70xl/extra_defines.h>

/* RISC-V Machine Timer configuration */
#define RISCV_MTIME_BASE             0x0200BFF8
#define RISCV_MTIMECMP_BASE          0x02004000

/* lib-c hooks required RAM defined variables */
#define RISCV_RAM_BASE               DT_SRAM_BASE_ADDRESS
#define RISCV_RAM_SIZE               KB(DT_SRAM_SIZE)

#ifdef CONFIG_RISCV_GP
ulong_t __soc_get_gp_initial_value(void);
#endif

#ifdef CONFIG_CODE_DATA_RELOCATION
#define ITCMF __attribute__((section(".itcm")))
#else
#define ITCMF
#endif

#endif /* !_ASMLANGUAGE */

#endif /* _SOC__H_ */
