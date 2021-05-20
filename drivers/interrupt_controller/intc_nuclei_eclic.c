/*
 * Copyright (c) 2021 Tokita, Hiroshi <tokita.hiroshi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Nuclie's Extended Core Interrupt Controller
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <init.h>
#include <soc.h>

#include <sw_isr_table.h>

// static inline void eclic_init(uint32_t num_irq)
// {
// 	ECLIC_SetMth(0);
// 	ECLIC_SetCfgNlbits(__ECLIC_INTCTLBITS);
// }

// static inline void eclic_mode_enable(void)
// {
// 	uint32_t mtvec_value = __RV_CSR_READ(CSR_MTVEC);

// 	mtvec_value = mtvec_value & 0xFFFFFFC0;
// 	mtvec_value = mtvec_value | 0x00000003;
// 	__RV_CSR_WRITE(CSR_MTVEC, mtvec_value);
// }

// static inline void eclic_global_interrupt_enable(void)
// {
// 	/* set machine interrupt enable bit */
// 	__RV_CSR_SET(CSR_MSTATUS, MSTATUS_MIE);
// }

/**
 * @brief Initialize the Platform Level Interrupt Controller
 * @return N/A
 */
static int _eclic_init(const struct device *dev)
{
	/* Initialze ECLIC */
	eclic_init(CONFIG_NUM_IRQS);
	eclic_mode_enable();
	eclic_global_interrupt_enable();

	return 0;
}

SYS_INIT(_eclic_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
