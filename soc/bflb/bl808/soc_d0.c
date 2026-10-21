/*
 * Copyright (c) 2026 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Bouffalo Lab RISC-V MCU series initialization code
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/cache.h>

#include <bflb_soc.h>
#include <glb_reg.h>
#include <hbn_reg.h>
#include <mm_glb_reg.h>
#include <mm_misc_reg.h>
#include <mcu_misc_reg.h>
#include <pds_reg.h>
#include <tzc_sec_reg.h>

#include "thead_csr.h"

void soc_early_init_hook(void)
{
	/* Invalidate all MMU TLB Entries */
	__asm__ volatile("sfence.vma x0, x0");
}

void arch_cache_init(void)
{
	enable_thead_isa_ext();
	set_thead_enforce_aligned(false);
	sys_cache_data_enable();
	enable_branchpred(true);
	sys_cache_instr_enable();
	disable_interrupt_autostacking();
	set_thead_mhint(true, AMR_64L, PREF_N_2);
	sys_cache_data_flush_and_invd_all();
	sys_cache_instr_invd_all();
}
