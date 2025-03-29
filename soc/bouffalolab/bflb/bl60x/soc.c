/*
 * Copyright (c) 2024, MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Bouffalo Lab RISC-V MCU series initialization code
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/eeprom.h>

#include <soc.h>
#include <clic.h>

/* Set Embedded Flash Pullup */
static void system_bor_init(void)
{
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(HBN_BASE + HBN_BOR_CFG_OFFSET);
	/* borThreshold = 1 */
	tmpVal = (tmpVal & HBN_BOR_VTH_UMSK) | ((uint32_t)(1) << HBN_BOR_VTH_POS);
	/* enablePorInBor true*/
	tmpVal = (tmpVal & HBN_BOR_SEL_UMSK) | ((uint32_t)(1) << HBN_BOR_SEL_POS);
	/* enableBor true*/
	tmpVal = (tmpVal & HBN_PU_BOR_UMSK) | ((uint32_t)(1) << HBN_PU_BOR_POS);
	sys_write32(tmpVal, HBN_BASE + HBN_BOR_CFG_OFFSET);


	/* enableBorInt false */
	tmpVal = sys_read32(HBN_BASE + HBN_IRQ_MODE_OFFSET);
	tmpVal = tmpVal & HBN_IRQ_BOR_EN_UMSK;
	sys_write32(tmpVal, HBN_BASE + HBN_IRQ_MODE_OFFSET);
}

#ifdef CONFIG_RISCV_GP
ulong_t __soc_get_gp_initial_value(void)
{
	extern uint32_t __global_pointer$;
	return (ulong_t)&__global_pointer$;
}
#endif

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */

void soc_early_init_hook(void)
{
	uint32_t key;
	uint32_t *p;
	uint32_t i = 0;
	uint32_t tmpVal = 0;

	key = irq_lock();


	/* disable hardware_pullup_pull_down (reg_en_hw_pu_pd = 0) */
	tmpVal = sys_read32(HBN_BASE + HBN_IRQ_MODE_OFFSET);
	/* "BL_CLR_REG_BIT" */
	tmpVal = tmpVal & HBN_REG_EN_HW_PU_PD_UMSK;
	sys_write32(tmpVal, HBN_BASE + HBN_IRQ_MODE_OFFSET);

	/* 'seam' 0kb, undocumented */
	tmpVal = sys_read32(GLB_BASE + GLB_SEAM_MISC_OFFSET);
	tmpVal = (tmpVal & GLB_EM_SEL_UMSK) | ((uint32_t)(0) << GLB_EM_SEL_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_SEAM_MISC_OFFSET);

	/* Fix 26M xtal clkpll_sdmin */
	tmpVal = sys_read32(PDS_BASE + PDS_CLKPLL_SDM_OFFSET);

	if ((tmpVal & PDS_CLKPLL_SDMIN_MSK) == 0x49D39D) {
		tmpVal = (tmpVal & PDS_CLKPLL_SDMIN_UMSK) | (uint32_t)(0x49D89E);
		sys_write32(tmpVal, PDS_BASE + PDS_CLKPLL_SDM_OFFSET);
	}

	tmpVal = sys_read32(GLB_BASE + GLB_PARM_OFFSET);
	/* GLB_UART_Sig_Swap_Set(UART_SIG_SWAP_NONE);
	 * no swap = 0
	 * see bl602_glb.h for other possible values
	 */
	tmpVal = (tmpVal & GLB_UART_SWAP_SET_UMSK) | ((uint32_t)(0) <<
GLB_UART_SWAP_SET_POS);
	/* GLB_JTAG_Sig_Swap_Set(JTAG_SIG_SWAP_NONE); */
	tmpVal = (tmpVal & GLB_JTAG_SWAP_SET_UMSK) | ((uint32_t)(0) <<
GLB_JTAG_SWAP_SET_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_PARM_OFFSET);

	/* CLear all interrupt */
	p = (uint32_t *)(CLIC_HART0_ADDR + CLIC_INTIE);

	for (i = 0; i < (IRQn_LAST + 3) / 4; i++) {
		p[i] = 0;
	}

	p = (uint32_t *)(CLIC_HART0_ADDR + CLIC_INTIP);

	for (i = 0; i < (IRQn_LAST + 3) / 4; i++) {
		p[i] = 0;
	}

	/* init bor for all platform */
	system_bor_init();


	irq_unlock(key);
}

