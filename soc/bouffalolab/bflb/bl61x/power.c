/*
 * Copyright (c) 2024 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>

#include <bouffalolab/bl61x/glb_reg.h>
#include <bouffalolab/bl61x/cci_reg.h>
#include <bouffalolab/bl61x/pds_reg.h>
#include <bouffalolab/bl61x/hbn_reg.h>
#include <bouffalolab/bl61x/aon_reg.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(soc_power, CONFIG_SOC_LOG_LEVEL);

static void system_clock_settle(void)
{
	__asm__ volatile (".rept 15 ; nop ; .endr");
}

static void system_set_root_clock(uint32_t clock)
{
	uint32_t tmp = 0;

	/* invalid value, fallback to internal 32M */
	if (clock < 0 || clock > 3) {
		clock = 0;
	}
	tmp = *(uint32_t*)(HBN_BASE + HBN_GLB_OFFSET);
	tmp = (tmp & HBN_ROOT_CLK_SEL_UMSK) | (clock << HBN_ROOT_CLK_SEL_POS);
	*(uint32_t*)(HBN_BASE + HBN_GLB_OFFSET) = tmp;

	system_clock_settle();
}

/* enters 'HBN3' True Power OFF (AON power is disabled, power cycle is the only way to wake) */
void z_sys_poweroff(void)
{
	uint32_t tmp;

	/* set clock to osc 32M */
	system_set_root_clock(0);

	/* Power Off RC32K (no rtc needed) */
	tmp = *(uint32_t*)(HBN_BASE + HBN_GLB_OFFSET);
	tmp &= HBN_PU_RC32K_UMSK;
	*(uint32_t*)(HBN_BASE + HBN_GLB_OFFSET) = tmp;

	/* Disable HBN pullups */
	tmp = *(uint32_t*)(HBN_BASE + HBN_IRQ_MODE_OFFSET);
	tmp &= HBN_REG_EN_HW_PU_PD_UMSK;
	*(uint32_t*)(HBN_BASE + HBN_IRQ_MODE_OFFSET) = tmp;

	/* Disable HBN GPIO interrupt */
	tmp = *(uint32_t*)(HBN_BASE + HBN_IRQ_MODE_OFFSET);
	tmp &= HBN_PIN_WAKEUP_MASK_UMSK;
	*(uint32_t*)(HBN_BASE + HBN_IRQ_MODE_OFFSET) = tmp;

	/* disable RTC counter */
	tmp = *(uint32_t*)(HBN_BASE + HBN_CTL_OFFSET);
	*(uint32_t*)(HBN_BASE + HBN_CTL_OFFSET) = tmp & (~1);


	/* Power Off Flash */
	/* TODO requires intelligence for multi-flash */

	/* Power off crystal */
	tmp = *(uint32_t*)(AON_BASE + AON_RF_TOP_AON_OFFSET);
	tmp &= AON_PU_XTAL_AON_UMSK;
	tmp &= AON_PU_XTAL_BUF_AON_UMSK;
	*(uint32_t*)(AON_BASE + AON_RF_TOP_AON_OFFSET) = tmp;

	/* Power Off WIFI pll */
	tmp = *(uint32_t*)(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmp &= GLB_PU_WIFIPLL_UMSK;
	*(uint32_t*)(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET) = tmp;
	tmp = *(uint32_t*)(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmp &= GLB_PU_WIFIPLL_SFREG_UMSK;
	*(uint32_t*)(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET) = tmp;

	/* Power Off Audio PLL */
	tmp = *(uint32_t*)(CCI_BASE + CCI_AUDIO_PLL_CFG0_OFFSET);
	tmp &= CCI_PU_AUPLL_UMSK;
	*(uint32_t*)(CCI_BASE + CCI_AUDIO_PLL_CFG0_OFFSET) = tmp;
	tmp = *(uint32_t*)(CCI_BASE + CCI_AUDIO_PLL_CFG0_OFFSET);
	tmp &= CCI_PU_AUPLL_SFREG_UMSK;
	*(uint32_t*)(CCI_BASE + CCI_AUDIO_PLL_CFG0_OFFSET) = tmp;

	/* Setup undocumented HBN LDO */
	tmp = *(uint32_t*)(HBN_BASE + HBN_CTL_OFFSET);
	tmp &= HBN_LDO11_AON_VOUT_SEL_UMSK;
	tmp &= HBN_LDO11_RT_VOUT_SEL_UMSK;
	tmp |= 10 << HBN_LDO11_AON_VOUT_SEL_POS;
	tmp |= 10 << HBN_LDO11_RT_VOUT_SEL_POS;
	*(uint32_t*)(HBN_BASE + HBN_CTL_OFFSET) = tmp;

	/* set undocumented 'HBN Flag' */
	*(uint32_t*)(HBN_BASE + HBN_RSV0_OFFSET) = 0x4e424845;

	tmp = *(uint32_t*)(HBN_BASE + HBN_CTL_OFFSET);
	/* Power Off HBN_Core */
	tmp |= HBN_PWRDN_HBN_CORE_MSK;
	/* Power Off HBN_RTC */
	tmp |= HBN_PWRDN_HBN_RTC_MSK;
	/* Set double reset at power on, 0: double 1: single*/
	tmp &= HBN_PWR_ON_OPTION_UMSK;
	*(uint32_t*)(HBN_BASE + HBN_CTL_OFFSET) = tmp;

	/* gate all peripherals */
	*(uint32_t*)(GLB_BASE + GLB_CGEN_CFG1_OFFSET) = 0;

	/* Enable HBN */
	tmp = *(uint32_t*)(HBN_BASE + HBN_CTL_OFFSET);
	tmp |= HBN_MODE_MSK;
	*(uint32_t*)(HBN_BASE + HBN_CTL_OFFSET) = tmp;

	k_sleep(K_MSEC(1000));
}
