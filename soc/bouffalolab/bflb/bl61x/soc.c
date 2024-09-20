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

#include "pll_data.h"

#define CLOCK_TIMEOUT 1024

/* brown out detection */
void bl61x_BOD_init(void)
{
	uint32_t tmpVal = 0;

	/* disable BOD interrupt */
	tmpVal = sys_read32(HBN_BASE + HBN_IRQ_MODE_OFFSET);
	tmpVal &= ~HBN_IRQ_BOR_EN_MSK;
	sys_write32(tmpVal, HBN_BASE + HBN_IRQ_MODE_OFFSET);

	tmpVal = sys_read32(HBN_BASE + HBN_BOR_CFG_OFFSET);
	/* when brownout thresold, restart*/
	tmpVal |= HBN_BOD_SEL_MSK;
	/* set BOD thresold:
	 * 0:2.05v,1:2.10v,2:2.15v....7:2.4v
	 */
	tmpVal &= ~HBN_BOD_VTH_MSK;
	tmpVal |= (7 << HBN_BOD_VTH_POS);
	/* enable BOD */
	tmpVal |= HBN_PU_BOD_MSK;
	sys_write32(tmpVal, HBN_BASE + HBN_BOR_CFG_OFFSET);
}


static void system_clock_settle(void)
{
	__asm__ volatile (".rept 15 ; nop ; .endr");
}

/* this imagines we are at 320M clock */
static void system_clock_at_least_us(uint32_t us)
{
	uint32_t i = 0;

	for(i = 0; i < us * 32; i++) {
		__asm__ volatile (".rept 10 ; nop ; .endr");
	}
}

static void system_clock_trim_32M(void)
{
	uint32_t tmpVal = 0;
	uint32_t trim = 0;
	const struct device *efuse = DEVICE_DT_GET(DT_NODELABEL(efuse));


	tmpVal = eeprom_read(efuse, 0x7C, &trim, 4);
	if (tmpVal < 0) {
		printk("Error: Couldn't read efuses: err: %d.\n", tmpVal);
		return;
	}
	/* TODO: check trim parity */
	trim = (trim & 0xFF0) >> 4;
	tmpVal = sys_read32(PDS_BASE + PDS_RC32M_CTRL0_OFFSET);
	tmpVal = (tmpVal & PDS_RC32M_EXT_CODE_EN_UMSK) | 1 << PDS_RC32M_EXT_CODE_EN_POS;
	sys_write32(tmpVal, PDS_BASE + PDS_RC32M_CTRL0_OFFSET);
	system_clock_settle();

	tmpVal = sys_read32(PDS_BASE + PDS_RC32M_CTRL2_OFFSET);
	tmpVal = (tmpVal & PDS_RC32M_CODE_FR_EXT2_UMSK) | trim << PDS_RC32M_CODE_FR_EXT2_POS;
	sys_write32(tmpVal, PDS_BASE + PDS_RC32M_CTRL2_OFFSET);

	tmpVal = sys_read32(PDS_BASE + PDS_RC32M_CTRL2_OFFSET);
	tmpVal = (tmpVal & PDS_RC32M_EXT_CODE_SEL_UMSK) | 1 << PDS_RC32M_EXT_CODE_SEL_POS;
	sys_write32(tmpVal, PDS_BASE + PDS_RC32M_CTRL2_OFFSET);
	system_clock_settle();
}

static uint32_t system_get_xtal(void)
{
	uint32_t tmpVal;
	tmpVal = sys_read32(HBN_BASE + HBN_RSV3_OFFSET);
	tmpVal &= 0xF;

	switch (tmpVal) {
	case 0:
		return 0;
	case 1:
		return 24 * 1000 * 1000;
	case 2:
		return 32 * 1000 * 1000;
	case 3:
		return 38.4 * 1000 * 1000;
	case 4:
		return 40 * 1000 * 1000;
	case 5:
		return 26 * 1000 * 1000;
	case 6:
		return 32 * 1000 * 1000;
	default:
		return 0;
	}
}

/* source for most clocks, either XTAL or RC32M */
static uint32_t system_get_xclk(void)
{
	uint32_t tmpVal = 0;
	tmpVal = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmpVal &= HBN_ROOT_CLK_SEL_MSK;
	tmpVal = tmpVal >> HBN_ROOT_CLK_SEL_POS;
	tmpVal &= 1;
	if (tmpVal == 0) {
		return (32 * 1000 * 1000);
	} else if (tmpVal == 1) {
		return system_get_xtal();
	} else {
		return 0;
	}
}


/* Almost always CPU, AXI bus, SRAM Memory, Cache, use HCLK query instead */
static uint32_t system_get_fclk(void)
{
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmpVal &= HBN_ROOT_CLK_SEL_MSK;
	tmpVal = (tmpVal >> HBN_ROOT_CLK_SEL_POS) >> 1;
	tmpVal &= 1;

	if (tmpVal == 0) {
		return system_get_xclk();
	} else if (tmpVal == 1) {
		tmpVal = sys_read32(PDS_BASE + PDS_CPU_CORE_CFG1_OFFSET);
		tmpVal = (tmpVal & PDS_REG_PLL_SEL_MSK) >> PDS_REG_PLL_SEL_POS;
		if (tmpVal == 3) {
			return 320 * 1000 * 1000;
		} else if (tmpVal == 2) {
			return 240 * 1000 * 1000;
		} else if (tmpVal == 1) {
			/* TODO AUPLL DIV 1 */
		} else if (tmpVal == 0) {
			/* TODO AUPLL DIV 2 */
		}
	}
	return 0;
}

/* also CPU, AXI bus, SRAM Memory, Cache */
static uint32_t system_get_hclk(void)
{
	uint32_t tmpVal = 0;
	uint32_t clock = 0;

	tmpVal = sys_read32(GLB_BASE + GLB_SYS_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_REG_HCLK_DIV_MSK) >> GLB_REG_HCLK_DIV_POS;
	clock = system_get_fclk();
	return clock / (tmpVal + 1);
}

/* most peripherals clock */
static uint32_t system_get_bclk(void)
{
	uint32_t tmpVal = 0;
	uint32_t clock = 0;

	tmpVal = sys_read32(GLB_BASE + GLB_SYS_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_REG_BCLK_DIV_MSK) >> GLB_REG_BCLK_DIV_POS;
	clock = system_get_hclk();
	return clock / (tmpVal + 1);
}

/* 32 Mhz Oscillator: 0
 * crystal: 1
 * PLL and 32M: 2
 * PLL and crystal: 3
 */
static void system_set_root_clock(uint32_t clock)
{
	uint32_t tmpVal = 0;

	/* invalid value, fallback to internal 32M */
	if (clock < 0 || clock > 3) {
		clock = 0;
	}
	tmpVal = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmpVal = (tmpVal & HBN_ROOT_CLK_SEL_UMSK) | (clock << HBN_ROOT_CLK_SEL_POS);
	sys_write32(tmpVal, HBN_BASE + HBN_GLB_OFFSET);

	system_clock_settle();
}


static uint32_t system_get_root_clock(void)
{
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	return (((tmpVal & HBN_ROOT_CLK_SEL_MSK) >> HBN_ROOT_CLK_SEL_POS) & 0x3);
}

/* /!\ on bl61x hclk is only for CLIC
 * FCLK is the core clock
 */
static int system_set_root_clock_dividers(uint32_t hclk_div, uint32_t bclk_div)
{
	uint32_t tmpVal = 0;
	uint32_t old_rootclk = 0;
	int count = CLOCK_TIMEOUT;

	old_rootclk = system_get_root_clock();

	/* security RC32M */
	if (old_rootclk > 1) {
		system_set_root_clock(0);
	}

	/* set dividers */
	tmpVal = sys_read32(GLB_BASE + GLB_SYS_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_REG_HCLK_DIV_UMSK) | (hclk_div << GLB_REG_HCLK_DIV_POS);
	tmpVal = (tmpVal & GLB_REG_BCLK_DIV_UMSK) | (bclk_div << GLB_REG_BCLK_DIV_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_SYS_CFG0_OFFSET);

	tmpVal = sys_read32(GLB_BASE + GLB_SYS_CFG1_OFFSET);
	tmpVal = (tmpVal & GLB_REG_BCLK_DIV_ACT_PULSE_UMSK) | (1 << GLB_REG_BCLK_DIV_ACT_PULSE_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_SYS_CFG1_OFFSET);


	do {
		tmpVal = sys_read32(GLB_BASE + GLB_SYS_CFG1_OFFSET);
		tmpVal &= GLB_STS_BCLK_PROT_DONE_MSK;
		tmpVal = tmpVal >> GLB_STS_BCLK_PROT_DONE_POS;
		count--;
	} while (count > 0 && tmpVal == 0);

	system_set_root_clock(old_rootclk);
	system_clock_settle();

	if (count < 1) {
		return -1;
	}

	return 0;
}

static void system_set_machine_timer_clock_enable(uint32_t enable)
{
	uint32_t tmpVal = 0;

	if (enable > 1) {
		enable = 1;
	}
	tmpVal = sys_read32(MCU_MISC_BASE + MCU_MISC_MCU_E907_RTC_OFFSET);
	tmpVal = (tmpVal & MCU_MISC_REG_MCU_RTC_EN_UMSK) | (enable << MCU_MISC_REG_MCU_RTC_EN_POS);
	sys_write32(tmpVal, MCU_MISC_BASE + MCU_MISC_MCU_E907_RTC_OFFSET);
}

/* clock:
 * 0: XCLK (RC32M or XTAL)
 * 1: Root Clock (FCLK: RC32M, XTAL or PLLs)
 */

static void system_set_machine_timer_clock(uint32_t enable, uint32_t clock, uint32_t divider)
{
	uint32_t tmpVal = 0;

	if (clock > 1) {
		clock = 0;
	}

	tmpVal = sys_read32(MCU_MISC_BASE + MCU_MISC_MCU_E907_RTC_OFFSET);
	tmpVal = (tmpVal & MCU_MISC_REG_MCU_RTC_CLK_SEL_UMSK)
		| (clock << MCU_MISC_REG_MCU_RTC_CLK_SEL_POS);
	sys_write32(tmpVal, MCU_MISC_BASE + MCU_MISC_MCU_E907_RTC_OFFSET);

	/* disable first, then set div */
	system_set_machine_timer_clock_enable(0);

	tmpVal = sys_read32(MCU_MISC_BASE + MCU_MISC_MCU_E907_RTC_OFFSET);
	tmpVal = (tmpVal & MCU_MISC_REG_MCU_RTC_DIV_UMSK)
		| ((divider & 0x3FF) << MCU_MISC_REG_MCU_RTC_DIV_POS);
	sys_write32(tmpVal, MCU_MISC_BASE + MCU_MISC_MCU_E907_RTC_OFFSET);

	system_set_machine_timer_clock_enable(enable);
}

static int system_init_crystal(void)
{
	uint32_t tmpVal = 0;
	uint32_t old_rootclk = 0;
	int count = CLOCK_TIMEOUT;

	old_rootclk = system_get_root_clock();
	system_set_root_clock(0);

	/* power crystal */
	tmpVal = sys_read32(AON_BASE + AON_RF_TOP_AON_OFFSET);
	tmpVal = (tmpVal & AON_PU_XTAL_AON_UMSK) | ((uint32_t)(1) << AON_PU_XTAL_AON_POS);
	tmpVal = (tmpVal & AON_PU_XTAL_BUF_AON_UMSK) | ((uint32_t)(1) <<
AON_PU_XTAL_BUF_AON_POS);
	sys_write32(tmpVal, AON_BASE + AON_RF_TOP_AON_OFFSET);

	/* wait for crystal to be powered on */
	do {
		system_clock_settle();
		tmpVal = sys_read32(AON_BASE + AON_TSEN_OFFSET);
		count--;
	} while (!(tmpVal & AON_XTAL_RDY_MSK) && count > 0);

	system_set_root_clock(old_rootclk);
	system_clock_settle();
	if (count < 1) {
		return -1;
	}
	return 0;
}

/* No Crystal / RC32M : 0
 * 24M: 1
 * 26M: 5
 * 32M: 2
 * 38P4M: 3
 * 40M: 4
 */
static void system_set_crystal(uint32_t crystal)
{
	uint32_t tmpVal = 0;

	if (crystal == 0 || crystal > 6) {
		crystal = 6;
	}
	tmpVal = sys_read32(HBN_BASE + HBN_RSV3_OFFSET);
	tmpVal = (tmpVal & ~0xF0) | (0x8 << 4);
	tmpVal = (tmpVal & ~0xF) | (crystal << 0);
	sys_write32(tmpVal, HBN_BASE + HBN_RSV3_OFFSET);
}

static void system_deinit_WIFIPLL(void)
{
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmpVal &= GLB_PU_WIFIPLL_UMSK;
	tmpVal &= GLB_PU_WIFIPLL_SFREG_UMSK;
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
}

/* RC32M : 0
 * XTAL : 1
 */
static void system_set_crystal_PLL_reference(uint32_t ref)
{
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG1_OFFSET);
	if (ref == 0) {
		tmpVal = (tmpVal & GLB_WIFIPLL_REFCLK_SEL_UMSK) | (3 << GLB_WIFIPLL_REFCLK_SEL_POS);
	} else if (ref == 1) {
		tmpVal = (tmpVal & GLB_WIFIPLL_REFCLK_SEL_UMSK) | (1 << GLB_WIFIPLL_REFCLK_SEL_POS);
	}
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG1_OFFSET);
}


static void system_init_WIFIPLL_setup(const bl61x_pll_config *const config)
{
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG1_OFFSET);
	tmpVal = (tmpVal & GLB_WIFIPLL_REFDIV_RATIO_UMSK)
	| (config->pllRefdivRatio << GLB_WIFIPLL_REFDIV_RATIO_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG1_OFFSET);

	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG2_OFFSET);
	tmpVal = (tmpVal & GLB_WIFIPLL_INT_FRAC_SW_UMSK)
	| (config->pllIntFracSw << GLB_WIFIPLL_INT_FRAC_SW_POS);
	tmpVal = (tmpVal & GLB_WIFIPLL_ICP_1U_UMSK)
	| (config->pllIcp1u << GLB_WIFIPLL_ICP_1U_POS);
	tmpVal = (tmpVal & GLB_WIFIPLL_ICP_5U_UMSK)
	| (config->pllIcp5u << GLB_WIFIPLL_ICP_5U_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG2_OFFSET);

	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG3_OFFSET);
	tmpVal = (tmpVal & GLB_WIFIPLL_RZ_UMSK)
	| (config->pllRz << GLB_WIFIPLL_RZ_POS);
	tmpVal = (tmpVal & GLB_WIFIPLL_CZ_UMSK)
	| (config->pllCz << GLB_WIFIPLL_CZ_POS);
	tmpVal = (tmpVal & GLB_WIFIPLL_C3_UMSK)
	| (config->pllC3 << GLB_WIFIPLL_C3_POS);
	tmpVal = (tmpVal & GLB_WIFIPLL_R4_SHORT_UMSK)
	| (config->pllR4Short << GLB_WIFIPLL_R4_SHORT_POS);
	tmpVal = (tmpVal & GLB_WIFIPLL_C4_EN_UMSK)
	| (config->pllC4En << GLB_WIFIPLL_C4_EN_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG3_OFFSET);

	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG4_OFFSET);
	tmpVal = (tmpVal & GLB_WIFIPLL_SEL_SAMPLE_CLK_UMSK)
	| (config->pllSelSampleClk << GLB_WIFIPLL_SEL_SAMPLE_CLK_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG4_OFFSET);

	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG5_OFFSET);
	tmpVal = (tmpVal & GLB_WIFIPLL_VCO_SPEED_UMSK)
	| (config->pllVcoSpeed << GLB_WIFIPLL_VCO_SPEED_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG5_OFFSET);

	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG6_OFFSET);
	tmpVal = (tmpVal & GLB_WIFIPLL_SDM_CTRL_HW_UMSK)
	| (config->pllSdmCtrlHw << GLB_WIFIPLL_SDM_CTRL_HW_POS);
	tmpVal = (tmpVal & GLB_WIFIPLL_SDM_BYPASS_UMSK)
	| (config->pllSdmBypass << GLB_WIFIPLL_SDM_BYPASS_POS);
	tmpVal = (tmpVal & GLB_WIFIPLL_SDMIN_UMSK)
	| (config->pllSdmin << GLB_WIFIPLL_SDMIN_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG6_OFFSET);

	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);
	tmpVal = (tmpVal & GLB_USBPLL_SDMIN_UMSK)
	| (0x28000 << GLB_USBPLL_SDMIN_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);

	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG12_OFFSET);
	tmpVal = (tmpVal & GLB_SSCDIV_SDMIN_UMSK)
	| (0x28000 << GLB_SSCDIV_SDMIN_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG12_OFFSET);

	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_PU_WIFIPLL_SFREG_UMSK)
	| (1 << GLB_PU_WIFIPLL_SFREG_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);

	system_clock_at_least_us(5);

	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_PU_WIFIPLL_UMSK)
	| (1 << GLB_PU_WIFIPLL_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);

	system_clock_at_least_us(5);

	/* 'SDM reset' */
	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_WIFIPLL_SDM_RSTB_UMSK)
	| (1 << GLB_WIFIPLL_SDM_RSTB_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	system_clock_at_least_us(5);
	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_WIFIPLL_SDM_RSTB_UMSK)
	| (0 << GLB_WIFIPLL_SDM_RSTB_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	system_clock_at_least_us(5);
	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_WIFIPLL_SDM_RSTB_UMSK)
	| (1 << GLB_WIFIPLL_SDM_RSTB_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);

	/* 'pll reset' */
	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_WIFIPLL_FBDV_RSTB_UMSK)
	| (1 << GLB_WIFIPLL_FBDV_RSTB_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	system_clock_at_least_us(5);
	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_WIFIPLL_FBDV_RSTB_UMSK)
	| (0 << GLB_WIFIPLL_FBDV_RSTB_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	system_clock_at_least_us(5);
	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_WIFIPLL_FBDV_RSTB_UMSK)
	| (1 << GLB_WIFIPLL_FBDV_RSTB_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG0_OFFSET);

	/* enable PLL outputs */
	tmpVal = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG8_OFFSET);
	tmpVal = (tmpVal & GLB_WIFIPLL_EN_DIV3_UMSK)
	| (1 << GLB_WIFIPLL_EN_DIV3_POS);
	tmpVal = (tmpVal & GLB_WIFIPLL_EN_DIV4_UMSK)
	| (1 << GLB_WIFIPLL_EN_DIV4_POS);
	tmpVal = (tmpVal & GLB_WIFIPLL_EN_DIV5_UMSK)
	| (1 << GLB_WIFIPLL_EN_DIV5_POS);
	tmpVal = (tmpVal & GLB_WIFIPLL_EN_DIV6_UMSK)
	| (1 << GLB_WIFIPLL_EN_DIV6_POS);
	tmpVal = (tmpVal & GLB_WIFIPLL_EN_DIV8_UMSK)
	| (1 << GLB_WIFIPLL_EN_DIV8_POS);
	tmpVal = (tmpVal & GLB_WIFIPLL_EN_DIV10_UMSK)
	| (1 << GLB_WIFIPLL_EN_DIV10_POS);
	tmpVal = (tmpVal & GLB_WIFIPLL_EN_DIV12_UMSK)
	| (1 << GLB_WIFIPLL_EN_DIV12_POS);
	tmpVal = (tmpVal & GLB_WIFIPLL_EN_DIV20_UMSK)
	| (1 << GLB_WIFIPLL_EN_DIV20_POS);
	tmpVal = (tmpVal & GLB_WIFIPLL_EN_DIV30_UMSK)
	| (1 << GLB_WIFIPLL_EN_DIV30_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_WIFI_PLL_CFG8_OFFSET);

	system_clock_at_least_us(50);
}

/* No Crystal / RC32M : 0
 * 24M: 1
 * 26M: 5
 * 32M: 2
 * 38P4M: 3
 * 40M: 4
 */
static void system_init_WIFIPLL(uint32_t crystal)
{
	uint32_t tmpVal = 0;
	uint32_t old_rootclk = 0;
	int count = CLOCK_TIMEOUT;

	old_rootclk = system_get_root_clock();

	/* security RC32M */
	if (old_rootclk > 1) {
		system_set_root_clock(0);
	}

	system_set_crystal(crystal);

	system_deinit_WIFIPLL();

	if (crystal == 0) {
		system_set_crystal_PLL_reference(0);
	} else {
		system_set_crystal_PLL_reference(1);
	}

	system_init_WIFIPLL_setup(bl61x_pll_configs[crystal]);

	/* enable PLL clock */
	tmpVal = sys_read32(GLB_BASE + GLB_SYS_CFG0_OFFSET);
	tmpVal |= GLB_REG_PLL_EN_MSK;
	sys_write32(tmpVal, GLB_BASE + GLB_SYS_CFG0_OFFSET);

	system_set_root_clock(old_rootclk);
	system_clock_settle();
}


static void system_init_AUPLL(uint32_t crystal)
{

}
/*
 * AUPLL   DIV1: 1
 * AUPLL   DIV2: 0
 * WIFIPLL 240Mhz: 2
 * WIFIPLL 320Mhz: 3
 */
static void system_set_PLL(uint8_t pll)
{
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(PDS_BASE + PDS_CPU_CORE_CFG1_OFFSET);
	tmpVal = (tmpVal & PDS_REG_PLL_SEL_UMSK) | (pll << PDS_REG_PLL_SEL_POS);
	sys_write32(tmpVal, PDS_BASE + PDS_CPU_CORE_CFG1_OFFSET);
}

/* 'just for safe' */

/* ISP WIFIPLL 80M : 2
 * ISP AUPLL DIV5 : 3
 * ISP AUPLL DIV6 : 4
 * TOP AUPLL DIV5 : 5
 * TOP AUPLL DIV6 : 6
 * PSRAMB WIFIPLL 320M : 7
 * PSRAMB AUPLL DIV1 : 8
 * TOP WIFIPLL 240M : 13
 * TOP WIFIPLL 320M : 14
 * TOP AUPLL DIV2 : 15
 * TOP AUPLL DIV1 : 16
 */

static void system_ungate_pll(uint8_t pll)
{
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(PDS_BASE + GLB_CGEN_CFG3_OFFSET);
	tmpVal |= (1 << pll);
	sys_write32(tmpVal, PDS_BASE + GLB_CGEN_CFG3_OFFSET);
}

/* Frequency Source:
 * No Crystal/ RC32M: 0
 * 24M: 1
 * 26M: 5
 * 32M: 2
 * 38P4M: 3
 * 40M: 4
 *
 * /!\ When Frequency Source is 32M, we do not power crystal
 *
 * Clock Frequency:
 * Crystal: 0
 * AUPLL   DIV1: 1
 * AUPLL   DIV2: 2
 * WIFIPLL 240Mhz: 3
 * WIFIPLL 320Mhz: 4
 * 32MHz Oscillator : 32
 *
 *  /!\ When Clock Frequency is 32M, we do not power crystal or PLL
 */
static int system_init_root_clock(uint32_t crystal, uint32_t clock_frequency_source)
{
	uint32_t tmpVal = 0;

	if (clock_frequency_source == 1 || clock_frequency_source == 2) {
		return -ENOTSUP;
	}

	if (crystal > 5) {
		return -EINVAL;
	}

	if (clock_frequency_source > 4 && clock_frequency_source != 32) {
		return -EINVAL;
	}

	/* make sure all clocks are enabled */
	tmpVal = sys_read32(GLB_BASE + GLB_SYS_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_REG_BCLK_EN_UMSK) | ((uint32_t)(1) << GLB_REG_BCLK_EN_POS);
	tmpVal = (tmpVal & GLB_REG_HCLK_EN_UMSK) | ((uint32_t)(1) << GLB_REG_HCLK_EN_POS);
	tmpVal = (tmpVal & GLB_REG_FCLK_EN_UMSK) | ((uint32_t)(1) << GLB_REG_FCLK_EN_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_SYS_CFG0_OFFSET);

	/* set clock to internal 32MHz Oscillator as failsafe */
	system_set_root_clock(0);
	if (system_set_root_clock_dividers(0, 0) != 0) {
		return -1;
	}

	if (crystal != 0) {
		if (system_init_crystal() != 0) {
			return -1;
		}
	}

	switch (clock_frequency_source) {
	case 32:
		system_set_root_clock_dividers(0, 0);
		system_set_root_clock(0);
		break;
	case 0:
		system_set_root_clock_dividers(0, 0);
		system_set_root_clock(1);
		break;
	case 2:
		system_init_AUPLL(crystal);
		system_set_PLL(0);
		if (system_set_root_clock_dividers(0, 2) != 0) {
			return -1;
		}
		/* 2T rom access goes here */
		system_ungate_pll(15);
		if (crystal == 0) {
			system_set_root_clock(2);
		} else {
			system_set_root_clock(3);
		}
		break;
	case 1:
		system_init_AUPLL(crystal);
		system_set_PLL(1);
		if (system_set_root_clock_dividers(1, 2) != 0) {
			return -1;
		}
		/* 2T rom access goes here */
		system_ungate_pll(16);
		if (crystal == 0) {
			system_set_root_clock(2);
		} else {
			system_set_root_clock(3);
		}
		break;
	case 3:
		system_init_WIFIPLL(crystal);
		system_set_PLL(2);
		if (system_set_root_clock_dividers(0, 2) != 0) {
			return -1;
		}
		/* 2T rom access goes here */
		system_ungate_pll(13);
		if (crystal == 0) {
			system_set_root_clock(2);
		} else {
			system_set_root_clock(3);
		}
		break;
	case 4:
		system_init_WIFIPLL(crystal);
		system_set_PLL(3);
		if (system_set_root_clock_dividers(0, 3) != 0) {
			return -1;
		}
		/* 2T rom access goes here */
		system_ungate_pll(14);
		if (crystal == 0) {
			system_set_root_clock(2);
		} else {
			system_set_root_clock(3);
		}
		break;
	default:
		break;
	}

	system_clock_settle();
	return 0;
}

static uint32_t mtimer_get_xclk_src_div(void)
{
	uint32_t xclk_div = -1;

	return (system_get_xclk() / 1000 / 1000 - 1);
}

static void system_uart_set_clock_enable(uint32_t enable)
{
	uint32_t tmpVal = 0;

	if (enable > 1) {
		enable = 1;
	}
	tmpVal = sys_read32(GLB_BASE + GLB_UART_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_UART_CLK_EN_UMSK) | (enable << GLB_UART_CLK_EN_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_UART_CFG0_OFFSET);
}

/* Clock:
 * BCLK: 0
 * 160 Mhz PLL: 1
 * XCLK: 2
 */
static void system_uart_set_clock(uint32_t enable, uint32_t clock, uint32_t divider)
{
	uint32_t tmpVal = 0;

	if (divider > 0x7) {
		divider = 0x7;
	}
	if (clock > 2) {
		clock = 2;
	}
	/* disable uart clock */
	system_uart_set_clock_enable(0);


	tmpVal = sys_read32(GLB_BASE + GLB_UART_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_UART_CLK_DIV_UMSK) | (divider << GLB_UART_CLK_DIV_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_UART_CFG0_OFFSET);

	tmpVal = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	if (clock < 2) {
		tmpVal = (tmpVal & HBN_UART_CLK_SEL_UMSK) | (clock << HBN_UART_CLK_SEL_POS);
		tmpVal = (tmpVal & HBN_UART_CLK_SEL2_UMSK) | (0 << HBN_UART_CLK_SEL2_POS);
	} else {
		tmpVal = (tmpVal & HBN_UART_CLK_SEL_UMSK) | (0 << HBN_UART_CLK_SEL_POS);
		tmpVal = (tmpVal & HBN_UART_CLK_SEL2_UMSK) | (1 << HBN_UART_CLK_SEL2_POS);
	}
	sys_write32(tmpVal, HBN_BASE + HBN_GLB_OFFSET);

	system_uart_set_clock_enable(enable);
}

/* TODO: should take crystal type, defaults to 40Mhz crystal for BL616 as seems the most common */
static void system_clock_init(void)
{
#if 1
	system_init_root_clock(0, 32);
	system_set_root_clock_dividers(0, 0);
	system_clock_trim_32M();
#else
	system_init_root_clock(4, 4);
#endif
	system_set_machine_timer_clock(1, 0, mtimer_get_xclk_src_div());
}

static void peripheral_clock_init(void)
{
	uint32_t regval = sys_read32(GLB_BASE + GLB_CGEN_CFG1_OFFSET);
	/* enable ADC clock routing */
	regval |= (1 << 2);
	/* enable UART0 clock routing */
	regval |= (1 << 16);
	/* enable UART1 clock routing */
	regval |= (1 << 17);
	/* enable I2C0 clock routing */
	regval |= (1 << 19);
	sys_write32(regval, GLB_BASE + GLB_CGEN_CFG1_OFFSET);
	system_uart_set_clock(1, 2, 0);
}


#ifdef CONFIG_RISCV_GP
ulong_t __soc_get_gp_initial_value(void)
{
	extern uint32_t __global_pointer$;
	return (ulong_t)&__global_pointer$;
}
#endif


static void clean_dcache(void)
{
	__asm__ volatile (
		"fence\n"
		/* th.dcache.call*/
		".insn 0x10000B\n"
		"fence\n"
	);
}

static void clean_icache(void)
{
	__asm__ volatile (
		"fence\n"
		"fence.i\n"
		/* th.icache.iall */
		".insn 0x100000B\n"
		"fence\n"
		"fence.i\n"
	);
}

static void enable_icache(void)
{
	uint32_t tmpVal = 0;

	__asm__ volatile (
		"fence\n"
		"fence.i\n"
		/* th.icache.iall */
		".insn 0x100000B\n"
	);
	__asm__ volatile(
		"csrr %0, 0x7C1"
		: "=r"(tmpVal));
	tmpVal |= (1 << 0);
	__asm__ volatile(
		"csrw 0x7C1, %0"
		:
		: "r"(tmpVal));
	__asm__ volatile (
		"fence\n"
		"fence.i\n"
	);
}

static void enable_dcache(void)
{
	uint32_t tmpVal = 0;

	__asm__ volatile (
		"fence\n"
		"fence.i\n"
		/* th.dcache.iall */
		".insn 0x20000B\n"
	);
	__asm__ volatile(
		"csrr %0, 0x7C1"
		: "=r"(tmpVal));
	tmpVal |= (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 12);
	__asm__ volatile(
		"csrw 0x7C1, %0"
		:
		: "r"(tmpVal));
	__asm__ volatile (
		"fence\n"
		"fence.i\n"
	);
}

static void enable_thead_isa_ext(void)
{
	uint32_t tmpVal = 0;

	__asm__ volatile(
		"csrr %0, 0x7C0"
		: "=r"(tmpVal));
	tmpVal |= (1 << 22);
	__asm__ volatile(
		"csrw 0x7C0, %0"
		:
		: "r"(tmpVal));
}

static void disable_interrupt_autostacking(void)
{
	uint32_t tmpVal = 0;

	__asm__ volatile(
		"csrr %0, 0x7E1"
		: "=r"(tmpVal));
	tmpVal &= ~(0x3 << 16);
	__asm__ volatile(
		"csrw 0x7E1, %0"
		:
		: "r"(tmpVal));
}

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */

static int bl61x_riscv_init(void)
{
	uint32_t key;
	uint32_t tmpVal = 0;

	key = irq_lock();

	/* turn off USB power */
	sys_write32((1 << 5), PDS_BASE + PDS_USB_CTL_OFFSET);
	sys_write32(0, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	/* here go FPU manage if needed */
	/* enable unaligned access error goes here */
	/* supplementary clic init goes here */

	enable_thead_isa_ext();
	enable_dcache();
	enable_icache();
	disable_interrupt_autostacking();
	clean_dcache();

	/* reset uart signals */
	sys_write32(0xffffffff, GLB_BASE + GLB_UART_CFG1_OFFSET);
	sys_write32(0x0000ffff, GLB_BASE + GLB_UART_CFG2_OFFSET);

	/* TODO: 'em' config for ble goes here */

	bl61x_BOD_init();

	system_clock_init();
	peripheral_clock_init();

	irq_unlock(key);

	/* wait 10 ms for peripherals to be ready */
	k_timepoint_t end_timeout = sys_timepoint_calc(K_MSEC(10));

	while (!sys_timepoint_expired(end_timeout)) {
	}

	return 0;
}


SYS_INIT(bl61x_riscv_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
