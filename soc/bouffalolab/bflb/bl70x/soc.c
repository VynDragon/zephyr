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

	tmpVal = sys_read32(HBN_BASE + HBN_MISC_OFFSET);
	/* borThreshold = 1 */
	tmpVal = (tmpVal & HBN_BOR_VTH_UMSK) | ((uint32_t)(1) << HBN_BOR_VTH_POS);
	/* enablePorInBor true*/
	tmpVal = (tmpVal & HBN_BOR_SEL_UMSK) | ((uint32_t)(1) << HBN_BOR_SEL_POS);
	/* enableBor true*/
	tmpVal = (tmpVal & HBN_PU_BOR_UMSK) | ((uint32_t)(1) << HBN_PU_BOR_POS);
	sys_write32(tmpVal, HBN_BASE + HBN_MISC_OFFSET);


	/* enableBorInt false */
	tmpVal = sys_read32(HBN_BASE + HBN_IRQ_MODE_OFFSET);
	tmpVal = tmpVal & HBN_IRQ_BOR_EN_UMSK;
	sys_write32(tmpVal, HBN_BASE + HBN_IRQ_MODE_OFFSET);
}

static uint32_t mtimer_get_clk_src_div(void)
{
	uint32_t bclk_div = -1;

	bclk_div = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	bclk_div = (bclk_div & GLB_REG_BCLK_DIV_MSK) >> GLB_REG_BCLK_DIV_POS;
	return ((sys_read32(CORECLOCKREGISTER) / (bclk_div + 1)) / 1000 / 1000 - 1);
}

static void system_clock_settle(void)
{
	__asm__ volatile (".rept 15 ; nop ; .endr");
}


static void system_clock_delay_32M_ms(uint32_t ms)
{
	uint32_t count = 0;

	do {
		__asm__ volatile (".rept 32 ; nop ; .endr");
		count++;
	} while (count < ms);
}

static void system_clock_trim_32M(void)
{
	uint32_t tmpVal = 0;
	uint32_t trim = 0;
	const struct device *efuse = DEVICE_DT_GET(DT_NODELABEL(efuse));


	tmpVal = eeprom_read(efuse, 0xC, &trim, 4);
	if (tmpVal < 0) {
		printk("Error: Couldn't read efuses: err: %d.\n", tmpVal);
		return;
	}
	/* TODO: check trim parity */
	trim = (trim & 0x3FC00) >> 10;
	tmpVal = sys_read32(PDS_BASE + PDS_RC32M_CTRL0_OFFSET);
	tmpVal = (tmpVal & PDS_RC32M_EXT_CODE_EN_UMSK) | 1 << PDS_RC32M_EXT_CODE_EN_POS;
	tmpVal = (tmpVal & PDS_RC32M_CODE_FR_EXT_UMSK) | trim << PDS_RC32M_CODE_FR_EXT_POS;
	sys_write32(tmpVal, PDS_BASE + PDS_RC32M_CTRL0_OFFSET);
	system_clock_settle();
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

static void system_set_root_clock_dividers(uint32_t hclk_div, uint32_t bclk_div)
{
	uint32_t tmpVal = 0;

	/* set dividers */
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_REG_HCLK_DIV_UMSK) | (hclk_div << GLB_REG_HCLK_DIV_POS);
	tmpVal = (tmpVal & GLB_REG_BCLK_DIV_UMSK) | (bclk_div << GLB_REG_BCLK_DIV_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_CLK_CFG0_OFFSET);

	/* do something undocumented, probably acknowledging clock change by disabling then
	 * reenabling bclk
	 */
	sys_write32(0x00000001, 0x40000FFC);
	sys_write32(0x00000000, 0x40000FFC);

	/* set core clock ?*/
	sys_write32(sys_read32(CORECLOCKREGISTER) / (hclk_div + 1), CORECLOCKREGISTER);

	system_clock_settle();

	/* enable clocks */
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_REG_BCLK_EN_UMSK) | ((uint32_t)(1) << GLB_REG_BCLK_EN_POS);
	tmpVal = (tmpVal & GLB_REG_HCLK_EN_UMSK) | ((uint32_t)(1) << GLB_REG_HCLK_EN_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_CLK_CFG0_OFFSET);

	system_clock_settle();
}

/* HCLK: 0
 * PLL120M: 1
 */

static void system_set_PKA_clock(uint32_t pka_clock)
{
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(GLB_BASE + GLB_SWRST_CFG2_OFFSET);
	tmpVal = (tmpVal & GLB_PKA_CLK_SEL_UMSK) | (pka_clock << GLB_PKA_CLK_SEL_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_SWRST_CFG2_OFFSET);
}


static void system_set_machine_timer_clock_enable(uint32_t enable)
{
	uint32_t tmpVal = 0;

	if (enable > 1) {
		enable = 1;
	}
	tmpVal = sys_read32(GLB_BASE + GLB_CPU_CLK_CFG_OFFSET);
	tmpVal = (tmpVal & GLB_CPU_RTC_EN_UMSK) | (enable << GLB_CPU_RTC_EN_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_CPU_CLK_CFG_OFFSET);
}

/* clock:
 * 0: BCLK
 * 1: 32Khz Oscillator (RC32*K*)
 */

static void system_set_machine_timer_clock(uint32_t enable, uint32_t clock, uint32_t divider)
{
	uint32_t tmpVal = 0;

	if (divider > 0x1FFFF) {
		divider = 0x1FFFF;
	}
	if (clock > 1) {
		clock = 1;
	}
	/* disable mtime clock */
	system_set_machine_timer_clock_enable(0);


	tmpVal = sys_read32(GLB_BASE + GLB_CPU_CLK_CFG_OFFSET);
	tmpVal = (tmpVal & GLB_CPU_RTC_SEL_UMSK) | (clock << GLB_CPU_RTC_SEL_POS);
	tmpVal = (tmpVal & GLB_CPU_RTC_DIV_UMSK) | (divider << GLB_CPU_RTC_DIV_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_CPU_CLK_CFG_OFFSET);

	system_set_machine_timer_clock_enable(enable);
}


/* No Crystal: 0
 * 32M: 1
 * 32MHz Oscillator : 32
 */

/* TODO: function is very long, consider splitting. pds power down and power up good candidate? */
/* TODO: Is PLL just called DLL on BL702? Or is it actually a different system? Why even?
 * at least it's simpler i guess
 */
static void system_setup_DLL(uint32_t crystal)
{
	uint32_t tmpVal = 0;

	/* DLL Off */

	tmpVal = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmpVal = (tmpVal & GLB_PPU_DLL_UMSK) | ((uint32_t)(0) << GLB_PPU_DLL_POS);
	tmpVal = (tmpVal & GLB_PU_DLL_UMSK) | ((uint32_t)(0) << GLB_PU_DLL_POS);
	tmpVal = (tmpVal & GLB_DLL_RESET_UMSK) | ((uint32_t)(1) << GLB_DLL_RESET_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_DLL_OFFSET);

	if (crystal == 32) {
		/* make sure we are on RC32M before trim */
		system_set_root_clock(0);
		system_set_root_clock_dividers(0, 0);
		sys_write32(32 * 1000 * 1000, CORECLOCKREGISTER);

		/* Trim RC32M */
		system_clock_trim_32M();

		tmpVal = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
		tmpVal = (tmpVal & GLB_DLL_REFCLK_SEL_UMSK) | ((uint32_t)(0) <<
GLB_DLL_REFCLK_SEL_POS);
		sys_write32(tmpVal, GLB_BASE + GLB_DLL_OFFSET);

	} else {
		tmpVal = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
		tmpVal = (tmpVal & GLB_DLL_REFCLK_SEL_UMSK) | ((uint32_t)(1) <<
GLB_DLL_REFCLK_SEL_POS);
		sys_write32(tmpVal, GLB_BASE + GLB_DLL_OFFSET);
	}


	/* init sequence */

	tmpVal = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmpVal = (tmpVal & GLB_DLL_PRECHG_SEL_UMSK) | ((uint32_t)(1) << GLB_DLL_PRECHG_SEL_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_DLL_OFFSET);

	tmpVal = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmpVal = (tmpVal & GLB_PPU_DLL_UMSK) | ((uint32_t)(1) << GLB_PPU_DLL_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_DLL_OFFSET);

	tmpVal = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmpVal = (tmpVal & GLB_PU_DLL_UMSK) | ((uint32_t)(1) << GLB_PU_DLL_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_DLL_OFFSET);

	tmpVal = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmpVal = (tmpVal & GLB_DLL_RESET_UMSK) | ((uint32_t)(0) << GLB_DLL_RESET_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_DLL_OFFSET);

	system_clock_settle();

	/* enable all DLL clocks*/
	tmpVal = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmpVal = (tmpVal & GLB_DLL_CLK_57P6M_EN_UMSK) | ((uint32_t)(1) << GLB_DLL_CLK_57P6M_EN_POS);
	tmpVal = (tmpVal & GLB_DLL_CLK_96M_EN_UMSK) | ((uint32_t)(1) << GLB_DLL_CLK_96M_EN_POS);
	tmpVal = (tmpVal & GLB_DLL_CLK_144M_EN_UMSK) | ((uint32_t)(1) << GLB_DLL_CLK_144M_EN_POS);
	tmpVal = (tmpVal & GLB_DLL_CLK_288M_EN_UMSK) | ((uint32_t)(1) << GLB_DLL_CLK_288M_EN_POS);
	tmpVal = (tmpVal & GLB_DLL_CLK_MMDIV_EN_UMSK) | ((uint32_t)(1) << GLB_DLL_CLK_MMDIV_EN_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_DLL_OFFSET);
}


static uint32_t system_uart_bflb_get_crystal_frequency(void)
{
	return (32 * 1000 * 1000);
}

/* Frequency Source:
 * No Crystal: 0
 * 32M: 1
 * 32MHz Oscillator: 32
 *
 * /!\ When Frequency Source is 32M, we do not power crystal
 *
 * Clock Frequency:
 * Crystal: 0
 * DLL 57.6MHz: 1
 * DLL 96Mhz: 2
 * DLL 144Mhz: 3
 * ? DLL 288 Mhz: 4 ?
 * 32MHz Oscillator : 32
 *
 *  /!\ When Clock Frequency is 32M, we do not power crystal or PLL
 */
static void system_init_root_clock(uint32_t crystal, uint32_t clock_frequency_source)
{
	uint32_t tmpVal = 0;
	uint32_t xtal_power_timeout = 0;


	/* make sure all clocks are enabled */
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_REG_BCLK_EN_UMSK) | ((uint32_t)(1) << GLB_REG_BCLK_EN_POS);
	tmpVal = (tmpVal & GLB_REG_HCLK_EN_UMSK) | ((uint32_t)(1) << GLB_REG_HCLK_EN_POS);
	tmpVal = (tmpVal & GLB_REG_FCLK_EN_UMSK) | ((uint32_t)(1) << GLB_REG_FCLK_EN_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_CLK_CFG0_OFFSET);

	/* set clock to internal 32MHz Oscillator as failsafe */
	system_set_root_clock(0);
	system_set_root_clock_dividers(0, 0);
	sys_write32(32 * 1000 * 1000, CORECLOCKREGISTER);

	if (clock_frequency_source == 32) {
		return;
	}

	if (crystal != 32) {
		/* power crystal */
		tmpVal = sys_read32(AON_BASE + AON_RF_TOP_AON_OFFSET);
		tmpVal = (tmpVal & AON_PU_XTAL_AON_UMSK) | ((uint32_t)(1) << AON_PU_XTAL_AON_POS);
		tmpVal = (tmpVal & AON_PU_XTAL_BUF_AON_UMSK) | ((uint32_t)(1) <<
AON_PU_XTAL_BUF_AON_POS);
		sys_write32(tmpVal, AON_BASE + AON_RF_TOP_AON_OFFSET);

		/* wait for crystal to be powered on */
		/* TODO: figure way to communicate this failed */
		do {
			system_clock_delay_32M_ms(1);
			tmpVal = sys_read32(AON_BASE + AON_TSEN_OFFSET);
			xtal_power_timeout++;
		} while (!(tmpVal & AON_XTAL_RDY_MSK) && xtal_power_timeout < 120);
	}

	/* power PLL
	 * This code path only when PLL!
	 */
	system_setup_DLL(crystal);
	/* Big settle, 55us in SDK */
	system_clock_delay_32M_ms(10);


	/* glb enable pll actual? */
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_REG_PLL_EN_UMSK) | ((uint32_t)(1) << GLB_REG_PLL_EN_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_CLK_CFG0_OFFSET);

	/* set PLL to use in GLB*/
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_REG_PLL_SEL_UMSK) | ((uint32_t)(clock_frequency_source - 1) <<
GLB_REG_PLL_SEL_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_CLK_CFG0_OFFSET);

	/* set root clock settings */
	switch (clock_frequency_source) {
	case 32:
		/* we are not supposed to be here */
	break;

	case 0:
		system_set_root_clock(1);
		sys_write32(system_uart_bflb_get_crystal_frequency(), CORECLOCKREGISTER);
	break;

	case 1:
		system_set_root_clock(crystal == 32 ? 2 : 3);
		sys_write32(57 * 1000 * 1000 + 6 * 100 * 1000, CORECLOCKREGISTER);
	break;

	case 2:
		system_set_root_clock_dividers(0, 1);
		system_set_root_clock(crystal == 32 ? 2 : 3);
		sys_write32(96 * 1000 * 1000, CORECLOCKREGISTER);
	break;

	case 3:
		/* TODO: enable rom access 2T*/
		system_set_root_clock_dividers(0, 1);
		system_set_root_clock(crystal == 32 ? 2 : 3);
		sys_write32(144 * 1000 * 1000, CORECLOCKREGISTER);
	break;

	case 4:
		/* TODO: enable rom access 2T*/
		system_set_root_clock_dividers(0, 1);
		system_set_root_clock(crystal == 32 ? 2 : 3);
		sys_write32(288 * 1000 * 1000, CORECLOCKREGISTER);
	break;

	default:
	break;
	}

	/* settle */
	system_clock_delay_32M_ms(10);
}

static void uart_set_clock_enable(uint32_t enable)
{
	uint32_t tmpVal = 0;

	if (enable > 1) {
		enable = 1;
	}
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG2_OFFSET);
	tmpVal = (tmpVal & GLB_UART_CLK_EN_UMSK) | (enable << GLB_UART_CLK_EN_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_CLK_CFG2_OFFSET);
}

/* Clock:
 * FCLK: 0
 * 160 Mhz PLL: 1
 * When using PLL root clock, we can use either setting, when using the 32Mhz Oscillator with a
 * uninitialized PLL, only FCLK will be available.
 */
static void uart_set_clock(uint32_t enable, uint32_t clock, uint32_t divider)
{
	uint32_t tmpVal = 0;

	if (divider > 0x7) {
		divider = 0x7;
	}
	if (clock > 1) {
		clock = 1;
	}
	/* disable uart clock */
	uart_set_clock_enable(0);


	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG2_OFFSET);
	tmpVal = (tmpVal & GLB_UART_CLK_DIV_UMSK) | (divider << GLB_UART_CLK_DIV_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_CLK_CFG2_OFFSET);

	tmpVal = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmpVal = (tmpVal & HBN_UART_CLK_SEL_UMSK) | (clock << HBN_UART_CLK_SEL_POS);
	sys_write32(tmpVal, HBN_BASE + HBN_GLB_OFFSET);

	uart_set_clock_enable(enable);
}


/* TODO: should take crystal type, defaults to 40Mhz crystal for BL602 as seems the most common */
static void system_clock_init(void)
{
#if 0
	system_init_root_clock(0, 32);
	system_set_root_clock_dividers(0, 0);
	system_clock_trim_32M();
#else
	system_init_root_clock(1, 3);
	system_set_root_clock_dividers(0, 1);
#endif
	system_set_machine_timer_clock(1, 0, mtimer_get_clk_src_div());
}

static void peripheral_clock_init(void)
{

	uint32_t regval = sys_read32(0x40000024);
	/* enable UART0 clock routing */
	regval |= (1 << 16);
	/* enable I2C0 clock routing */
	regval |= (1 << 19);
	sys_write32(regval, 0x40000024);
	uart_set_clock(1, 0, 0);
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

static int bl_riscv_init(void)
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


	/* GLB_UART_Sig_Swap_Set(UART_SIG_SWAP_NONE); */
	tmpVal = sys_read32(GLB_BASE + GLB_PARM_OFFSET);
	tmpVal = (tmpVal & GLB_UART_SWAP_SET_UMSK) | ((uint32_t)(0) <<
GLB_UART_SWAP_SET_POS);
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

	system_clock_init();
	peripheral_clock_init();

	irq_unlock(key);

	/* wait 10 ms for peripherals to be ready */
	k_timepoint_t end_timeout = sys_timepoint_calc(K_MSEC(10));

	while (!sys_timepoint_expired(end_timeout)) {
	}

	return 0;
}


SYS_INIT(bl_riscv_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
