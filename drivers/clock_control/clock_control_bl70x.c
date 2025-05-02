/*
 * Copyright (c) 2025 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_bl70x_clock_controller

#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/dt-bindings/clock/bflb_bl70x_clock.h>


#include <bouffalolab/bl70x/bflb_soc.h>
#include <bouffalolab/bl70x/aon_reg.h>
#include <bouffalolab/bl70x/glb_reg.h>
#include <bouffalolab/bl70x/hbn_reg.h>
#include <bouffalolab/bl70x/pds_reg.h>
#include <bouffalolab/bl70x/l1c_reg.h>
#include <bouffalolab/bl70x/extra_defines.h>

#define CLK_SRC_IS(clk, src)                                                                       \
	DT_SAME_NODE(DT_CLOCKS_CTLR_BY_IDX(DT_INST_CLOCKS_CTLR_BY_NAME(0, clk), 0),                \
		     DT_INST_CLOCKS_CTLR_BY_NAME(0, src))

#define CLOCK_TIMEOUT	1024
#define RC32M_FREQ	(32 * 1000 * 1000)

enum bl70x_clkid {
	bl70x_clkid_none = -1,
	bl70x_clkid_clk_root = BL70X_CLKID_CLK_ROOT,
	bl70x_clkid_clk_rc32m = BL70X_CLKID_CLK_RC32M,
	bl70x_clkid_clk_crystal = BL70X_CLKID_CLK_CRYSTAL,
	bl70x_clkid_clk_dll = BL70X_CLKID_CLK_DLL,
	bl70x_clkid_clk_bclk = BL70X_CLKID_CLK_BCLK,
};

struct clock_control_bl70x_dll_config {
	enum bl70x_clkid	source;
	bool			overclock;
};

struct clock_control_bl70x_root_config {
	enum bl70x_clkid	source;
	uint8_t			dll_select;
	uint8_t			divider;
};

struct clock_control_bl70x_bclk_config {
	uint8_t			divider;
};

struct clock_control_bl70x_config {
	uint32_t	crystal_frequency;
};

struct clock_control_bl70x_data {
	bool	crystal_enabled;
	bool	dll_enabled;
	struct clock_control_bl70x_dll_config	dll;
	struct clock_control_bl70x_root_config	root;
	struct clock_control_bl70x_bclk_config	bclk;
};

static void clock_control_bl70x_clock_settle(void)
{
	__asm__ volatile (".rept 15 ; nop ; .endr");
}

/* 32 Mhz Oscillator: 0
 * crystal: 1
 * DLL and 32M: 2
 * DLL and crystal: 3
 */
static void clock_control_bl70x_set_root_clock(uint32_t clock)
{
	uint32_t tmp = 0;

	/* invalid value, fallback to internal 32M */
	if (clock < 0 || clock > 3) {
		clock = 0;
	}
	tmp = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmp = (tmp & HBN_ROOT_CLK_SEL_UMSK) | (clock << HBN_ROOT_CLK_SEL_POS);
	sys_write32(tmp, HBN_BASE + HBN_GLB_OFFSET);

	clock_control_bl70x_clock_settle();
}


static uint32_t clock_control_bl70x_get_root_clock(void)
{
	uint32_t tmp = 0;

	tmp = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	return (((tmp & HBN_ROOT_CLK_SEL_MSK) >> HBN_ROOT_CLK_SEL_POS) & 0x3);
}

static int clock_control_bl70x_init_crystal(void)
{
	uint32_t tmp = 0;
	uint32_t old_rootclk = 0;
	int count = CLOCK_TIMEOUT;

	old_rootclk = clock_control_bl70x_get_root_clock();
	clock_control_bl70x_set_root_clock(0);

	/* power crystal */
	tmp = sys_read32(AON_BASE + AON_RF_TOP_AON_OFFSET);
	tmp = (tmp & AON_PU_XTAL_AON_UMSK) | ((uint32_t)(1) << AON_PU_XTAL_AON_POS);
	tmp = (tmp & AON_PU_XTAL_BUF_AON_UMSK) | ((uint32_t)(1) <<
AON_PU_XTAL_BUF_AON_POS);
	sys_write32(tmp, AON_BASE + AON_RF_TOP_AON_OFFSET);

	/* wait for crystal to be powered on */
	do {
		clock_control_bl70x_clock_settle();
		tmp = sys_read32(AON_BASE + AON_TSEN_OFFSET);
		count--;
	} while (!(tmp & AON_XTAL_RDY_MSK) && count > 0);

	clock_control_bl70x_set_root_clock(old_rootclk);
	clock_control_bl70x_clock_settle();
	if (count < 1) {
		return -1;
	}
	return 0;
}

/* HCLK is the core clock */
static int clock_control_bl70x_set_root_clock_dividers(uint32_t hclk_div, uint32_t bclk_div)
{
	uint32_t tmp = 0;
	uint32_t old_rootclk = 0;

	old_rootclk = clock_control_bl70x_get_root_clock();

	/* security RC32M */
	if (old_rootclk > 1) {
		clock_control_bl70x_set_root_clock(0);
	}

	/* set dividers */
	tmp = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmp = (tmp & GLB_REG_HCLK_DIV_UMSK) | (hclk_div << GLB_REG_HCLK_DIV_POS);
	tmp = (tmp & GLB_REG_BCLK_DIV_UMSK) | (bclk_div << GLB_REG_BCLK_DIV_POS);
	sys_write32(tmp, GLB_BASE + GLB_CLK_CFG0_OFFSET);

	/* do something undocumented, probably acknowledging clock change by disabling then
	 * reenabling bclk
	 */
	sys_write32(0x00000001, 0x40000FFC);
	sys_write32(0x00000000, 0x40000FFC);

	clock_control_bl70x_clock_settle();

	/* enable clocks */
	tmp = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmp = (tmp & GLB_REG_BCLK_EN_UMSK) | ((uint32_t)(1) << GLB_REG_BCLK_EN_POS);
	tmp = (tmp & GLB_REG_HCLK_EN_UMSK) | ((uint32_t)(1) << GLB_REG_HCLK_EN_POS);
	sys_write32(tmp, GLB_BASE + GLB_CLK_CFG0_OFFSET);

	clock_control_bl70x_set_root_clock(old_rootclk);
	clock_control_bl70x_clock_settle();

	return 0;
}

static void clock_control_bl70x_set_machine_timer_clock_enable(uint32_t enable)
{
	uint32_t tmp = 0;

	if (enable > 1) {
		enable = 1;
	}
	tmp = sys_read32(GLB_BASE + GLB_CPU_CLK_CFG_OFFSET);
	tmp = (tmp & GLB_CPU_RTC_EN_UMSK) | (enable << GLB_CPU_RTC_EN_POS);
	sys_write32(tmp, GLB_BASE + GLB_CPU_CLK_CFG_OFFSET);
}

/* clock:
 * 0: BCLK
 * 1: 32Khz Oscillator (RC32*K*)
 */
static void clock_control_bl70x_set_machine_timer_clock(uint32_t enable, uint32_t clock, uint32_t divider)
{
	uint32_t tmp = 0;

	if (divider > 0x1FFFF) {
		divider = 0x1FFFF;
	}
	if (clock > 1) {
		clock = 1;
	}

	/* disable first, then set div */
	clock_control_bl70x_set_machine_timer_clock_enable(0);

	tmp = sys_read32(GLB_BASE + GLB_CPU_CLK_CFG_OFFSET);
	tmp = (tmp & GLB_CPU_RTC_SEL_UMSK) | (clock << GLB_CPU_RTC_SEL_POS);
	tmp = (tmp & GLB_CPU_RTC_DIV_UMSK) | (divider << GLB_CPU_RTC_DIV_POS);
	sys_write32(tmp, GLB_BASE + GLB_CPU_CLK_CFG_OFFSET);

	clock_control_bl70x_set_machine_timer_clock_enable(enable);
}

static void clock_control_bl70x_deinit_dll(void)
{
	uint32_t tmp = 0;

	/* DLL Off */
	tmp = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmp = (tmp & GLB_PPU_DLL_UMSK) | ((uint32_t)(0) << GLB_PPU_DLL_POS);
	tmp = (tmp & GLB_PU_DLL_UMSK) | ((uint32_t)(0) << GLB_PU_DLL_POS);
	tmp = (tmp & GLB_DLL_RESET_UMSK) | ((uint32_t)(1) << GLB_DLL_RESET_POS);
	sys_write32(tmp, GLB_BASE + GLB_DLL_OFFSET);
}

/* RC32M : 0
 * XTAL : 1
 */
static void clock_control_bl70x_set_dll_source(uint32_t source)
{
	uint32_t tmp = 0;

	tmp = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	if (source > 0) {
		tmp = (tmp & GLB_DLL_REFCLK_SEL_UMSK) | ((uint32_t)(1) <<
	GLB_DLL_REFCLK_SEL_POS);
	} else {
		tmp = (tmp & GLB_DLL_REFCLK_SEL_UMSK) | ((uint32_t)(0) <<
	GLB_DLL_REFCLK_SEL_POS);
	}
	sys_write32(tmp, GLB_BASE + GLB_DLL_OFFSET);
}

static void clock_control_bl70x_init_dll(enum bl70x_clkid source, uint32_t crystal_frequency)
{
	uint32_t tmp;
	uint32_t old_rootclk;

	old_rootclk = clock_control_bl70x_get_root_clock();

	/* security RC32M */
	if (old_rootclk > 1) {
		clock_control_bl70x_set_root_clock(0);
	}

	if (crystal_frequency != RC32M_FREQ)
	{
		return;
	}

	clock_control_bl70x_deinit_dll();

	if (source == BL70X_CLKID_CLK_CRYSTAL) {
		clock_control_bl70x_set_dll_source(1);
	} else {
		clock_control_bl70x_set_dll_source(0);
	}

	/* init sequence */
	tmp = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmp = (tmp & GLB_DLL_PRECHG_SEL_UMSK) | ((uint32_t)(1) << GLB_DLL_PRECHG_SEL_POS);
	sys_write32(tmp, GLB_BASE + GLB_DLL_OFFSET);

	tmp = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmp = (tmp & GLB_PPU_DLL_UMSK) | ((uint32_t)(1) << GLB_PPU_DLL_POS);
	sys_write32(tmp, GLB_BASE + GLB_DLL_OFFSET);

	tmp = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmp = (tmp & GLB_PU_DLL_UMSK) | ((uint32_t)(1) << GLB_PU_DLL_POS);
	sys_write32(tmp, GLB_BASE + GLB_DLL_OFFSET);

	tmp = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmp = (tmp & GLB_DLL_RESET_UMSK) | ((uint32_t)(0) << GLB_DLL_RESET_POS);
	sys_write32(tmp, GLB_BASE + GLB_DLL_OFFSET);

	clock_control_bl70x_clock_settle();

	/* enable all DLL clocks*/
	tmp = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmp = (tmp & GLB_DLL_CLK_57P6M_EN_UMSK) | ((uint32_t)(1) << GLB_DLL_CLK_57P6M_EN_POS);
	tmp = (tmp & GLB_DLL_CLK_96M_EN_UMSK) | ((uint32_t)(1) << GLB_DLL_CLK_96M_EN_POS);
	tmp = (tmp & GLB_DLL_CLK_144M_EN_UMSK) | ((uint32_t)(1) << GLB_DLL_CLK_144M_EN_POS);
	tmp = (tmp & GLB_DLL_CLK_288M_EN_UMSK) | ((uint32_t)(1) << GLB_DLL_CLK_288M_EN_POS);
	tmp = (tmp & GLB_DLL_CLK_MMDIV_EN_UMSK) | ((uint32_t)(1) << GLB_DLL_CLK_MMDIV_EN_POS);
	sys_write32(tmp, GLB_BASE + GLB_DLL_OFFSET);

	clock_control_bl70x_set_root_clock(old_rootclk);
	clock_control_bl70x_clock_settle();
}

/*
 * 0: 57.6M
 * 1: 96M
 * 2: 144M
 * 3: 120M (do not use)
 */
static void clock_control_bl70x_select_DLL(uint8_t dll)
{
	uint32_t tmp = 0;

	tmp = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmp = (tmp & GLB_REG_PLL_SEL_UMSK) | (dll << GLB_REG_PLL_SEL_POS);
	sys_write32(tmp, GLB_BASE + GLB_CLK_CFG0_OFFSET);
}

static int clock_control_bl70x_clock_trim_32M(void)
{
	uint32_t tmp = 0;
	uint32_t trim = 0;
	const struct device *efuse = DEVICE_DT_GET_ONE(bflb_efuse);


	tmp = syscon_read_reg(efuse, 0xC, &trim);
	if (tmp < 0) {
		printk("Error: Couldn't read efuses: err: %d.\n", tmp);
		return tmp;
	}
	/* TODO: check trim parity */
	trim = (trim & 0x3FC00) >> 10;
	tmp = sys_read32(PDS_BASE + PDS_RC32M_CTRL0_OFFSET);
	tmp = (tmp & PDS_RC32M_EXT_CODE_EN_UMSK) | 1 << PDS_RC32M_EXT_CODE_EN_POS;
	tmp = (tmp & PDS_RC32M_CODE_FR_EXT_UMSK) | trim << PDS_RC32M_CODE_FR_EXT_POS;
	sys_write32(tmp, PDS_BASE + PDS_RC32M_CTRL0_OFFSET);

	clock_control_bl70x_clock_settle();

	return 0;
}

/* source for most clocks, either XTAL or RC32M */
static uint32_t clock_control_bl70x_get_xclk(const struct device *dev)
{
	uint32_t tmp;
	const struct clock_control_bl70x_config *config = dev->config;

	tmp = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmp &= HBN_ROOT_CLK_SEL_MSK;
	tmp = tmp >> HBN_ROOT_CLK_SEL_POS;
	tmp &= 1;
	if (tmp == 0) {
		return RC32M_FREQ;
	} else if (tmp == 1) {
		return config->crystal_frequency;
	} else {
		return 0;
	}
}

static uint32_t clock_control_bl70x_get_clk(const struct device *dev)
{
	uint32_t tmp;
	uint32_t hclk_div;

	hclk_div = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	hclk_div = (hclk_div & GLB_REG_HCLK_DIV_MSK) >> GLB_REG_HCLK_DIV_POS;

	tmp = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmp &= HBN_ROOT_CLK_SEL_MSK;
	tmp = (tmp >> HBN_ROOT_CLK_SEL_POS) >> 1;
	tmp &= 1;

	if (tmp == 0) {
		return clock_control_bl70x_get_xclk(dev) / (hclk_div + 1);
	} else if (tmp == 1) {
		tmp = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
		tmp = (tmp & GLB_REG_PLL_SEL_MSK) >> GLB_REG_PLL_SEL_POS;
		if (tmp == 3) {
			return 120 * 1000 * 1000 / (hclk_div + 1);
		} else if (tmp == 2) {
			return 144 * 1000 * 1000 / (hclk_div + 1);
		} else if (tmp == 1) {
			return 96 * 1000 * 1000 / (hclk_div + 1);
		} else if (tmp == 0) {
			return 576 * 100 * 1000 / (hclk_div + 1);
		}
	}
	return 0;
}

/* most peripherals clock */
static uint32_t clock_control_bl70x_get_bclk(const struct device *dev)
{
	uint32_t tmp = 0;
	uint32_t clock = 0;

	tmp = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmp = (tmp & GLB_REG_BCLK_DIV_MSK) >> GLB_REG_BCLK_DIV_POS;
	clock = clock_control_bl70x_get_clk(dev);
	return clock / (tmp + 1);
}

static uint32_t clock_control_bl70x_mtimer_get_clk_src_div(const struct device *dev)
{
	return clock_control_bl70x_get_bclk(dev) / 1000 / 1000 - 1;
}

static void clock_control_bl70x_cache_2T(bool yes)
{
	uint32_t tmp = 0;

	tmp = sys_read32(L1C_BASE + L1C_CONFIG_OFFSET);

	if (yes) {
		tmp |= L1C_IROM_2T_ACCESS_MSK;
	} else {
		tmp &= ~L1C_IROM_2T_ACCESS_MSK;
	}

	sys_write32(tmp, L1C_BASE + L1C_CONFIG_OFFSET);
}

static void clock_control_bl70x_init_root_as_dll(const struct device *dev)
{
	struct clock_control_bl70x_data *data = dev->data;
	const struct clock_control_bl70x_config *config = dev->config;
	uint32_t tmp;

	clock_control_bl70x_init_dll(data->dll.source, config->crystal_frequency);

	/* glb enable dll actual? */
	tmp = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmp = (tmp & GLB_REG_PLL_EN_UMSK) | ((uint32_t)(1) << GLB_REG_PLL_EN_POS);
	sys_write32(tmp, GLB_BASE + GLB_CLK_CFG0_OFFSET);

	clock_control_bl70x_select_DLL(data->root.dll_select);

	if (data->dll.source == bl70x_clkid_clk_crystal) {
		clock_control_bl70x_set_root_clock(3);
	} else {
		clock_control_bl70x_set_root_clock(2);
	}

	if (clock_control_bl70x_get_clk(dev) >= 120000000) {
		clock_control_bl70x_cache_2T(true);
	}

	sys_write32(clock_control_bl70x_get_clk(dev), CORECLOCKREGISTER);
}

static void clock_control_bl70x_init_root_as_crystal(const struct device *dev)
{
	clock_control_bl70x_set_root_clock(1);
	sys_write32(clock_control_bl70x_get_clk(dev), CORECLOCKREGISTER);
}

static int clock_control_bl70x_init_root(const struct device *dev)
{
	struct clock_control_bl70x_data *data = dev->data;
	uint32_t tmp;
	int ret;

	/* make sure all clocks are enabled */
	tmp = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmp = (tmp & GLB_REG_BCLK_EN_UMSK) | ((uint32_t)(1) << GLB_REG_BCLK_EN_POS);
	tmp = (tmp & GLB_REG_HCLK_EN_UMSK) | ((uint32_t)(1) << GLB_REG_HCLK_EN_POS);
	tmp = (tmp & GLB_REG_FCLK_EN_UMSK) | ((uint32_t)(1) << GLB_REG_FCLK_EN_POS);
	sys_write32(tmp, GLB_BASE + GLB_CLK_CFG0_OFFSET);

	/* set root clock to internal 32MHz Oscillator as failsafe */
	clock_control_bl70x_set_root_clock(0);
	if (clock_control_bl70x_set_root_clock_dividers(0, 0) != 0) {
		return -EIO;
	}
	sys_write32(RC32M_FREQ, CORECLOCKREGISTER);

	if (data->crystal_enabled) {
		if (clock_control_bl70x_init_crystal() < 0) {
			return -EIO;
		}
	}

	ret = clock_control_bl70x_set_root_clock_dividers(data->root.divider - 1, data->bclk.divider - 1);
	if (ret < 0) {
		return ret;
	}

	if (data->root.source == bl70x_clkid_clk_dll) {
		clock_control_bl70x_init_root_as_dll(dev);
	} else if (data->root.source == bl70x_clkid_clk_crystal) {
		clock_control_bl70x_init_root_as_crystal(dev);
	}

	ret = clock_control_bl70x_clock_trim_32M();
	if (ret < 0) {
		return ret;
	}
	clock_control_bl70x_set_machine_timer_clock(1, 0, clock_control_bl70x_mtimer_get_clk_src_div(dev));

	clock_control_bl70x_clock_settle();

	return ret;
}

static void clock_control_bl70x_uart_set_clock_enable(uint32_t enable)
{
	uint32_t tmp = 0;

	if (enable > 1) {
		enable = 1;
	}
	tmp = sys_read32(GLB_BASE + GLB_CLK_CFG2_OFFSET);
	tmp = (tmp & GLB_UART_CLK_EN_UMSK) | (enable << GLB_UART_CLK_EN_POS);
	sys_write32(tmp, GLB_BASE + GLB_CLK_CFG2_OFFSET);
}

/* Clock:
 * FCLK: 0
 * 96 Mhz DLL: 1
 * When using DLL root clock, we can use either setting, when using the 32Mhz Oscillator with a
 * uninitialized DLL, only FCLK will be available.
 */
static void clock_control_bl70x_uart_set_clock(uint32_t enable, uint32_t clock, uint32_t divider)
{
	uint32_t tmp = 0;

	if (divider > 0x7) {
		divider = 0x7;
	}
	if (clock > 1) {
		clock = 1;
	}
	/* disable uart clock */
	clock_control_bl70x_uart_set_clock_enable(0);

	tmp = sys_read32(GLB_BASE + GLB_CLK_CFG2_OFFSET);
	tmp = (tmp & GLB_UART_CLK_DIV_UMSK) | (divider << GLB_UART_CLK_DIV_POS);
	sys_write32(tmp, GLB_BASE + GLB_CLK_CFG2_OFFSET);

	tmp = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmp = (tmp & HBN_UART_CLK_SEL_UMSK) | (clock << HBN_UART_CLK_SEL_POS);
	sys_write32(tmp, HBN_BASE + HBN_GLB_OFFSET);

	clock_control_bl70x_uart_set_clock_enable(enable);
}

/* Simple function to enable all peripherals for now */
static void clock_control_bl70x_peripheral_clock_init(void)
{
	uint32_t regval = sys_read32(GLB_BASE + GLB_CGEN_CFG1_OFFSET);

	/* enable ADC clock routing */
	regval |= (1 << 2);
	/* enable UART0 clock routing */
	regval |= (1 << 16);
	/* enable I2C0 clock routing */
	regval |= (1 << 19);

	sys_write32(regval, GLB_BASE + GLB_CGEN_CFG1_OFFSET);

	clock_control_bl70x_uart_set_clock(1, 0, 0);
}

static int clock_control_bl70x_on(const struct device *dev, clock_control_subsys_t sys)
{
	return -ENOTSUP;
}

static int clock_control_bl70x_off(const struct device *dev, clock_control_subsys_t sys)
{
	return -ENOTSUP;
}

static enum clock_control_status clock_control_bl70x_get_status(const struct device *dev,
								   clock_control_subsys_t sys)
{
	return CLOCK_CONTROL_STATUS_UNKNOWN;
}

static int clock_control_bl70x_get_rate(const struct device *dev, clock_control_subsys_t sys,
					   uint32_t *rate)
{
	const struct clock_control_bl70x_config *config = dev->config;

	if ((enum bl70x_clkid)sys == bl70x_clkid_clk_root) {
		*rate = clock_control_bl70x_get_clk(dev);
	} else if  ((enum bl70x_clkid)sys == bl70x_clkid_clk_bclk) {
		*rate = clock_control_bl70x_get_bclk(dev);
	} else if  ((enum bl70x_clkid)sys == bl70x_clkid_clk_crystal) {
		*rate = config->crystal_frequency;
	} else if  ((enum bl70x_clkid)sys == bl70x_clkid_clk_rc32m) {
		*rate = RC32M_FREQ;
	} else {
		return -EINVAL;
	}
	return 0;
}

static int clock_control_bl70x_init(const struct device *dev)
{
	int ret;
	uint32_t key;

	key = irq_lock();

	ret = clock_control_bl70x_init_root(dev);
	if (ret < 0) {
		irq_unlock(key);
		return ret;
	}

	clock_control_bl70x_peripheral_clock_init();

	irq_unlock(key);

	return 0;
}

static DEVICE_API(clock_control, clock_control_bl70x_api) = {
	.on = clock_control_bl70x_on,
	.off = clock_control_bl70x_off,
	.get_rate = clock_control_bl70x_get_rate,
	.get_status = clock_control_bl70x_get_status,
};

static const struct clock_control_bl70x_config clock_control_bl70x_config = {
	.crystal_frequency = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, crystal), clock_frequency),
};

static struct clock_control_bl70x_data clock_control_bl70x_data = {
	.crystal_enabled = DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, crystal)),
	.dll_enabled = DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, dll)),

	.root = {
#if CLK_SRC_IS(root, dll)
		.source = bl70x_clkid_clk_dll,
#elif CLK_SRC_IS(root, crystal)
		.source = bl70x_clkid_clk_crystal,
#else
		.source = bl70x_clkid_clk_rc32m,
#endif
		.dll_select = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, root), dll_select),
		.divider = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, root), divider),
	},

	.dll = {
#if CLK_SRC_IS(dll, crystal)
		.source = bl70x_clkid_clk_crystal,
#else
		.source = bl70x_clkid_clk_rc32m,
#endif
	},

	.bclk = {
		.divider = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, bclk), divider),
	},
};

BUILD_ASSERT( CLK_SRC_IS(dll, crystal) || CLK_SRC_IS(root, crystal) ? DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, crystal)) : 1,
	     "Crystal must be enabled to use it");

BUILD_ASSERT(CLK_SRC_IS(root, dll) ? DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, dll)) : 1,
	     "DLL must be enabled to use it");

BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, rc32m)),
	     "RC32M is always on");

DEVICE_DT_INST_DEFINE(0, clock_control_bl70x_init, NULL, &clock_control_bl70x_data,
		      &clock_control_bl70x_config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_bl70x_api);

