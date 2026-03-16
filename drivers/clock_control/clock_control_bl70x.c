/*
 * Copyright (c) 2025-2026 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_bl70x_clock_controller

#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/clock/bflb_bl70x_clock.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_bl70x, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

#include <bouffalolab/bl70x/bflb_soc.h>
#include <bouffalolab/bl70x/aon_reg.h>
#include <bouffalolab/bl70x/glb_reg.h>
#include <bouffalolab/bl70x/hbn_reg.h>
#include <bouffalolab/bl70x/pds_reg.h>
#include <bouffalolab/bl70x/l1c_reg.h>
#include <bouffalolab/bl70x/extra_defines.h>
#include <bouffalolab/bl70x/sf_ctrl_reg.h>
#include <zephyr/drivers/clock_control/clock_control_bflb_common.h>

enum bflb_clkid {
	bflb_clkid_clk_root = BL70X_CLKID_CLK_ROOT,
	bflb_clkid_clk_rc32m = BL70X_CLKID_CLK_RC32M,
	bflb_clkid_clk_crystal = BL70X_CLKID_CLK_CRYSTAL,
	bflb_clkid_clk_dll = BL70X_CLKID_CLK_DLL,
	bflb_clkid_clk_bclk = BL70X_CLKID_CLK_BCLK,
	bflb_clkid_clk_f32k = BL70X_CLKID_CLK_F32K,
	bflb_clkid_clk_xtal32k = BL70X_CLKID_CLK_XTAL32K,
	bflb_clkid_clk_rc32k = BL70X_CLKID_CLK_RC32K,
};

struct clock_control_bflb_flashclk_config {
	enum bflb_clkid	source;
	uint8_t		divider;
	uint8_t		read_delay;
	bool		clock_invert;
	bool		rx_clock_invert;
};

#include "clock_control_bflb_common.h"

#define CLOCK_TIMEOUT			1024

#define EFUSE_RC32M_TRIM_OFFSET		0x0C
#define EFUSE_RC32M_TRIM_EN_POS		19
#define EFUSE_RC32M_TRIM_PARITY_POS	18
#define EFUSE_RC32M_TRIM_POS		10
#define EFUSE_RC32M_TRIM_MSK		0x3FC00
#define EFUSE_RC32K_TRIM_OFFSET		0x0C
#define EFUSE_RC32K_TRIM_EN_POS		31
#define EFUSE_RC32K_TRIM_PARITY_POS	30
#define EFUSE_RC32K_TRIM_POS		20
#define EFUSE_RC32K_TRIM_MSK		0x3FF00000

#define CLOCK_CTRL_BFLB_XLL_0		dll

struct clock_control_bflb_data {
	bool 						crystal_enabled;
	struct clock_control_bflb_root_config		root;
	struct clock_control_bflb_bclk_config		bclk;
	struct clock_control_bflb_f32k_config		f32k;
	struct clock_control_bflb_xll_config		dll;
	struct clock_control_bflb_flashclk_config	flashclk;
};

static void clock_control_bflb_clock_at_least_us(uint32_t us)
{
	for (uint32_t i = 0; i < us * 8; i++) {
		clock_bflb_settle();
	}
}

static void clock_control_bflb_deinit_dll(void)
{
	uint32_t tmp;

	/* DLL Off */
	tmp = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmp = (tmp & GLB_PPU_DLL_UMSK) | (0U << GLB_PPU_DLL_POS);
	tmp = (tmp & GLB_PU_DLL_UMSK) | (0U << GLB_PU_DLL_POS);
	tmp = (tmp & GLB_DLL_RESET_UMSK) | (1U << GLB_DLL_RESET_POS);
	sys_write32(tmp, GLB_BASE + GLB_DLL_OFFSET);
}

/* RC32M : 0
 * XTAL : 1
 */
static void clock_control_bflb_set_dll_source(uint32_t source)
{
	uint32_t tmp;

	tmp = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	if (source > 0) {
		tmp = (tmp & GLB_DLL_REFCLK_SEL_UMSK) | (1U << GLB_DLL_REFCLK_SEL_POS);
	} else {
		tmp = (tmp & GLB_DLL_REFCLK_SEL_UMSK) | (0U << GLB_DLL_REFCLK_SEL_POS);
	}
	sys_write32(tmp, GLB_BASE + GLB_DLL_OFFSET);
}

static void clock_control_bflb_init_dll(enum bflb_clkid source)
{
	uint32_t tmp;
	uint32_t old_rootclk;

	old_rootclk = clock_bflb_get_root_clock();

	/* security RC32M */
	if (old_rootclk > 1) {
		clock_bflb_set_root_clock(BFLB_MAIN_CLOCK_RC32M);
	}

	clock_control_bflb_deinit_dll();

	if (source == BL70X_CLKID_CLK_CRYSTAL) {
		clock_control_bflb_set_dll_source(1);
	} else {
		clock_control_bflb_set_dll_source(0);
	}

	/* init sequence */
	tmp = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmp = (tmp & GLB_DLL_PRECHG_SEL_UMSK) | (1U << GLB_DLL_PRECHG_SEL_POS);
	sys_write32(tmp, GLB_BASE + GLB_DLL_OFFSET);

	tmp = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmp = (tmp & GLB_PPU_DLL_UMSK) | (1U << GLB_PPU_DLL_POS);
	sys_write32(tmp, GLB_BASE + GLB_DLL_OFFSET);

	tmp = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmp = (tmp & GLB_PU_DLL_UMSK) | (1U << GLB_PU_DLL_POS);
	sys_write32(tmp, GLB_BASE + GLB_DLL_OFFSET);

	tmp = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmp = (tmp & GLB_DLL_RESET_UMSK) | (0U << GLB_DLL_RESET_POS);
	sys_write32(tmp, GLB_BASE + GLB_DLL_OFFSET);

	clock_bflb_settle();

	clock_bflb_set_root_clock(old_rootclk);
	clock_bflb_settle();
}

static uint32_t clock_control_bflb_get_xllclk(const struct device *dev)
{
	return MHZ(144);
}

static uint32_t clock_control_bflb_get_hclk(const struct device *dev)
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
		return clock_control_bflb_get_xclk() / (hclk_div + 1);
	}
	tmp = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmp = (tmp & GLB_REG_PLL_SEL_MSK) >> GLB_REG_PLL_SEL_POS;
	if (tmp == 3) {
		return MHZ(120) / (hclk_div + 1);
	} else if (tmp == 2) {
		return MHZ(144) / (hclk_div + 1);
	} else if (tmp == 1) {
		return MHZ(96) / (hclk_div + 1);
	} else if (tmp == 0) {
		return MHZ(57.6) / (hclk_div + 1);
	}
	return 0;
}

/* most peripherals clock */
static uint32_t clock_control_bflb_get_bclk(const struct device *dev)
{
	uint32_t tmp;
	uint32_t clock_id;

	tmp = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmp = (tmp & GLB_REG_BCLK_DIV_MSK) >> GLB_REG_BCLK_DIV_POS;
	clock_id = clock_control_bflb_get_hclk(dev);
	return clock_id / (tmp + 1);
}

static uint32_t clock_control_bflb_mtimer_get_clk_src_div(const struct device *dev)
{
	return clock_control_bflb_get_bclk(dev) / 1000 / 1000 - 1;
}

static void clock_control_bflb_setup_dll(const struct device *dev)
{
	struct clock_control_bflb_data *data = dev->data;
	uint32_t tmp;

	clock_control_bflb_init_dll(data->dll.source);

	/* enable all 'PDS' clocks / Divider out */
	tmp = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmp = (tmp & GLB_DLL_CLK_57P6M_EN_UMSK) | (1U << GLB_DLL_CLK_57P6M_EN_POS);
	tmp = (tmp & GLB_DLL_CLK_96M_EN_UMSK) | (1U << GLB_DLL_CLK_96M_EN_POS);
	tmp = (tmp & GLB_DLL_CLK_144M_EN_UMSK) | (1U << GLB_DLL_CLK_144M_EN_POS);
	tmp = (tmp & GLB_DLL_CLK_288M_EN_UMSK) | (1U << GLB_DLL_CLK_288M_EN_POS);
	tmp = (tmp & GLB_DLL_CLK_MMDIV_EN_UMSK) | (1U << GLB_DLL_CLK_MMDIV_EN_POS);
	sys_write32(tmp, GLB_BASE + GLB_DLL_OFFSET);

	/* glb enable dll actual? */
	tmp = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmp = (tmp & GLB_REG_PLL_EN_UMSK) | (1U << GLB_REG_PLL_EN_POS);
	sys_write32(tmp, GLB_BASE + GLB_CLK_CFG0_OFFSET);
}

static void clock_control_bflb_init_root_as_dll(const struct device *dev)
{
	struct clock_control_bflb_data *data = dev->data;

	clock_control_bflb_select_xLL(data->root.xll_select);

	if (data->dll.source == bflb_clkid_clk_crystal) {
		clock_bflb_set_root_clock(BFLB_MAIN_CLOCK_PLL_XTAL);
	} else {
		clock_bflb_set_root_clock(BFLB_MAIN_CLOCK_PLL_RC32M);
	}

	if (clock_control_bflb_get_hclk(dev) > MHZ(120)) {
		clock_control_bflb_cache_2T(true);
	}

	sys_write32(clock_control_bflb_get_hclk(dev), CORECLOCKREGISTER);
}

static void clock_control_bflb_init_root_as_crystal(const struct device *dev)
{
	clock_bflb_set_root_clock(BFLB_MAIN_CLOCK_XTAL);
	sys_write32(clock_control_bflb_get_hclk(dev), CORECLOCKREGISTER);
}

static int clock_control_bflb_update_clocks(const struct device *dev)
{
	struct clock_control_bflb_data *data = dev->data;
	uint32_t tmp;
	int ret;

	/* make sure all clocks are enabled */
	tmp = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmp = (tmp & GLB_REG_BCLK_EN_UMSK) | (1U << GLB_REG_BCLK_EN_POS);
	tmp = (tmp & GLB_REG_HCLK_EN_UMSK) | (1U << GLB_REG_HCLK_EN_POS);
	tmp = (tmp & GLB_REG_FCLK_EN_UMSK) | (1U << GLB_REG_FCLK_EN_POS);
	sys_write32(tmp, GLB_BASE + GLB_CLK_CFG0_OFFSET);

	/* set root clock to internal 32MHz Oscillator as failsafe */
	clock_bflb_set_root_clock(BFLB_MAIN_CLOCK_RC32M);
	clock_control_bflb_set_root_clock_dividers(0, 0);
	sys_write32(BFLB_RC32M_FREQUENCY, CORECLOCKREGISTER);

	clock_control_bflb_cache_2T(false);

	ret = clock_control_bflb_update_f32k(&data->f32k);
	if (ret < 0) {
		return ret;
	}

	if (data->crystal_enabled) {
		if (clock_control_bflb_init_crystal() < 0) {
			return -EIO;
		}
	} else {
		clock_control_bflb_deinit_crystal();
	}

	clock_control_bflb_set_root_clock_dividers(data->root.divider - 1, data->bclk.divider - 1);

	if (data->dll.enabled) {
		clock_control_bflb_setup_dll(dev);
	} else {
		clock_control_bflb_deinit_dll();
	}

	if (data->root.source == bflb_clkid_clk_dll) {
		if (!data->dll.enabled) {
			return -EINVAL;
		}
		clock_control_bflb_init_root_as_dll(dev);
	} else if (data->root.source == bflb_clkid_clk_crystal) {
		if (!data->crystal_enabled) {
			return -EINVAL;
		}
		clock_control_bflb_init_root_as_crystal(dev);
	} else {
		/* Root clock already setup as RC32M */
	}

	ret = clock_control_bflb_clock_trim_32M();
	if (ret < 0) {
		return ret;
	}
	clock_control_bflb_set_machine_timer_clock(
		1, 0, clock_control_bflb_mtimer_get_clk_src_div(dev));

	clock_bflb_settle();

	return ret;
}

/* Simple function to enable all peripherals for now */
static void clock_control_bflb_peripheral_clock_init(void)
{
	uint32_t regval = sys_read32(GLB_BASE + GLB_CGEN_CFG1_OFFSET);

	/* enable ADC clock routing */
	regval |= (1 << 2);
	/* enable SEC clock routing */
	regval |= (1 << 3);
	/* enable UART0 clock routing */
	regval |= (1 << 16);
	/* enable SPI0 clock routing */
	regval |= (1 << 18);
	/* enable I2C0 clock routing */
	regval |= (1 << 19);
	/* enable PWM clock routing */
	regval |= (1 << 20);
	/* enable DMA clock routing */
	regval |= (1 << 12);
	/* enable IR clock routing */
	regval |= (1 << 22);

	sys_write32(regval, GLB_BASE + GLB_CGEN_CFG1_OFFSET);

	clock_control_bflb_uart_set_clock(true, 0, 0);
}

static int clock_control_bflb_on(const struct device *dev, clock_control_subsys_t sys)
{
	int ret;
	uint32_t key;

	key = irq_lock();

	ret = clock_control_bflb_on_common(dev, sys);

	irq_unlock(key);

	return ret;
}

static int clock_control_bflb_off(const struct device *dev, clock_control_subsys_t sys)
{
	int ret;
	uint32_t key;

	key = irq_lock();

	ret = clock_control_bflb_off_common(dev, sys);

	irq_unlock(key);

	return ret;
}

static enum clock_control_status clock_control_bflb_get_status(const struct device *dev,
							       clock_control_subsys_t sys)
{
	return clock_control_bflb_get_status_common(dev, sys);
}

static int clock_control_bflb_get_rate(const struct device *dev, clock_control_subsys_t sys,
				       uint32_t *rate)
{
	return clock_control_bflb_get_rate_common(dev, sys, rate);
}

static int clock_control_bflb_init(const struct device *dev)
{
	struct clock_control_bflb_data *data = dev->data;
	int ret;
	uint32_t key;

	key = irq_lock();

	ret = clock_control_bflb_update_clocks(dev);
	if (ret < 0) {
		irq_unlock(key);
		return ret;
	}

	clock_control_bflb_peripheral_clock_init();

	clock_bflb_settle();

	clock_control_bflb_update_flash_clk(&data->flashclk);

	irq_unlock(key);

	return 0;
}

static DEVICE_API(clock_control, clock_control_bflb_api) = {
	.on = clock_control_bflb_on,
	.off = clock_control_bflb_off,
	.get_rate = clock_control_bflb_get_rate,
	.get_status = clock_control_bflb_get_status,
};

static struct clock_control_bflb_data clock_control_bflb_data = {
	.crystal_enabled = DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, crystal)),

	.root = {
#if CLK_SRC_IS(root, dll_144)
			.source = bflb_clkid_clk_dll,
			.xll_select = DT_CLOCKS_CELL(DT_INST_CLOCKS_CTLR_BY_NAME(0, root), select),
#elif CLK_SRC_IS(root, crystal)
			.source = bflb_clkid_clk_crystal,
#else
			.source = bflb_clkid_clk_rc32m,
#endif
			.divider = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, root), divider),
		},

	.dll = {
#if CLK_SRC_IS(dll_144, crystal)
			.source = bflb_clkid_clk_crystal,
#else
			.source = bflb_clkid_clk_rc32m,
#endif
			.enabled = DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, dll_144)),
		},

	.bclk = {
			.divider = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, bclk), divider),
		},

	.flashclk = {
#if CLK_SRC_IS(flash, crystal)
		.source = bflb_clkid_clk_crystal,
#elif CLK_SRC_IS(flash, bclk)
		.source = bflb_clkid_clk_bclk,
#elif CLK_SRC_IS(flash, dll_144)
		.source = bflb_clkid_clk_dll,
#else
		.source = bflb_clkid_clk_rc32m,
#endif
		.read_delay = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, flash), read_delay),
		.clock_invert = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, flash), clock_invert),
		.rx_clock_invert = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, flash), rx_clock_invert),
		.divider = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, flash), divider),
	},

	.f32k = {
#if CLK_SRC_IS(f32k, xtal32k)
		.source = bflb_clkid_clk_xtal32k,
#else
		.source = bflb_clkid_clk_rc32k,
#endif
		.xtal_enabled = DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, xtal32k)),
	},
};

BUILD_ASSERT((CLK_SRC_IS(dll_144, crystal) || CLK_SRC_IS(root, crystal))
		     ? DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, crystal))
		     : 1,
	     "Crystal must be enabled to use it");

BUILD_ASSERT(CLK_SRC_IS(root, dll_144) ?
	DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, dll_144)) : 1,
	"PLL must be enabled to use it");

BUILD_ASSERT(DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, crystal), clock_frequency)
	== BFLB_RC32M_FREQUENCY, "Crystal must be 32M for BL70x");

DEVICE_DT_INST_DEFINE(0, clock_control_bflb_init, NULL, &clock_control_bflb_data,
		      NULL, PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &clock_control_bflb_api);

/* We precisely want this file in the compilation unit, just as if it was part of this file */
#include "clock_control_bflb_common.c"
