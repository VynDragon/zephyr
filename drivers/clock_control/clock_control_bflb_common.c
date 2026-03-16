/*
 * Copyright (c) 2026 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Functions applicable to all bflb clock drivers.
 * They are to be used only locally and the file split is to be exclusively used for deduplication.
 * Do not re-use or include elsewhere.
 */

#if !defined(_DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_BFLB_COMMON_C_)	\
    && defined(_DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_BFLB_COMMON_H_)
#define _DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_BFLB_COMMON_C_

/* 0: rc32k
 * 1: xtal32k
 * 3: dig32k
 */
static void clock_control_bflb_set_f32k_src(uint8_t src)
{
	uint32_t tmp;

	tmp = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmp &= HBN_F32K_SEL_UMSK;
	tmp |= src << HBN_F32K_SEL_POS;
	sys_write32(tmp, HBN_BASE + HBN_GLB_OFFSET);
}

static int clock_control_bflb_deinit_crystal(void)
{
	uint32_t tmp;

	/* Unpower crystal */
	tmp = sys_read32(AON_BASE + AON_RF_TOP_AON_OFFSET);
	tmp = tmp & AON_PU_XTAL_AON_UMSK;
	tmp = tmp & AON_PU_XTAL_BUF_AON_UMSK;
	sys_write32(tmp, AON_BASE + AON_RF_TOP_AON_OFFSET);

	clock_bflb_settle();
	return 0;
}

static int clock_control_bflb_init_crystal(void)
{
	uint32_t tmp;
	int count = CLOCK_TIMEOUT;

	/* Power crystal */
	tmp = sys_read32(AON_BASE + AON_RF_TOP_AON_OFFSET);
	tmp = (tmp & AON_PU_XTAL_AON_UMSK) | (1U << AON_PU_XTAL_AON_POS);
	tmp = (tmp & AON_PU_XTAL_BUF_AON_UMSK) | (1U << AON_PU_XTAL_BUF_AON_POS);
	sys_write32(tmp, AON_BASE + AON_RF_TOP_AON_OFFSET);

	/* Wait for crystal to be powered on */
	do {
		clock_bflb_settle();
		tmp = sys_read32(AON_BASE + AON_TSEN_OFFSET);
		count--;
	} while (!(tmp & AON_XTAL_RDY_MSK) && count > 0);

	clock_bflb_settle();
	if (count < 1) {
		return -1;
	}
	return 0;
}

/* source for most clocks, either XTAL or RC32M */
static uint32_t clock_control_bflb_get_xclk(void)
{
	uint32_t tmp;

	tmp = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmp &= HBN_ROOT_CLK_SEL_MSK;
	tmp = tmp >> HBN_ROOT_CLK_SEL_POS;
	tmp &= 1;
	if (tmp == 0) {
		return BFLB_RC32M_FREQUENCY;
	} else if (tmp == 1) {
		return DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, crystal), clock_frequency);
	} else {
		return 0;
	}
}

static int clock_control_bflb_clock_trim_32K(void)
{
	uint32_t tmp;
	int err;
	uint32_t trim, trim_parity;
	const struct device *efuse = DEVICE_DT_GET_ONE(bflb_efuse);

	err = syscon_read_reg(efuse, EFUSE_RC32K_TRIM_OFFSET, &trim);
	if (err < 0) {
		LOG_ERR("Error: Couldn't read efuses: err: %d.\n", err);
		return err;
	}
	if (!((trim >> EFUSE_RC32K_TRIM_EN_POS) & 1)) {
		LOG_ERR("RC32K trim disabled!");
		return -EINVAL;
	}

	trim_parity = (trim >> EFUSE_RC32K_TRIM_PARITY_POS) & 1;
	trim = (trim & EFUSE_RC32K_TRIM_MSK) >> EFUSE_RC32K_TRIM_POS;

	if (trim_parity != (POPCOUNT(trim) & 1)) {
		LOG_ERR("Bad trim parity");
		return -EINVAL;
	}

	tmp = sys_read32(HBN_BASE + HBN_RC32K_CTRL0_OFFSET);
	tmp |= HBN_RC32K_EXT_CODE_EN_MSK;
	tmp = (tmp & HBN_RC32K_CODE_FR_EXT_UMSK) | trim << HBN_RC32K_CODE_FR_EXT_POS;
	sys_write32(tmp, HBN_BASE + HBN_RC32K_CTRL0_OFFSET);

	clock_bflb_settle();

	return 0;
}

/* Functions applicable to BL60x and BL70x/L*/
#if !defined(CONFIG_SOC_SERIES_BL61X)

/* HCLK is the core clock */
static void clock_control_bflb_set_root_clock_dividers(uint32_t hclk_div, uint32_t bclk_div)
{
	uint32_t tmp;
	uint32_t old_rootclk;

	old_rootclk = clock_bflb_get_root_clock();

	/* security RC32M */
	if (old_rootclk > 1) {
		clock_bflb_set_root_clock(BFLB_MAIN_CLOCK_RC32M);
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

	clock_bflb_settle();

	/* enable clocks */
	tmp = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmp = (tmp & GLB_REG_BCLK_EN_UMSK) | (1U << GLB_REG_BCLK_EN_POS);
	tmp = (tmp & GLB_REG_HCLK_EN_UMSK) | (1U << GLB_REG_HCLK_EN_POS);
	sys_write32(tmp, GLB_BASE + GLB_CLK_CFG0_OFFSET);

	clock_bflb_set_root_clock(old_rootclk);
	clock_bflb_settle();
}

static void clock_control_bflb_set_machine_timer_clock_enable(bool enable)
{
	uint32_t tmp;

	tmp = sys_read32(GLB_BASE + GLB_CPU_CLK_CFG_OFFSET);
	if (enable) {
		tmp = (tmp & GLB_CPU_RTC_EN_UMSK) | (1U << GLB_CPU_RTC_EN_POS);
	} else {
		tmp = (tmp & GLB_CPU_RTC_EN_UMSK) | (0U << GLB_CPU_RTC_EN_POS);
	}
	sys_write32(tmp, GLB_BASE + GLB_CPU_CLK_CFG_OFFSET);
}

/* clock:
 * 0: BCLK
 * 1: 32Khz Oscillator (RC32*K*)
 */
static void clock_control_bflb_set_machine_timer_clock(bool enable, uint32_t clock,
							uint32_t divider)
{
	uint32_t tmp;

	if (divider > 0x1FFFF) {
		divider = 0x1FFFF;
	}
	if (clock > 1) {
		clock = 1;
	}

	/* disable first, then set div */
	clock_control_bflb_set_machine_timer_clock_enable(false);

	tmp = sys_read32(GLB_BASE + GLB_CPU_CLK_CFG_OFFSET);
	tmp = (tmp & GLB_CPU_RTC_SEL_UMSK) | (clock << GLB_CPU_RTC_SEL_POS);
	tmp = (tmp & GLB_CPU_RTC_DIV_UMSK) | (divider << GLB_CPU_RTC_DIV_POS);
	sys_write32(tmp, GLB_BASE + GLB_CPU_CLK_CFG_OFFSET);

	clock_control_bflb_set_machine_timer_clock_enable(enable);
}

static void clock_control_bflb_rc32k_enabled(bool yes)
{
	uint32_t tmp;

	tmp = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmp &= HBN_PU_RC32K_UMSK;
	if (yes) {
		tmp |= HBN_PU_RC32K_MSK;
	}
	sys_write32(tmp, HBN_BASE + HBN_GLB_OFFSET);
}

static bool clock_control_bflb_rc32k_is_enabled(void)
{
	return (sys_read32(HBN_BASE + HBN_GLB_OFFSET) & HBN_PU_RC32K_MSK) != 0;
}

static void clock_control_bflb_select_xLL(uint8_t xll)
{
	uint32_t tmp;

	tmp = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmp = (tmp & GLB_REG_PLL_SEL_UMSK) | (xll << GLB_REG_PLL_SEL_POS);
	sys_write32(tmp, GLB_BASE + GLB_CLK_CFG0_OFFSET);
}

static int clock_control_bflb_clock_trim_32M(void)
{
	uint32_t tmp;
	int err;
	uint32_t trim, trim_parity;
	const struct device *efuse = DEVICE_DT_GET_ONE(bflb_efuse);

	err = syscon_read_reg(efuse, EFUSE_RC32M_TRIM_OFFSET, &trim);
	if (err < 0) {
		LOG_ERR("Error: Couldn't read efuses: err: %d.\n", err);
		return err;
	}
	if (!((trim >> EFUSE_RC32M_TRIM_EN_POS) & 1)) {
		LOG_ERR("RC32M trim disabled!");
		return -EINVAL;
	}

	trim_parity = (trim >> EFUSE_RC32M_TRIM_PARITY_POS) & 1;
	trim = (trim & EFUSE_RC32M_TRIM_MSK) >> EFUSE_RC32M_TRIM_POS;

	if (trim_parity != (POPCOUNT(trim) & 1)) {
		LOG_ERR("Bad trim parity");
		return -EINVAL;
	}

	tmp = sys_read32(PDS_BASE + PDS_RC32M_CTRL0_OFFSET);
	tmp = (tmp & PDS_RC32M_EXT_CODE_EN_UMSK) | 1 << PDS_RC32M_EXT_CODE_EN_POS;
	tmp = (tmp & PDS_RC32M_CODE_FR_EXT_UMSK) | trim << PDS_RC32M_CODE_FR_EXT_POS;
	sys_write32(tmp, PDS_BASE + PDS_RC32M_CTRL0_OFFSET);

	clock_bflb_settle();

	return 0;
}

static void clock_control_bflb_cache_2T(bool yes)
{
	uint32_t tmp;

	tmp = sys_read32(L1C_BASE + L1C_CONFIG_OFFSET);

	if (yes) {
		tmp |= L1C_IROM_2T_ACCESS_MSK;
	} else {
		tmp &= ~L1C_IROM_2T_ACCESS_MSK;
	}

	sys_write32(tmp, L1C_BASE + L1C_CONFIG_OFFSET);
}

static __ramfunc void clock_control_bflb_update_flash_clk(
	const struct clock_control_bflb_flashclk_config * const cfg)
{
	volatile uint32_t tmp;

	tmp = *(volatile uint32_t *)(GLB_BASE + GLB_CLK_CFG2_OFFSET);
	tmp &= GLB_SF_CLK_DIV_UMSK;
	tmp &= GLB_SF_CLK_EN_UMSK;
	tmp |= (cfg->divider - 1) << GLB_SF_CLK_DIV_POS;
	*(volatile uint32_t *)(GLB_BASE + GLB_CLK_CFG2_OFFSET) = tmp;

	tmp = *(volatile uint32_t *)(SF_CTRL_BASE + SF_CTRL_0_OFFSET);
	tmp |= SF_CTRL_SF_IF_READ_DLY_EN_MSK;
	tmp &= ~SF_CTRL_SF_IF_READ_DLY_N_MSK;
	tmp |= (cfg->read_delay << SF_CTRL_SF_IF_READ_DLY_N_POS);
	if (cfg->clock_invert) {
		tmp &= ~SF_CTRL_SF_CLK_OUT_INV_SEL_MSK;
	} else {
		tmp |= SF_CTRL_SF_CLK_OUT_INV_SEL_MSK;
	}
	if (cfg->rx_clock_invert) {
		tmp |= SF_CTRL_SF_CLK_SF_RX_INV_SEL_MSK;
	} else {
		tmp &= ~SF_CTRL_SF_CLK_SF_RX_INV_SEL_MSK;
	}
	*(volatile uint32_t *)(SF_CTRL_BASE + SF_CTRL_0_OFFSET) = tmp;

	tmp = *(volatile uint32_t *)(GLB_BASE + GLB_CLK_CFG2_OFFSET);
	tmp &= GLB_SF_CLK_SEL_UMSK;
	tmp &= GLB_SF_CLK_SEL2_UMSK;

#if defined(CONFIG_SOC_SERIES_BL60X)
	if (cfg->source == bflb_clkid_clk_pll) {
#else
	if (cfg->source == bflb_clkid_clk_dll) {
#endif
		tmp |= 0U << GLB_SF_CLK_SEL_POS;
		tmp |= 0U << GLB_SF_CLK_SEL2_POS;
	} else if (cfg->source == bflb_clkid_clk_crystal) {
		tmp |= 0U << GLB_SF_CLK_SEL_POS;
		tmp |= 1U << GLB_SF_CLK_SEL2_POS;
	} else {
		/* If using RC32M or BCLK, use BCLK */
		tmp |= 2U << GLB_SF_CLK_SEL_POS;
	}

	*(volatile uint32_t *)(GLB_BASE + GLB_CLK_CFG2_OFFSET) = tmp;

	tmp = *(volatile uint32_t *)(GLB_BASE + GLB_CLK_CFG2_OFFSET);
	tmp |= GLB_SF_CLK_EN_MSK;
	*(volatile uint32_t *)(GLB_BASE + GLB_CLK_CFG2_OFFSET) = tmp;

	clock_bflb_settle();
}

static int clock_control_bflb_update_f32k(const struct clock_control_bflb_f32k_config * const f32k)
{
	bool wait_change = false;
	uint32_t tmp, tmpold;
	int ret;

	if (f32k->source != bflb_clkid_clk_xtal32k
		&& f32k->source != bflb_clkid_clk_rc32k) {
		return -EINVAL;
	}

	if (!clock_control_bflb_rc32k_is_enabled()) {
		clock_control_bflb_rc32k_enabled(true);
		wait_change = true;
	}

	if (f32k->xtal_enabled) {
		tmp = sys_read32(HBN_BASE + HBN_XTAL32K_OFFSET);
		tmpold = tmp;
		tmp |= HBN_PU_XTAL32K_MSK;
		tmp |= HBN_PU_XTAL32K_BUF_MSK;
		if (tmpold != tmp) {
			sys_write32(tmp, HBN_BASE + HBN_XTAL32K_OFFSET);
			wait_change = true;
		}
	} else {
		tmp = sys_read32(HBN_BASE + HBN_XTAL32K_OFFSET);
		tmp &= HBN_PU_XTAL32K_UMSK;
		tmp &= HBN_PU_XTAL32K_BUF_UMSK;
		sys_write32(tmp, HBN_BASE + HBN_XTAL32K_OFFSET);
	}

	if (wait_change) {
		clock_control_bflb_clock_at_least_us(1000);
	}

	if (f32k->source == bflb_clkid_clk_rc32k) {
		ret = clock_control_bflb_clock_trim_32K();
		if (ret < 0) {
			return ret;
		}
		clock_control_bflb_set_f32k_src(0);
	} else {
		clock_control_bflb_set_f32k_src(1);
	}

	return 0;
}

static void clock_control_bflb_uart_set_clock_enable(bool enable)
{
	uint32_t tmp;

	tmp = sys_read32(GLB_BASE + GLB_CLK_CFG2_OFFSET);
	if (enable) {
		tmp = (tmp & GLB_UART_CLK_EN_UMSK) | (1U << GLB_UART_CLK_EN_POS);
	} else {
		tmp = (tmp & GLB_UART_CLK_EN_UMSK) | (0U << GLB_UART_CLK_EN_POS);
	}
	sys_write32(tmp, GLB_BASE + GLB_CLK_CFG2_OFFSET);
}

/* Clock:
 * FCLK: 0
 * PLL/DLL: 1
 * When using PLL root clock, we can use either setting, when using the 32Mhz Oscillator with a
 * uninitialized PLL, only FCLK will be available.
 */
static void clock_control_bflb_uart_set_clock(bool enable, uint32_t clock, uint32_t divider)
{
	uint32_t tmp;

	if (divider > 0x7) {
		divider = 0x7;
	}
	if (clock > 1) {
		clock = 1;
	}
	/* disable uart clock */
	clock_control_bflb_uart_set_clock_enable(false);

	tmp = sys_read32(GLB_BASE + GLB_CLK_CFG2_OFFSET);
	tmp = (tmp & GLB_UART_CLK_DIV_UMSK) | (divider << GLB_UART_CLK_DIV_POS);
	sys_write32(tmp, GLB_BASE + GLB_CLK_CFG2_OFFSET);

	tmp = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmp = (tmp & HBN_UART_CLK_SEL_UMSK) | (clock << HBN_UART_CLK_SEL_POS);
	sys_write32(tmp, HBN_BASE + HBN_GLB_OFFSET);

	clock_control_bflb_uart_set_clock_enable(enable);
}

#endif

/* Common API handling bits */

static int clock_control_bflb_on_common(const struct device *dev, clock_control_subsys_t sys)
{
	struct clock_control_bflb_data *data = dev->data;
	int ret = -EINVAL;
	enum bflb_clkid oldroot;

	if ((enum bflb_clkid)sys == bflb_clkid_clk_crystal) {
		if (data->crystal_enabled) {
			ret = 0;
		} else {
			data->crystal_enabled = true;
			ret = clock_control_bflb_update_clocks(dev);
			if (ret < 0) {
				data->crystal_enabled = false;
			}
		}
	} else if ((enum bflb_clkid)sys == bflb_clkid_clk_xtal32k) {
		if (data->f32k.xtal_enabled) {
			ret = 0;
		} else {
			data->f32k.xtal_enabled = true;
			ret = clock_control_bflb_update_clocks(dev);
			if (ret < 0) {
				data->f32k.xtal_enabled = false;
			}
		}
	} else if ((enum bflb_clkid)sys == BFLB_CLKID_CLK_XLL) {
		if (data->CLOCK_CTRL_BFLB_XLL_0.enabled) {
			ret = 0;
		} else {
			data->CLOCK_CTRL_BFLB_XLL_0.enabled = true;
			ret = clock_control_bflb_update_clocks(dev);
			if (ret < 0) {
				data->CLOCK_CTRL_BFLB_XLL_0.enabled = false;
			}
		}
	} else if ((int)sys == BFLB_FORCE_ROOT_RC32M) {
		if (data->root.source == bflb_clkid_clk_rc32m) {
			ret = 0;
		} else {
			/* Cannot fail to set root to rc32m */
			data->root.source = bflb_clkid_clk_rc32m;
			ret = clock_control_bflb_update_clocks(dev);
		}
	} else if ((int)sys == BFLB_FORCE_ROOT_CRYSTAL) {
		if (data->root.source == bflb_clkid_clk_crystal) {
			ret = 0;
		} else {
			oldroot = data->root.source;
			data->root.source = bflb_clkid_clk_crystal;
			ret = clock_control_bflb_update_clocks(dev);
			if (ret < 0) {
				data->root.source = oldroot;
			}
		}
	} else if ((int)sys == BFLB_FORCE_ROOT_XLL) {
		if (data->root.source == BFLB_CLKID_CLK_XLL) {
			ret = 0;
		} else {
			oldroot = data->root.source;
			data->root.source = BFLB_CLKID_CLK_XLL;
			ret = clock_control_bflb_update_clocks(dev);
			if (ret < 0) {
				data->root.source = oldroot;
			}
		}
	}

	return ret;
}

static int clock_control_bflb_off_common(const struct device *dev, clock_control_subsys_t sys)
{
	struct clock_control_bflb_data *data = dev->data;
	int ret = -EINVAL;

	if ((enum bflb_clkid)sys == bflb_clkid_clk_crystal) {
		if (!data->crystal_enabled) {
			ret = 0;
		} else {
			data->crystal_enabled = false;
			ret = clock_control_bflb_update_clocks(dev);
			if (ret < 0) {
				data->crystal_enabled = true;
			}
		}
	} else if ((enum bflb_clkid)sys == bflb_clkid_clk_xtal32k) {
		if (!data->f32k.xtal_enabled) {
			ret = 0;
		} else {
			data->f32k.xtal_enabled = false;
			ret = clock_control_bflb_update_clocks(dev);
			if (ret < 0) {
				data->f32k.xtal_enabled = true;
			}
		}
	} else if ((enum bflb_clkid)sys == BFLB_CLKID_CLK_XLL) {
		if (!data->CLOCK_CTRL_BFLB_XLL_0.enabled) {
			ret = 0;
		} else {
			data->CLOCK_CTRL_BFLB_XLL_0.enabled = false;
			ret = clock_control_bflb_update_clocks(dev);
			if (ret < 0) {
				data->CLOCK_CTRL_BFLB_XLL_0.enabled = true;
			}
		}
	}

	return ret;
}

static enum clock_control_status clock_control_bflb_get_status_common(const struct device *dev,
								      clock_control_subsys_t sys)
{
	struct clock_control_bflb_data *data = dev->data;

	switch ((enum bflb_clkid)sys) {
	case bflb_clkid_clk_root:
	case bflb_clkid_clk_bclk:
	case bflb_clkid_clk_rc32m:
	case bflb_clkid_clk_rc32k:
		return CLOCK_CONTROL_STATUS_ON;
	case bflb_clkid_clk_crystal:
		if (data->crystal_enabled) {
			return CLOCK_CONTROL_STATUS_ON;
		}
		return CLOCK_CONTROL_STATUS_OFF;
	case bflb_clkid_clk_xtal32k:
		if (data->f32k.xtal_enabled) {
			return CLOCK_CONTROL_STATUS_ON;
		}
		return CLOCK_CONTROL_STATUS_OFF;
	case BFLB_CLKID_CLK_XLL:
		if (data->CLOCK_CTRL_BFLB_XLL_0.enabled) {
			return CLOCK_CONTROL_STATUS_ON;
		}
		return CLOCK_CONTROL_STATUS_OFF;
	default:
		return -EINVAL;
	}
}

static int clock_control_bflb_get_rate_common(const struct device *dev, clock_control_subsys_t sys,
					      uint32_t *rate)
{
	switch ((enum bflb_clkid)sys) {
	case bflb_clkid_clk_root:
		*rate = clock_control_bflb_get_hclk(dev);
		break;
	case bflb_clkid_clk_bclk:
		*rate = clock_control_bflb_get_bclk(dev);
		break;
	case bflb_clkid_clk_rc32m:
		*rate = BFLB_RC32M_FREQUENCY;
		break;
	case bflb_clkid_clk_f32k:
		*rate = BFLB_F32K_FREQUENCY;
		break;
	case bflb_clkid_clk_rc32k:
		*rate = BFLB_F32K_FREQUENCY;
		break;
	case bflb_clkid_clk_crystal:
		*rate = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, crystal), clock_frequency);
		break;
	case bflb_clkid_clk_xtal32k:
		*rate = DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, xtal32k), clock_frequency);
		break;
	case BFLB_CLKID_CLK_XLL:
		*rate = clock_control_bflb_get_xllclk(dev);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* Common checks */

BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, rc32m)), "RC32M is always on");
BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, rc32k)), "RC32K is always on");

BUILD_ASSERT(CLK_SRC_IS(f32k, xtal32k)
	? DT_NODE_HAS_STATUS_OKAY(DT_INST_CLOCKS_CTLR_BY_NAME(0, xtal32k)) : 1,
	"XTAL32K must be enabled to use it");

BUILD_ASSERT(DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(0, rc32m), clock_frequency)
	== BFLB_RC32M_FREQUENCY, "RC32M must be 32M");
#else
#error "Do not misuse this file!"
#endif /* _DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_BFLB_COMMON_C_ */
