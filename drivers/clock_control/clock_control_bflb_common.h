/*
 * Copyright (c) 2026 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_BFLB_COMMON_H_
#define _DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_BFLB_COMMON_H_

/* Generic local types*/

struct clock_control_bflb_xll_config {
	enum bflb_clkid				source;
	uint32_t				top_frequency;
	bool					enabled;
};

struct clock_control_bflb_root_config {
	enum bflb_clkid				source;
	uint8_t					xll_select;
	uint8_t					divider;
};

struct clock_control_bflb_bclk_config {
	uint8_t					divider;
};

struct clock_control_bflb_f32k_config {
	enum bflb_clkid				source;
	bool					xtal_enabled;
};

/* Generic local macros */

#define CRYSTAL_ID_FREQ_32000000	0
#define CRYSTAL_ID_FREQ_24000000	1
#define CRYSTAL_ID_FREQ_38400000	2
#define CRYSTAL_ID_FREQ_40000000	3
#define CRYSTAL_ID_FREQ_26000000	4
#define CRYSTAL_VALUES_CNT		5

#define CRYSTAL_FREQ_TO_ID(freq) CONCAT(CRYSTAL_ID_FREQ_, freq)
#define CLK_SRC_IS(clk, src)                                                                       \
	DT_SAME_NODE(DT_CLOCKS_CTLR_BY_IDX(DT_INST_CLOCKS_CTLR_BY_NAME(0, clk), 0),                \
		     DT_INST_CLOCKS_CTLR_BY_NAME(0, src))

/* Forward declarations of generic local functions */

static void clock_control_bflb_set_f32k_src(uint8_t src);
static int clock_control_bflb_deinit_crystal(void);
static int clock_control_bflb_init_crystal(void);
static uint32_t clock_control_bflb_get_xclk(void);
static int clock_control_bflb_clock_trim_32K(void);

/* Functions applicable to BL60x and BL70x/L*/
#if defined(CONFIG_SOC_SERIES_BL60X) || defined(CONFIG_SOC_SERIES_BL70X)
static void clock_control_bflb_set_root_clock_dividers(uint32_t hclk_div, uint32_t bclk_div);
static void clock_control_bflb_set_machine_timer_clock_enable(bool enable);
static void clock_control_bflb_set_machine_timer_clock(bool enable, uint32_t clock,
							uint32_t divider);
static void clock_control_bflb_rc32k_enabled(bool yes);
static bool clock_control_bflb_rc32k_is_enabled(void);
static void clock_control_bflb_select_xLL(uint8_t xll);
static int clock_control_bflb_clock_trim_32M(void);
static void clock_control_bflb_cache_2T(bool yes);
static __ramfunc void clock_control_bflb_update_flash_clk(
	const struct clock_control_bflb_flashclk_config * const cfg);
static int clock_control_bflb_update_f32k(const struct clock_control_bflb_f32k_config * const f32k);
static void clock_control_bflb_uart_set_clock_enable(bool enable);
static void clock_control_bflb_uart_set_clock(bool enable, uint32_t clock, uint32_t divider);
#endif

/* Forward declarations of generic API-related local functions */

static int clock_control_bflb_on_common(const struct device *dev, clock_control_subsys_t sys);
static int clock_control_bflb_off_common(const struct device *dev, clock_control_subsys_t sys);
static enum clock_control_status clock_control_bflb_get_status_common(const struct device *dev,
								      clock_control_subsys_t sys);
static int clock_control_bflb_get_rate_common(const struct device *dev, clock_control_subsys_t sys,
					      uint32_t *rate);

#endif /* _DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_BFLB_COMMON_H_ */
