/*
 * Copyright (c) 2024, MASSDRIVER EI (massdriver.space)
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_bl_uart

/**
 * @brief UART driver for Bouffalo Lab MCU family.
 * Based on bouffalo_SDK 2.0 UART driver
 */

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/arch/common/sys_io.h>
#include <zephyr/drivers/timer/system_timer.h>


#include <soc.h>
#include <common_hardware/uart_reg.h>
#include <bflb_uart.h>



struct bflb_config {
	const struct pinctrl_dev_config *pincfg;
	uint32_t baudrate;
	uint8_t direction;
	uint8_t data_bits;
	uint8_t stop_bits;
	uint8_t parity;
	uint8_t bit_order;
	uint8_t flow_ctrl;
	uint8_t tx_fifo_threshold;
	uint8_t rx_fifo_threshold;
	uint32_t base_reg;
};


static void ITCMF uart_bflb_enabled(const struct device *dev, uint32_t enable)
{
	const struct bflb_config *cfg = dev->config;
	uint32_t rxt = 0;
	uint32_t txt = 0;

	if (enable > 1)
	{
		enable = 1;
	}

	txt = sys_read32(cfg->base_reg + UART_UTX_CONFIG_OFFSET);
	txt = (txt & ~UART_CR_UTX_EN) | enable;
	rxt = sys_read32(cfg->base_reg + UART_URX_CONFIG_OFFSET);
	rxt = (rxt & ~UART_CR_URX_EN) | enable;
	sys_write32(rxt, cfg->base_reg + UART_URX_CONFIG_OFFSET);
	sys_write32(txt, cfg->base_reg + UART_UTX_CONFIG_OFFSET);
}

static uint32_t ITCMF uart_bflb_get_crystal_frequency(void)
{
	uint32_t tmpVal;

	/* get clkpll_sdmin */
	tmpVal = sys_read32(PDS_BASE + PDS_CLKPLL_SDM_OFFSET);
	tmpVal = (tmpVal & PDS_CLKPLL_SDMIN_MSK) >> PDS_CLKPLL_SDMIN_POS;

	switch (tmpVal) {
		case 0x500000:
		/* 24m */
		return (24 * 1000 * 1000);

		case 0x3C0000:
		/* 32m */
		return (32 * 1000 * 1000);

		case 0x320000:
		/* 38.4m */
		return (384 * 100 * 1000);

		case 0x300000:
		/* 40m */
		return (40 * 1000 * 1000);

		case 0x49D39D:
		/* 26m */
		return (26 * 1000 * 1000);

		default:
		/* 32m */
		return (32 * 1000 * 1000);
	}
}

static uint32_t ITCMF uart_bflb_get_PLL_frequency(void)
{
	uint32_t tmpVal;

	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_REG_PLL_SEL_MSK) >> GLB_REG_PLL_SEL_POS;

	if (tmpVal == 0) {
		/* pll 48m */
		return (48 * 1000 * 1000);
	} else if (tmpVal == 1) {
		/* pll 120m */
		return (120 * 1000 * 1000);
	} else if (tmpVal == 2) {
		/* pll 160m */
		return (160 * 1000 * 1000);
	} else if (tmpVal == 3) {
		/* pll 192m */
		return (192 * 1000 * 1000);
	} else {
		return 0;
	}
}

static uint32_t ITCMF uart_bflb_get_clock(void)
{
	uint32_t tmpVal = 0;
	uint32_t uart_divider = 0;
	uint32_t hclk_divider = 0;


	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG2_OFFSET);
	uart_divider = (tmpVal & GLB_UART_CLK_DIV_MSK) >> GLB_UART_CLK_DIV_POS;


	tmpVal = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmpVal = (tmpVal & HBN_UART_CLK_SEL_MSK) >> HBN_UART_CLK_SEL_POS;


	if (tmpVal == 0)
	{
		/* FCLK */
		tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
		hclk_divider = (tmpVal & GLB_REG_HCLK_DIV_MSK) >> GLB_REG_HCLK_DIV_POS;

		tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
		tmpVal = (tmpVal & GLB_HBN_ROOT_CLK_SEL_MSK) >> GLB_HBN_ROOT_CLK_SEL_POS;

		if (tmpVal == 0)
		{
			/* RC32M clock */
			tmpVal = (32 * 1000 * 1000) / (hclk_divider + 1);
			return (tmpVal / (uart_divider + 1));
		}
		else if (tmpVal == 1)
		{
			/* Crystal clock */
			tmpVal = uart_bflb_get_crystal_frequency() / (hclk_divider + 1);
			return (tmpVal / (uart_divider + 1));
		}
		else if (tmpVal > 1)
		{
			/* PLL Clock */
			tmpVal = uart_bflb_get_PLL_frequency() / (hclk_divider + 1);
			return (tmpVal / (uart_divider + 1));

		}
	}
	else
	{
		/* UART PLL 160 */
		return ((160 * 1000 * 1000) / (uart_divider + 1));
	}
	return 0;
}

static int ITCMF uart_bflb_configure(const struct device *dev)
{
	const struct bflb_config *cfg = dev->config;
	uint32_t tx_cfg = 0;
	uint32_t rx_cfg = 0;
	uint32_t divider = 0;
	uint32_t tmpVal = 0;

	//bflb_uart_init(uart, &(cfg->config));
	divider = (uart_bflb_get_clock() * 10 / cfg->baudrate + 5) / 10;
	if (divider >= 0xFFFF)
	{
		divider = 0xFFFF - 1;
	}

	uart_bflb_enabled(dev, 0);

	sys_write32(((divider - 1) << 0x10) | ((divider - 1) & 0xFFFF), \
cfg->base_reg + UART_BIT_PRD_OFFSET);


	/* Configure Parity */
	tx_cfg = sys_read32(cfg->base_reg + UART_UTX_CONFIG_OFFSET);
	rx_cfg = sys_read32(cfg->base_reg + UART_URX_CONFIG_OFFSET);

	switch (cfg->parity) {
		case UART_PARITY_NONE:
			tx_cfg &= ~UART_CR_UTX_PRT_EN;
			rx_cfg &= ~UART_CR_URX_PRT_EN;
			break;
		case UART_PARITY_ODD:
			tx_cfg |= UART_CR_UTX_PRT_EN;
			tx_cfg |= UART_CR_UTX_PRT_SEL;
			rx_cfg |= UART_CR_URX_PRT_EN;
			rx_cfg |= UART_CR_URX_PRT_SEL;
			break;
		case UART_PARITY_EVEN:
			tx_cfg |= UART_CR_UTX_PRT_EN;
			tx_cfg &= ~UART_CR_UTX_PRT_SEL;
			rx_cfg |= UART_CR_URX_PRT_EN;
			rx_cfg &= ~UART_CR_URX_PRT_SEL;
			break;
		default:
			break;
	}

	/* Configure data bits */
	tx_cfg &= ~UART_CR_UTX_BIT_CNT_D_MASK;
	tx_cfg |= (cfg->data_bits + 4) << UART_CR_UTX_BIT_CNT_D_SHIFT;
	rx_cfg &= ~UART_CR_URX_BIT_CNT_D_MASK;
	rx_cfg |= (cfg->data_bits + 4) << UART_CR_URX_BIT_CNT_D_SHIFT;

	/* Configure tx stop bits */
	tx_cfg &= ~UART_CR_UTX_BIT_CNT_P_MASK;
	tx_cfg |= cfg->stop_bits << UART_CR_UTX_BIT_CNT_P_SHIFT;

	/* Configure tx cts flow control function */
	if (cfg->flow_ctrl & UART_FLOWCTRL_CTS) {
		tx_cfg |= UART_CR_UTX_CTS_EN;
	} else {
		tx_cfg &= ~UART_CR_UTX_CTS_EN;
	}

	rx_cfg &= ~UART_CR_URX_DEG_EN;


	/* Write config */
	sys_write32(tx_cfg, cfg->base_reg + UART_UTX_CONFIG_OFFSET);
	sys_write32(rx_cfg, cfg->base_reg + UART_URX_CONFIG_OFFSET);

	/* enable hardware control RTS */
	#if defined(CONFIG_SOC_SERIES_BL6)
	tmpVal = sys_read32(cfg->base_reg + UART_URX_CONFIG_OFFSET);
	tmpVal &= ~UART_CR_URX_RTS_SW_MODE;
	sys_write32(tmpVal, cfg->base_reg + UART_URX_CONFIG_OFFSET);
	#else
	tmpVal = sys_read32(cfg->base_reg + UART_SW_MODE_OFFSET);
	tmpVal &= ~UART_CR_URX_RTS_SW_MODE;
	sys_write32(tmpVal, cfg->base_reg + UART_SW_MODE_OFFSET);
	#endif

	/* disable inversion */
	tmpVal = sys_read32(cfg->base_reg + UART_DATA_CONFIG_OFFSET);
	tmpVal &= ~UART_CR_UART_BIT_INV;
	sys_write32(tmpVal, cfg->base_reg + UART_DATA_CONFIG_OFFSET);


	/* TX free run enable */
	tmpVal = sys_read32(cfg->base_reg + UART_UTX_CONFIG_OFFSET);
	tmpVal |= UART_CR_UTX_FRM_EN;
	sys_write32(tmpVal, cfg->base_reg + UART_UTX_CONFIG_OFFSET);


	/* Configure FIFO thresholds */
	tmpVal = sys_read32(cfg->base_reg + UART_FIFO_CONFIG_1_OFFSET);
	tmpVal &= ~UART_TX_FIFO_TH_MASK;
	tmpVal &= ~UART_RX_FIFO_TH_MASK;
	tmpVal |= (cfg->tx_fifo_threshold << UART_TX_FIFO_TH_SHIFT) & UART_TX_FIFO_TH_MASK;
	tmpVal |= (cfg->rx_fifo_threshold << UART_RX_FIFO_TH_SHIFT) & UART_RX_FIFO_TH_MASK;
	sys_write32(tmpVal, cfg->base_reg + UART_FIFO_CONFIG_1_OFFSET);

	/* Clear FIFO */
	tmpVal = sys_read32(cfg->base_reg + UART_FIFO_CONFIG_0_OFFSET);
	tmpVal |= UART_TX_FIFO_CLR;
	tmpVal |= UART_RX_FIFO_CLR;
	tmpVal &= ~UART_DMA_TX_EN;
	tmpVal &= ~UART_DMA_RX_EN;
	sys_write32(tmpVal, cfg->base_reg + UART_FIFO_CONFIG_0_OFFSET);

	/* enable all irqs */
	sys_write32(0xFFFFFFFF, cfg->base_reg + UART_INT_MASK_OFFSET);

	uart_bflb_enabled(dev, 1);

	return 0;
}

static int ITCMF uart_bflb_init(const struct device *dev)
{
	const struct bflb_config *cfg = dev->config;

	pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	return uart_bflb_configure(dev);
}

static int ITCMF uart_bflb_poll_in(const struct device *dev, unsigned char *c)
{
	const struct bflb_config *cfg = dev->config;

	if ((sys_read32(cfg->base_reg + UART_FIFO_CONFIG_1_OFFSET) & UART_RX_FIFO_CNT_MASK) != 0) {
		*c = sys_read8(cfg->base_reg + UART_FIFO_RDATA_OFFSET);
		return 0;
	}

	return -1;
}

static void ITCMF uart_bflb_poll_out(const struct device *dev, unsigned char c)
{
	const struct bflb_config *cfg = dev->config;

	uint64_t start_time;

	start_time = sys_clock_cycle_get_64();
	while ((sys_read32(cfg->base_reg + UART_FIFO_CONFIG_1_OFFSET) & UART_TX_FIFO_CNT_MASK) == 0)
	{
		/* TODO: Incorrect usage of mtime */
		if ((sys_clock_cycle_get_64() - start_time) > 100000) {
			return;
		}
	}
	sys_write8(c, cfg->base_reg + UART_FIFO_WDATA_OFFSET);
	return;
}

#ifdef CONFIG_PM_DEVICE
static int uart_bl_pm_control(const struct device *dev,
			      enum pm_device_action action)
{
	const struct bl_config *cfg = dev->config;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		(void)pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
		const struct bflb_config *cfg = dev->config;
		uint32_t tx_cfg;
		uint32_t rx_cfg;

		tx_cfg = sys_read32(cfg->base_reg + UART_UTX_CONFIG_OFFSET);
		rx_cfg = sys_read32(cfg->base_reg + UART_URX_CONFIG_OFFSET);
		tx_cfg |= UART_CR_UTX_EN;
		rx_cfg |= UART_CR_URX_EN;
		sys_write32(tx_cfg, cfg->base_reg + UART_UTX_CONFIG_OFFSET);
		sys_write32(rx_cfg, cfg->base_reg + UART_URX_CONFIG_OFFSET);
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		if (pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_SLEEP)) {
			return -134;
		}
		const struct bflb_config *cfg = dev->config;
		uint32_t tx_cfg;
		uint32_t rx_cfg;

		tx_cfg = sys_read32(cfg->base_reg + UART_UTX_CONFIG_OFFSET);
		rx_cfg = sys_read32(cfg->base_reg + UART_URX_CONFIG_OFFSET);
		tx_cfg &= ~UART_CR_UTX_EN;
		rx_cfg &= ~UART_CR_URX_EN;
		sys_write32(tx_cfg, cfg->base_reg + UART_UTX_CONFIG_OFFSET);
		sys_write32(rx_cfg, cfg->base_reg + UART_URX_CONFIG_OFFSET);
		break;
	default:
		return -134;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct uart_driver_api uart_bflb_driver_api = {
	.poll_in = uart_bflb_poll_in,
	.poll_out = uart_bflb_poll_out,
};

#define BL_UART_INIT(instance)							\
	PINCTRL_DT_INST_DEFINE(instance);					\
	static const struct bflb_config bl_uart##instance##_config = {		\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(instance),		\
		.base_reg = DT_INST_REG_ADDR(instance),				\
										\
		.baudrate = DT_INST_PROP(instance, current_speed),	\
		.data_bits = UART_DATA_BITS_8,				\
		.stop_bits = UART_STOP_BITS_1,				\
		.parity = UART_PARITY_NONE,				\
		.bit_order = UART_MSB_FIRST,				\
		.flow_ctrl = UART_FLOWCTRL_NONE,			\
		.tx_fifo_threshold = 15,					\
		.rx_fifo_threshold = 15,					\
										\
	};									\
	DEVICE_DT_INST_DEFINE(instance, &uart_bflb_init,			\
			      uart_bl_pm_control,				\
			      NULL,						\
			      &bl_uart##instance##_config, PRE_KERNEL_1,	\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			      &uart_bflb_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BL_UART_INIT)
