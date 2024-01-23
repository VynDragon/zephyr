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

#include <soc.h>

#include <bflb_uart.h>
#include <bflb_clock.h>
#include <hardware/uart_reg.h>

struct bflb_config {
	const struct pinctrl_dev_config *pincfg;
	struct bflb_uart_config_s config;
	uint32_t base_reg;
};


static uint8_t get_idx(uint32_t base_reg)
{
	if (base_reg == 0x4000A000)
	{
		return 0;
	}
	if (base_reg == 0x4000A100)
	{
		return 1;
	}
	return -1;
}

static int uart_bflb_configure(const struct device *dev)
{
	const struct bflb_config *cfg = dev->config;

	uint8_t idx = get_idx(cfg->base_reg);
	if (idx < 0)
	{
		return idx;
	}

	uint32_t div = 0;
	uint32_t tx_cfg;
	uint32_t rx_cfg;
	uint32_t reg_base;
	uint32_t regval;

	reg_base = cfg->base_reg;

	div = (bflb_clk_get_peripheral_clock(BFLB_DEVICE_TYPE_UART, idx) * 10 / cfg->config.baudrate + 5) / 10;

	tx_cfg = getreg32(reg_base + UART_UTX_CONFIG_OFFSET);
	rx_cfg = getreg32(reg_base + UART_URX_CONFIG_OFFSET);
	tx_cfg &= ~UART_CR_UTX_EN;
	rx_cfg &= ~UART_CR_URX_EN;
	putreg32(tx_cfg, reg_base + UART_UTX_CONFIG_OFFSET);
	putreg32(rx_cfg, reg_base + UART_URX_CONFIG_OFFSET);

	putreg32(((div - 1) << 0x10) | ((div - 1) & 0xFFFF), reg_base + UART_BIT_PRD_OFFSET);

	/* configure parity type */

	tx_cfg = getreg32(reg_base + UART_UTX_CONFIG_OFFSET);
	rx_cfg = getreg32(reg_base + UART_URX_CONFIG_OFFSET);

	switch (cfg->config.parity) {
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
	tx_cfg |= (cfg->config.data_bits + 4) << UART_CR_UTX_BIT_CNT_D_SHIFT;
	rx_cfg &= ~UART_CR_URX_BIT_CNT_D_MASK;
	rx_cfg |= (cfg->config.data_bits + 4) << UART_CR_URX_BIT_CNT_D_SHIFT;

	/* Configure tx stop bits */
	tx_cfg &= ~UART_CR_UTX_BIT_CNT_P_MASK;
	tx_cfg |= cfg->config.stop_bits << UART_CR_UTX_BIT_CNT_P_SHIFT;

	/* Configure tx cts flow control function */
	if (cfg->config.flow_ctrl & UART_FLOWCTRL_CTS) {
		tx_cfg |= UART_CR_UTX_CTS_EN;
	} else {
		tx_cfg &= ~UART_CR_UTX_CTS_EN;
	}

	rx_cfg &= ~UART_CR_URX_DEG_EN;

	/* Write back */
	putreg32(tx_cfg, reg_base + UART_UTX_CONFIG_OFFSET);
	putreg32(rx_cfg, reg_base + UART_URX_CONFIG_OFFSET);
#if defined(BL602)
	regval = getreg32(reg_base + UART_URX_CONFIG_OFFSET);
	regval &= ~UART_CR_URX_RTS_SW_MODE;
	putreg32(regval, reg_base + UART_URX_CONFIG_OFFSET);

#else
	regval = getreg32(reg_base + UART_SW_MODE_OFFSET);
	regval &= ~UART_CR_URX_RTS_SW_MODE;
	putreg32(regval, reg_base + UART_SW_MODE_OFFSET);
#endif
	regval = getreg32(reg_base + UART_DATA_CONFIG_OFFSET);
	regval &= ~UART_CR_UART_BIT_INV;
	putreg32(regval, reg_base + UART_DATA_CONFIG_OFFSET);

	/* Enable tx free run mode */
	regval = getreg32(reg_base + UART_UTX_CONFIG_OFFSET);
	regval |= UART_CR_UTX_FRM_EN;
	putreg32(regval, reg_base + UART_UTX_CONFIG_OFFSET);

	/* Configure FIFO thresholds */
	regval = getreg32(reg_base + UART_FIFO_CONFIG_1_OFFSET);
	regval &= ~UART_TX_FIFO_TH_MASK;
	regval &= ~UART_RX_FIFO_TH_MASK;
	regval |= (cfg->config.tx_fifo_threshold << UART_TX_FIFO_TH_SHIFT) & UART_TX_FIFO_TH_MASK;
	regval |= (cfg->config.rx_fifo_threshold << UART_RX_FIFO_TH_SHIFT) & UART_RX_FIFO_TH_MASK;
	putreg32(regval, reg_base + UART_FIFO_CONFIG_1_OFFSET);

	/* Clear FIFO */
	regval = getreg32(reg_base + UART_FIFO_CONFIG_0_OFFSET);
	regval |= UART_TX_FIFO_CLR;
	regval |= UART_RX_FIFO_CLR;
	regval &= ~UART_DMA_TX_EN;
	regval &= ~UART_DMA_RX_EN;
	putreg32(regval, reg_base + UART_FIFO_CONFIG_0_OFFSET);

	putreg32(0xFFFFFFFF, reg_base + UART_INT_MASK_OFFSET);

	/* Enable UART tx rx unit */
	tx_cfg = getreg32(reg_base + UART_UTX_CONFIG_OFFSET);
	rx_cfg = getreg32(reg_base + UART_URX_CONFIG_OFFSET);
	tx_cfg |= UART_CR_UTX_EN;
	rx_cfg |= UART_CR_URX_EN;
	putreg32(tx_cfg, reg_base + UART_UTX_CONFIG_OFFSET);
	putreg32(rx_cfg, reg_base + UART_URX_CONFIG_OFFSET);

	return 0;
}

static int uart_bflb_init(const struct device *dev)
{
	const struct bflb_config *cfg = dev->config;

	uint8_t idx = get_idx(cfg->base_reg);
	if (idx < 0)
	{
		return idx;
	}

	pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);

	GLB_Set_UART_CLK(1, HBN_UART_CLK_160M, 0);

	return uart_bflb_configure(dev);

	return 0;
}

static int uart_bflb_poll_in(const struct device *dev, unsigned char *c)
{
	const struct bflb_config *cfg = dev->config;

	if ((getreg32(cfg->base_reg + UART_FIFO_CONFIG_1_OFFSET) & UART_RX_FIFO_CNT_MASK) != 0) {
		*c = getreg8(cfg->base_reg + UART_FIFO_RDATA_OFFSET);
	}

	return *c;
}

static void uart_bflb_poll_out(const struct device *dev, unsigned char c)
{
	const struct bflb_config *cfg = dev->config;

	uint64_t start_time;

	start_time = bflb_mtimer_get_time_ms();
	while ((getreg32(cfg->base_reg + UART_FIFO_CONFIG_1_OFFSET) & UART_TX_FIFO_CNT_MASK) == 0) {
		if ((bflb_mtimer_get_time_ms() - start_time) > 100) {
			return;
		}
	}
	putreg8(c, cfg->base_reg + UART_FIFO_WDATA_OFFSET);
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

		tx_cfg = getreg32(cfg->base_reg + UART_UTX_CONFIG_OFFSET);
		rx_cfg = getreg32(cfg->base_reg + UART_URX_CONFIG_OFFSET);
		tx_cfg |= UART_CR_UTX_EN;
		rx_cfg |= UART_CR_URX_EN;
		putreg32(tx_cfg, cfg->base_reg + UART_UTX_CONFIG_OFFSET);
		putreg32(rx_cfg, cfg->base_reg + UART_URX_CONFIG_OFFSET);
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		if (pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_SLEEP)) {
			return -134;
		}
		const struct bflb_config *cfg = dev->config;
		uint32_t tx_cfg;
		uint32_t rx_cfg;

		tx_cfg = getreg32(cfg->base_reg + UART_UTX_CONFIG_OFFSET);
		rx_cfg = getreg32(cfg->base_reg + UART_URX_CONFIG_OFFSET);
		tx_cfg &= ~UART_CR_UTX_EN;
		rx_cfg &= ~UART_CR_URX_EN;
		putreg32(tx_cfg, cfg->base_reg + UART_UTX_CONFIG_OFFSET);
		putreg32(rx_cfg, cfg->base_reg + UART_URX_CONFIG_OFFSET);
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
		.config.baudrate = DT_INST_PROP(instance, current_speed),	\
		.config.data_bits = UART_DATA_BITS_8,				\
		.config.stop_bits = UART_STOP_BITS_1,				\
		.config.parity = UART_PARITY_NONE,				\
		.config.bit_order = UART_MSB_FIRST,				\
		.config.flow_ctrl = UART_FLOWCTRL_RTS_CTS,			\
		.config.tx_fifo_threshold = 1,					\
		.config.rx_fifo_threshold = 1,					\
										\
	};									\
	DEVICE_DT_INST_DEFINE(instance, &uart_bflb_init,			\
			      uart_bl_pm_control,				\
			      NULL,						\
			      &bl_uart##instance##_config, PRE_KERNEL_1,	\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			      &uart_bflb_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BL_UART_INIT)
