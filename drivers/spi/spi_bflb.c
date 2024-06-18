/*
 * Copyright (c) 2024 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT bflb_bl_spi

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>



#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_bflb);

#include "spi_context.h"

/* Register Offsets */
#include <soc.h>
#include <common_hardware/spi_reg.h>

#if defined(CONFIG_SOC_SERIES_BL6) || defined(CONFIG_SOC_SERIES_BL7)
#define GLB_SPI_MODE_ADDRESS 0x40000080
#elif defined(CONFIG_SOC_SERIES_BL61)
#define GLB_SPI_MODE_ADDRESS 0x20000510
#endif

#define SPI_BUFFER_SIZE 512
#define SPI_WAIT_TIMEOUT_MS 100

/* Structure declarations */

struct spi_bflb_cfg {
	const struct pinctrl_dev_config *pincfg;
	uint32_t base;
	uint32_t dts_freq;
	void (*irq_config_func)(const struct device *);
};

struct spi_bflb_data {
	struct spi_context ctx;
	/* bit 0: we are busy, */
	atomic_t atomic_data;
};

static uint8_t spi_bflb_buffer[SPI_BUFFER_SIZE] = {0};
static uint8_t *spi_bflb_buffer_pointer = &(spi_bflb_buffer[0]);
static uint8_t *spi_bflb_buffer_pointer_written = &(spi_bflb_buffer[0]);


/* this will go in clock driver when clock driver is a thing */
#ifdef CONFIG_SOC_SERIES_BL6

static uint32_t uart_bflb_get_crystal_frequency(void)
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

static uint32_t uart_bflb_get_PLL_frequency(void)
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

#elif defined( CONFIG_SOC_SERIES_BL7 )


static uint32_t uart_bflb_get_crystal_frequency(void)
{
	return (32 * 1000 * 1000);
}

static uint32_t uart_bflb_get_PLL_frequency(void)
{
	uint32_t tmpVal;

	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_REG_PLL_SEL_MSK) >> GLB_REG_PLL_SEL_POS;

	if (tmpVal == 0) {
		return (57 * 1000 * 1000 + 6 * 100 * 1000);
	} else if (tmpVal == 1) {
		return (96 * 1000 * 1000);
	} else if (tmpVal == 2) {
		return (144 * 1000 * 1000);
	} else if (tmpVal == 3) {
		return (288 * 1000 * 1000);
	} else {
		return 0;
	}
}

#endif

static uint32_t spi_bflb_get_bclk_clk(void)
{
	uint32_t tmpVal = 0;
	uint32_t spi_divider = 0;
	uint32_t hclk_divider = 0;
	uint32_t bclk_divider = 0;

	/* root -> HCLK */
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	hclk_divider = (tmpVal & GLB_REG_HCLK_DIV_MSK) >> GLB_REG_HCLK_DIV_POS;

	/* HCLK -> BCLK */
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	bclk_divider = (tmpVal & GLB_REG_BCLK_DIV_MSK) >> GLB_REG_BCLK_DIV_POS;

	/* bclk -> spiclk */
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG3_OFFSET);
	spi_divider = (tmpVal & GLB_SPI_CLK_DIV_MSK) >> GLB_SPI_CLK_DIV_POS;

	/* what is root */
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_HBN_ROOT_CLK_SEL_MSK) >> GLB_HBN_ROOT_CLK_SEL_POS;

	if (tmpVal == 0)
	{
		/* RC32M clock */
		tmpVal = (32 * 1000 * 1000) / (hclk_divider + 1)
		/ (bclk_divider + 1) / (spi_divider + 1);
		return tmpVal;
	}
	else if (tmpVal == 1)
	{
		/* Crystal clock */
		tmpVal = uart_bflb_get_crystal_frequency() / (hclk_divider + 1)
		/ (bclk_divider + 1) / (spi_divider + 1);
		return tmpVal;
	}
	else if (tmpVal > 1)
	{
		/* PLL Clock */
		tmpVal = uart_bflb_get_PLL_frequency() / (hclk_divider + 1)
		/ (bclk_divider + 1) / (spi_divider + 1);
		return tmpVal;

	}
	return 0;
}

static void spi_bflb_trigger(const struct device *dev)
{
	uint32_t tmpVal = 0;
	const struct spi_bflb_cfg *config = dev->config;

	tmpVal = sys_read32(config->base + SPI_CONFIG_OFFSET);
	tmpVal |= SPI_CR_SPI_M_EN;
	sys_write32(tmpVal, config->base + SPI_CONFIG_OFFSET);
}

static void spi_bflb_detrigger(const struct device *dev)
{
	uint32_t tmpVal = 0;
	const struct spi_bflb_cfg *config = dev->config;

	tmpVal = sys_read32(config->base + SPI_CONFIG_OFFSET);
	tmpVal &= ~SPI_CR_SPI_M_EN;
	sys_write32(tmpVal, config->base + SPI_CONFIG_OFFSET);
}

static int spi_bflb_triggered(const struct device *dev)
{
	const struct spi_bflb_cfg *config = dev->config;

	return((sys_read32(config->base + SPI_CONFIG_OFFSET) & SPI_CR_SPI_M_EN) != 0);
}


static int spi_bflb_configure_freqs(const struct device *dev, const struct spi_config *config)
{
	const struct spi_bflb_cfg *cfg = dev->config;
	struct spi_bflb_data *data = dev->data;
	uint32_t tmpVal = 0;
	uint32_t tmpValb = 0;

	if (config->frequency > 40 * 1000 * 1000)
	{
		return -EINVAL;
	}

	/* set spi clock divider and make sure enabled */
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG3_OFFSET);
	tmpVal |= GLB_SPI_CLK_EN_MSK;
	/* 40 MHz speed divider is good down to 156250 Hz */
	tmpVal &= ~GLB_SPI_CLK_DIV_MSK;
	tmpVal |= 1 << GLB_SPI_CLK_DIV_POS;
	sys_write32(tmpVal, GLB_BASE + GLB_CLK_CFG3_OFFSET);
	printf(spi_bflb_get_bclk_clk());
	tmpVal = (spi_bflb_get_bclk_clk() / 2 * 10 / config->frequency + 5) / 10;
	tmpVal = (tmpVal) ? (tmpVal - 1) : 0;
	tmpVal = (tmpVal > 0xff) ? 0xff : tmpVal;

	tmpValb = 0;
	tmpValb |= tmpVal << SPI_CR_SPI_PRD_D_PH_0_SHIFT;
	tmpValb |= tmpVal << SPI_CR_SPI_PRD_D_PH_1_SHIFT;
	tmpValb |= tmpVal << SPI_CR_SPI_PRD_S_SHIFT;
	tmpValb |= tmpVal << SPI_CR_SPI_PRD_P_SHIFT;
	sys_write32(tmpValb, cfg->base + SPI_PRD_0_OFFSET);

	tmpValb = sys_read32(cfg->base + SPI_PRD_1_OFFSET);
	tmpValb &= ~SPI_CR_SPI_PRD_I_MASK;
	tmpValb |= (tmpVal) << SPI_CR_SPI_PRD_I_SHIFT;
	sys_write32(tmpValb, cfg->base + SPI_PRD_1_OFFSET);



	return 0;
}


static int spi_bflb_configure(const struct device *dev, const struct spi_config *config)
{
	int rc = 0;
	const struct spi_bflb_cfg *cfg = dev->config;
	struct spi_bflb_data *data = dev->data;
	uint32_t tmpVal = 0;
	uint32_t framesize = 0;

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	tmpVal = sys_read32(cfg->base + SPI_CONFIG_OFFSET);
	/* detrigger SPI slave and master*/
	tmpVal &= ~SPI_CR_SPI_S_EN;
	tmpVal &= ~SPI_CR_SPI_M_EN;

	sys_write32(tmpVal, cfg->base + SPI_CONFIG_OFFSET);

	tmpVal = sys_read32(GLB_SPI_MODE_ADDRESS);
	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
		tmpVal |= 1 << 12;
	}
	else
	{
		/* TODO */
		return -ENOTSUP;
		tmpVal &= ~(1 << 12);
	}
	sys_write32(tmpVal, GLB_SPI_MODE_ADDRESS);


	rc = spi_bflb_configure_freqs(dev, config);
	if (rc !=0)
	{
		return rc;
	}


	tmpVal = sys_read32(cfg->base + SPI_CONFIG_OFFSET);
	/* Disable deglitch */
	tmpVal &= ~SPI_CR_SPI_DEG_EN;
	tmpVal &= ~SPI_CR_SPI_DEG_CNT_MASK;
	/* enable continue transaction as long as valid (doesnt really matter, CS is GPIO)*/
	tmpVal |= SPI_CR_SPI_M_CONT_EN;
	/* disable ignore RX */
	tmpVal &= ~SPI_CR_SPI_RXD_IGNR_EN;

	/* bit order*/
	if ((config->operation & SPI_TRANSFER_LSB) != 0)
	{
		tmpVal |= SPI_CR_SPI_BIT_INV;
	}
	else
	{
		tmpVal &= ~SPI_CR_SPI_BIT_INV;
	}

	/* disable byte order inversion */
	tmpVal &= ~SPI_CR_SPI_BYTE_INV;


	if ((config->operation & SPI_MODE_CPOL) == 0 && (config->operation & SPI_MODE_CPHA) == 0)
	{
		tmpVal &= ~SPI_CR_SPI_SCLK_POL;
		tmpVal |= SPI_CR_SPI_SCLK_PH;
		//tmpVal &= ~SPI_CR_SPI_SCLK_PH;
	}
	else if ((config->operation & SPI_MODE_CPOL) == 0 && (config->operation & SPI_MODE_CPHA) !=
0)
	{
		tmpVal &= ~SPI_CR_SPI_SCLK_POL;
		tmpVal &= ~SPI_CR_SPI_SCLK_PH;
		//tmpVal |= SPI_CR_SPI_SCLK_PH;
	}
	else if ((config->operation & SPI_MODE_CPOL) != 0 && (config->operation & SPI_MODE_CPHA) ==
0)
	{
		tmpVal |= SPI_CR_SPI_SCLK_POL;
		tmpVal |= SPI_CR_SPI_SCLK_PH;
		//tmpVal &= ~SPI_CR_SPI_SCLK_PH;
	}
	else if ((config->operation & SPI_MODE_CPOL) != 0 && (config->operation & SPI_MODE_CPHA) !=
0)
	{
		tmpVal |= SPI_CR_SPI_SCLK_POL;
		tmpVal &= ~SPI_CR_SPI_SCLK_PH;
		//tmpVal |= SPI_CR_SPI_SCLK_PH;
	}

	/* set expected data frame size*/
	framesize = (config->operation & 0x7E0) >> 5;
	if (framesize != 8 && framesize != 16 && framesize != 24 && framesize != 32)
	{
		return -EINVAL;
	}
	tmpVal &= ~SPI_CR_SPI_FRAME_SIZE_MASK;
	if(framesize == 16)
	{
		tmpVal |=  1 << SPI_CR_SPI_FRAME_SIZE_SHIFT;
	}
	else if(framesize == 24)
	{
		tmpVal |= 2 << SPI_CR_SPI_FRAME_SIZE_SHIFT;
	}
	else if(framesize == 32)
	{
		tmpVal |= 3 << SPI_CR_SPI_FRAME_SIZE_SHIFT;
	}
	/* detrigger SPI slave and master*/
	tmpVal &= ~SPI_CR_SPI_S_EN;
	tmpVal &= ~SPI_CR_SPI_M_EN;

	sys_write32(tmpVal, cfg->base + SPI_CONFIG_OFFSET);

	/* clear fifo and make sure DMA is disabled */
	tmpVal = sys_read32(cfg->base + SPI_FIFO_CONFIG_0_OFFSET);
	tmpVal |= SPI_TX_FIFO_CLR;
	tmpVal |= SPI_RX_FIFO_CLR;
	tmpVal &= ~SPI_DMA_TX_EN;
	tmpVal &= ~SPI_DMA_RX_EN;
	sys_write32(tmpVal, cfg->base + SPI_FIFO_CONFIG_0_OFFSET);

	/* fifo thresholds to 0*/
	tmpVal = sys_read32(cfg->base + SPI_FIFO_CONFIG_1_OFFSET);
	tmpVal &= ~SPI_TX_FIFO_TH_MASK;
	tmpVal &= ~SPI_RX_FIFO_TH_MASK;
	sys_write32(tmpVal, cfg->base  + SPI_FIFO_CONFIG_1_OFFSET);

	tmpVal = sys_read32(cfg->base + SPI_INT_STS_OFFSET);

	/* enable all interrupts */
	tmpVal |= (SPI_CR_SPI_END_EN |
		SPI_CR_SPI_TXF_EN |
		SPI_CR_SPI_RXF_EN |
		SPI_CR_SPI_STO_EN |
		SPI_CR_SPI_TXU_EN |
		SPI_CR_SPI_FER_EN);
	/* mask some interrupts */
	tmpVal |= ( SPI_CR_SPI_STO_MASK |
		SPI_CR_SPI_TXU_MASK |
		SPI_CR_SPI_FER_MASK |
		SPI_CR_SPI_TXF_MASK |
		SPI_CR_SPI_RXF_MASK);

	/* unmask some interrupts */
	tmpVal &= ~SPI_CR_SPI_END_MASK;

	sys_write32(tmpVal, cfg->base + SPI_INT_STS_OFFSET);

	data->ctx.config = config;

	return 0;
}

static void spi_bflb_isr_END(const struct device *dev)
{
	const struct spi_bflb_cfg *config = dev->config;
	struct spi_bflb_data *data = dev->data;
	uint32_t tmpVal = 0;

	/* transmit done */
	spi_bflb_detrigger(dev);
	atomic_clear_bit(&(data->atomic_data), 0);
}

static void spi_bflb_isr_TXF(const struct device *dev)
{
	const struct spi_bflb_cfg *config = dev->config;
	struct spi_bflb_data *data = dev->data;
	uint32_t tmpVal = 0;
	uint32_t i = 0;
	bool fed = false;
	uint32_t framesize = 0;

	/* feed one */
	tmpVal = sys_read32(config->base + SPI_CONFIG_OFFSET);
	framesize = ((tmpVal & SPI_CR_SPI_FRAME_SIZE_MASK) >> SPI_CR_SPI_FRAME_SIZE_SHIFT) + 1;
	tmpVal = 0;
	while (i < framesize && spi_bflb_buffer_pointer_written != spi_bflb_buffer_pointer)
	{
		fed = true;
		tmpVal |= ((*spi_bflb_buffer_pointer_written) << ((i % 4) * 8));
		i++;
		spi_bflb_buffer_pointer_written = spi_bflb_buffer_pointer_written + 1;
		if (spi_bflb_buffer_pointer_written >= spi_bflb_buffer + SPI_BUFFER_SIZE)
		{
			spi_bflb_buffer_pointer_written = spi_bflb_buffer;
		}
	}
	sys_write32(tmpVal, config->base + SPI_FIFO_WDATA_OFFSET);
	if (fed)
	{
		spi_bflb_trigger(dev);
	}
	else
	{
		spi_bflb_detrigger(dev);
		atomic_clear_bit(&(data->atomic_data), 0);
	}
}


static void spi_bflb_isr_RXF(const struct device *dev)
{
	struct spi_bflb_data *data = dev->data;

	atomic_set_bit(&(data->atomic_data), 1);
}

static int spi_bflb_transceive_fill(const struct device *dev, const struct spi_buf_set *tx_bufs)
{
	const struct spi_bflb_cfg *config = dev->config;
	struct spi_bflb_data *data = dev->data;
	uint32_t tmpVal = 0;
	uint32_t timeout = 0;
	uint32_t i = 0;
	uint32_t j = 0;
	bool started = false;

	/* wait for transmission end*/
	while (atomic_test_bit(&(data->atomic_data), 0) && timeout < SPI_WAIT_TIMEOUT_MS * 200)
	{
		k_sleep(K_USEC(5));
		timeout++;
	}
	if (timeout >= SPI_WAIT_TIMEOUT_MS * 200)
	{
		return -ETIMEDOUT;
	}
	atomic_set_bit(&(data->atomic_data), 0);

	/* clean up TX */
	tmpVal = sys_read32(config->base + SPI_FIFO_CONFIG_0_OFFSET);
	tmpVal |= SPI_TX_FIFO_CLR;
	sys_write32(tmpVal, config->base + SPI_FIFO_CONFIG_0_OFFSET);

	if (!tx_bufs) {
		return -EINVAL;
	}

	/* unmask txf */
	tmpVal = sys_read32(config->base + SPI_INT_STS_OFFSET);
	tmpVal &= ~SPI_CR_SPI_TXF_MASK;
	sys_write32(tmpVal, config->base + SPI_INT_STS_OFFSET);
	for(i = 0; i < tx_bufs->count; i++)
	{
		for (j = 0; j < tx_bufs->buffers[i].len; j++)
		{
			(*spi_bflb_buffer_pointer) = ((uint8_t*)tx_bufs->buffers[i].buf)[j];
			spi_bflb_buffer_pointer += 1;
			if (spi_bflb_buffer_pointer >= spi_bflb_buffer + SPI_BUFFER_SIZE)
			{
				spi_bflb_buffer_pointer = spi_bflb_buffer;
			}
		}
		/* write one to start process */
		if (!started)
		{
			spi_bflb_isr_TXF(dev);
			started = true;
		}
	}


	return 0;
}

static int spi_bflb_transceive_sync(	const struct device *dev,
					const struct spi_config *config,
					const struct spi_buf_set *tx_bufs,
					const struct spi_buf_set *rx_bufs)
{
	int rc = 0;
	const struct spi_bflb_cfg *cfg = dev->config;
	struct spi_bflb_data *data = dev->data;

	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	rc = spi_bflb_configure(dev, config);
	if (rc != 0) {
		return rc;
	}
	spi_bflb_detrigger(dev);
	spi_context_cs_control(&data->ctx, true);
	spi_bflb_transceive_fill(dev, tx_bufs);


	while (atomic_test_bit(&(data->atomic_data), 0))
	{
		k_sleep(K_USEC(1));
	}
	spi_context_cs_control(&data->ctx, false);
	spi_bflb_detrigger(dev);
	spi_context_release(&data->ctx, rc);
	return rc;
}

static int spi_bflb_init(const struct device *dev)
{
	int rc = 0;
	const struct spi_bflb_cfg *cfg = dev->config;
	struct spi_bflb_data *data = dev->data;
	uint32_t tmpVal = 0;


	rc = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (rc < 0) {
		return rc;
	}

	rc = spi_context_cs_configure_all(&data->ctx);
	if (rc < 0) {
		return rc;
	}

	tmpVal = sys_read32(cfg->base + SPI_INT_STS_OFFSET);

	/* mask interrupts */
	tmpVal |= ( SPI_CR_SPI_STO_MASK |
		SPI_CR_SPI_TXU_MASK |
		SPI_CR_SPI_FER_MASK |
		SPI_CR_SPI_TXF_MASK |
		SPI_CR_SPI_RXF_MASK |
		SPI_CR_SPI_END_MASK);

	sys_write32(tmpVal, cfg->base + SPI_INT_STS_OFFSET);
	data->atomic_data = ATOMIC_INIT(0);
	cfg->irq_config_func(dev);

	spi_context_unlock_unconditionally(&data->ctx);

	return rc;
}


static int spi_bflb_release(const struct device *dev,
		       const struct spi_config *config)
{
	struct spi_bflb_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static void spi_bflb_isr(const struct device *dev)
{
	const struct spi_bflb_cfg *config = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(config->base + SPI_INT_STS_OFFSET);

	if ((tmpVal & SPI_END_INT) != 0)
	{
		spi_bflb_isr_END(dev);
		tmpVal |= SPI_CR_SPI_END_CLR;
	}
	if ((tmpVal & SPI_TXF_INT) != 0)
	{
		spi_bflb_isr_TXF(dev);
	}
	if ((tmpVal & SPI_RXF_INT) != 0)
	{
		spi_bflb_isr_RXF(dev);
	}
	sys_write32(tmpVal, config->base + SPI_INT_STS_OFFSET);
}

static const struct spi_driver_api spi_bflb_driver_api = {
	.transceive = spi_bflb_transceive_sync,
	/* TODO
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_bflb_transceive_async,
#endif*/
	.release = spi_bflb_release,
};


/* Device instantiation */

#define SPI_BFLB_IRQ_HANDLER_DECL(n)						\
	static void spi_bflb_config_func_##n(const struct device *dev);
#define SPI_BFLB_IRQ_HANDLER_FUNC(n)						\
	.irq_config_func = spi_bflb_config_func_##n
#define SPI_BFLB_IRQ_HANDLER(n)							\
	static void spi_bflb_config_func_##n(const struct device *dev)		\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n),					\
			    DT_INST_IRQ(n, priority),				\
			    spi_bflb_isr,					\
			    DEVICE_DT_INST_GET(n),				\
			    0);							\
		irq_enable(DT_INST_IRQN(n));					\
	}


#define SPI_BFLB_INIT(n) \
	PINCTRL_DT_INST_DEFINE(n);					\
	SPI_BFLB_IRQ_HANDLER_DECL(n)					\
	static struct spi_bflb_data spi##n##_bflb_data = {		\
		SPI_CONTEXT_INIT_LOCK(spi##n##_bflb_data, ctx), 	\
		SPI_CONTEXT_INIT_SYNC(spi##n##_bflb_data, ctx), 	\
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)	\
	}; 								\
	static const struct spi_bflb_cfg spi_bflb_cfg_##n = {		\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
		.base = DT_INST_REG_ADDR(n), 				\
		.dts_freq = DT_INST_PROP(n, clock_frequency),		\
		SPI_BFLB_IRQ_HANDLER_FUNC(n)				\
	}; 								\
	DEVICE_DT_INST_DEFINE(n,					\
			    spi_bflb_init,				\
			    NULL,					\
			    &spi##n##_bflb_data,			\
			    &spi_bflb_cfg_##n,				\
			    POST_KERNEL,				\
			    CONFIG_SPI_INIT_PRIORITY,			\
			    &spi_bflb_driver_api);			\
	SPI_BFLB_IRQ_HANDLER(n)

DT_INST_FOREACH_STATUS_OKAY(SPI_BFLB_INIT)
