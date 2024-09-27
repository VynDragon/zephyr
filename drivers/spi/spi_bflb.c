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
#include <zephyr/irq.h>



#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_bflb);

#include "spi_context.h"

/* Register Offsets */
#include <soc.h>
#include <bouffalolab/common/spi_reg.h>

#if defined(CONFIG_SOC_SERIES_BL60X) || defined(CONFIG_SOC_SERIES_BL70X)
#define GLB_SPI_MODE_ADDRESS 0x40000080
/* in words */
#define SPI_FIFO_SIZE 4
#elif defined(CONFIG_SOC_SERIES_BL61X)
#define GLB_SPI_MODE_ADDRESS 0x20000510
/* in bytes */
#define SPI_FIFO_SIZE 32
#endif
#define SPI_WAIT_TIMEOUT_MS 250

/* Structure declarations */

struct spi_bflb_cfg {
	const struct pinctrl_dev_config *pincfg;
	uint32_t base;
	void (*irq_config_func)(const struct device *dev);
};

struct spi_bflb_data {
	struct spi_context ctx;
};

/* this will go in clock driver when clock driver is a thing */
#ifdef CONFIG_SOC_SERIES_BL60X

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

#elif defined(CONFIG_SOC_SERIES_BL70X)


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

#elif defined(CONFIG_SOC_SERIES_BL61X)

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

#endif

#if defined(CONFIG_SOC_SERIES_BL70X) || defined(CONFIG_SOC_SERIES_BL60X)

static uint32_t spi_bflb_get_clk(void)
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

	if (tmpVal == 0) {
		/* RC32M clock */
		tmpVal = (32 * 1000 * 1000) / (hclk_divider + 1)
		/ (bclk_divider + 1) / (spi_divider + 1);
		return tmpVal;
	}
	else if (tmpVal == 1) {
		/* Crystal clock */
		tmpVal = uart_bflb_get_crystal_frequency() / (hclk_divider + 1)
		/ (bclk_divider + 1) / (spi_divider + 1);
		return tmpVal;
	}
	else if (tmpVal > 1) {
		/* PLL Clock */
		tmpVal = uart_bflb_get_PLL_frequency() / (hclk_divider + 1)
		/ (bclk_divider + 1) / (spi_divider + 1);
		return tmpVal;

	}
	return 0;
}

#elif defined(CONFIG_SOC_SERIES_BL61X)

static uint32_t spi_bflb_get_clk(void)
{
	return system_get_xclk();
}

#endif

static bool spi_bflb_bus_busy(const struct device *dev)
{
	uint32_t tmpVal = 0;
	const struct spi_bflb_cfg *config = dev->config;

	tmpVal = sys_read32(config->base + SPI_BUS_BUSY_OFFSET);
	return (tmpVal & SPI_STS_SPI_BUS_BUSY) == 0 ? false : true;
}
static void spi_bflb_trigger_master(const struct device *dev)
{
	uint32_t tmpVal = 0;
	const struct spi_bflb_cfg *config = dev->config;

	tmpVal = sys_read32(config->base + SPI_CONFIG_OFFSET);
	tmpVal |= SPI_CR_SPI_M_EN;
	sys_write32(tmpVal, config->base + SPI_CONFIG_OFFSET);
}

static void spi_bflb_detrigger_master(const struct device *dev)
{
	uint32_t tmpVal = 0;
	const struct spi_bflb_cfg *config = dev->config;

	tmpVal = sys_read32(config->base + SPI_CONFIG_OFFSET);
	tmpVal &= ~SPI_CR_SPI_M_EN;
	sys_write32(tmpVal, config->base + SPI_CONFIG_OFFSET);
}

static int spi_bflb_triggered_master(const struct device *dev)
{
	const struct spi_bflb_cfg *config = dev->config;

	return((sys_read32(config->base + SPI_CONFIG_OFFSET) & SPI_CR_SPI_M_EN) != 0);
}


static int spi_bflb_configure_freqs(const struct device *dev, const struct spi_config *config)
{
	const struct spi_bflb_cfg *cfg = dev->config;
	uint32_t tmpVal = 0;
	uint32_t tmpValb = 0;

	if (config->frequency > 40 * 1000 * 1000) {
		return -EINVAL;
	}

#if defined(CONFIG_SOC_SERIES_BL61X)
	tmpVal = sys_read32(GLB_BASE + GLB_SPI_CFG0_OFFSET);
	/* set div to 0 (1) */
	tmpVal = tmpVal & GLB_SPI_CLK_DIV_UMSK;
	tmpVal |= 0 << GLB_SPI_CLK_DIV_POS;
	tmpVal |= GLB_SPI_CLK_EN_MSK;
	/* select XCLK */
	tmpVal &= GLB_SPI_CLK_SEL_UMSK;
	tmpVal |= 1 <<  GLB_SPI_CLK_SEL_POS;
	sys_write32(tmpVal, GLB_BASE + GLB_SPI_CFG0_OFFSET);
#else
	/* set spi clock divider and make sure enabled */
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG3_OFFSET);
	tmpVal |= GLB_SPI_CLK_EN_MSK;
	/* 40 MHz speed divider is good down to 156250 Hz */
	tmpVal &= ~GLB_SPI_CLK_DIV_MSK;
	tmpVal |= 1 << GLB_SPI_CLK_DIV_POS;
	sys_write32(tmpVal, GLB_BASE + GLB_CLK_CFG3_OFFSET);
#endif
	tmpVal = (spi_bflb_get_clk() / 2 * 10 / config->frequency + 5) / 10;
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
	} else {
		/* TODO */
		return -ENOTSUP;
		tmpVal &= ~(1 << 12);
	}
	sys_write32(tmpVal, GLB_SPI_MODE_ADDRESS);


	rc = spi_bflb_configure_freqs(dev, config);
	if (rc !=0) {
		return rc;
	}


	tmpVal = sys_read32(cfg->base + SPI_CONFIG_OFFSET);
	/* Disable deglitch */
	tmpVal &= ~SPI_CR_SPI_DEG_EN;
	tmpVal &= ~SPI_CR_SPI_DEG_CNT_MASK;
	/* enable continue transaction as long as valid */
	tmpVal |= SPI_CR_SPI_M_CONT_EN;
	/* disable ignore RX */
	tmpVal &= ~SPI_CR_SPI_RXD_IGNR_EN;

	/* bit order*/
	if ((config->operation & SPI_TRANSFER_LSB) != 0) {
		tmpVal |= SPI_CR_SPI_BIT_INV;
	} else {
		tmpVal &= ~SPI_CR_SPI_BIT_INV;
	}

	/* disable byte order inversion */
	tmpVal &= ~SPI_CR_SPI_BYTE_INV;


	if ((config->operation & SPI_MODE_CPOL) == 0
		&& (config->operation & SPI_MODE_CPHA) == 0) {
		tmpVal &= ~SPI_CR_SPI_SCLK_POL;
		tmpVal |= SPI_CR_SPI_SCLK_PH;
		//tmpVal &= ~SPI_CR_SPI_SCLK_PH;
	}
	else if ((config->operation & SPI_MODE_CPOL) == 0
		&& (config->operation & SPI_MODE_CPHA) != 0) {
		tmpVal &= ~SPI_CR_SPI_SCLK_POL;
		tmpVal &= ~SPI_CR_SPI_SCLK_PH;
		//tmpVal |= SPI_CR_SPI_SCLK_PH;
	}
	else if ((config->operation & SPI_MODE_CPOL) != 0
		&& (config->operation & SPI_MODE_CPHA) == 0) {
		tmpVal |= SPI_CR_SPI_SCLK_POL;
		tmpVal |= SPI_CR_SPI_SCLK_PH;
		//tmpVal &= ~SPI_CR_SPI_SCLK_PH;
	}
	else if ((config->operation & SPI_MODE_CPOL) != 0
		&& (config->operation & SPI_MODE_CPHA) != 0) {
		tmpVal |= SPI_CR_SPI_SCLK_POL;
		tmpVal &= ~SPI_CR_SPI_SCLK_PH;
		//tmpVal |= SPI_CR_SPI_SCLK_PH;
	}

	/* set expected data frame size*/
	framesize = SPI_WORD_SIZE_GET(config->operation);
	if (framesize != 8 && framesize != 16
		&& framesize != 24 && framesize != 32) {
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

	/* fifo thresholds to 0 and 8
	 * tx triggers when pushed elements > 8
	 * rx trigger when received elements > 0
	 */
	tmpVal = sys_read32(cfg->base + SPI_FIFO_CONFIG_1_OFFSET);
	tmpVal &= ~SPI_TX_FIFO_TH_MASK;
	tmpVal &= ~SPI_RX_FIFO_TH_MASK;
	tmpVal |= 8 << SPI_TX_FIFO_TH_SHIFT;
	sys_write32(tmpVal, cfg->base  + SPI_FIFO_CONFIG_1_OFFSET);

	tmpVal = sys_read32(cfg->base + SPI_INT_STS_OFFSET);

	/* enable all interrupts */
	tmpVal |= (SPI_CR_SPI_END_EN |
		SPI_CR_SPI_TXF_EN |
		SPI_CR_SPI_RXF_EN |
		SPI_CR_SPI_STO_EN |
		SPI_CR_SPI_TXU_EN |
		SPI_CR_SPI_FER_EN);
	/* mask all interrupts */
	tmpVal |= ( SPI_CR_SPI_STO_MASK |
		SPI_CR_SPI_TXU_MASK |
		SPI_CR_SPI_FER_MASK |
		SPI_CR_SPI_TXF_MASK |
		SPI_CR_SPI_END_MASK |
		SPI_CR_SPI_RXF_MASK);

	sys_write32(tmpVal, cfg->base + SPI_INT_STS_OFFSET);

	data->ctx.config = config;

	return 0;
}

static uint32_t spi_bflb_from_buff_to_frame(const struct device *dev,
					    void *buff)
{
	const struct spi_bflb_cfg *cfg = dev->config;
	uint32_t tmpVal = 0;
	uint32_t frame = 0;

	tmpVal = sys_read32(cfg->base + SPI_CONFIG_OFFSET);
	tmpVal = (tmpVal & SPI_CR_SPI_FRAME_SIZE_MASK)
		>> SPI_CR_SPI_FRAME_SIZE_SHIFT;

	if(tmpVal == 0) {
		frame = *(uint32_t*)buff & 0xFF;
	} else if (tmpVal == 1) {
		frame = *(uint32_t*)buff & 0xFFFF;
	} else if (tmpVal == 2) {
		frame = *(uint32_t*)buff & 0xFFFFFF;
	} else if (tmpVal == 3) {
		frame = *(uint32_t*)buff;
	}

	return frame;
}

static void spi_bflb_copy_frame_to_buff(const struct device *dev,
					uint32_t *frame, void *buff)
{
	const struct spi_bflb_cfg *cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->base + SPI_CONFIG_OFFSET);
	tmpVal = (tmpVal & SPI_CR_SPI_FRAME_SIZE_MASK)
		>> SPI_CR_SPI_FRAME_SIZE_SHIFT;

	if(tmpVal == 0) {
		(*(uint8_t*)buff) = *frame & 0xFF;
	} else if (tmpVal == 1) {
		(*(uint16_t*)buff) = *frame & 0xFFFF;
	} else if (tmpVal == 2) {
		(*(uint16_t*)buff) = *frame & 0xFFFF;
		(*(uint8_t*)((uint8_t*)buff + 2)) = (*frame & 0xFF0000) >> 16;
	} else if (tmpVal == 3) {
		(*(uint32_t*)buff) = *frame;
	}
}

static int spi_bflb_transceive_sync_one(const struct device *dev,
					const struct spi_buf *tx_buf,
					const struct spi_buf *rx_buf)
{
	const struct spi_bflb_cfg *cfg = dev->config;
	uint32_t tmpVal = 0;
	uint32_t rx_x = 0;
	uint32_t tx_x = 0;
	uint8_t frame_size = 0;
	uint8_t rx_cnt = 0;
	uint8_t tx_cnt = 0;
	uint32_t zero = 0;

	tmpVal = sys_read32(cfg->base + SPI_CONFIG_OFFSET);
	/* in bytes */
	frame_size = ((tmpVal & SPI_CR_SPI_FRAME_SIZE_MASK)
		>> SPI_CR_SPI_FRAME_SIZE_SHIFT) + 1;

	while (rx_x < rx_buf->len || tx_x < tx_buf->len) {
		tmpVal = sys_read32(cfg->base + SPI_FIFO_CONFIG_1_OFFSET);
		rx_cnt = (tmpVal & SPI_RX_FIFO_CNT_MASK)
			>> SPI_RX_FIFO_CNT_SHIFT;
		tx_cnt = (tmpVal & SPI_TX_FIFO_CNT_MASK)
			>> SPI_TX_FIFO_CNT_SHIFT;

		if (rx_x < rx_buf->len && rx_cnt > 0) {
			tmpVal = sys_read32(cfg->base + SPI_FIFO_RDATA_OFFSET);
			if (rx_buf->buf != NULL) {
				spi_bflb_copy_frame_to_buff(dev, &tmpVal,
					(uint8_t*)rx_buf->buf + rx_x);
			}
			rx_x += frame_size;
		}

		if (tx_x < tx_buf->len && tx_cnt > 0) {
			if (tx_buf->buf != NULL) {
				sys_write32(spi_bflb_from_buff_to_frame(dev,
					(uint8_t*)tx_buf->buf + tx_x),
				cfg->base + SPI_FIFO_WDATA_OFFSET);
			} else {
				sys_write32(
				spi_bflb_from_buff_to_frame(dev, &zero),
				cfg->base + SPI_FIFO_WDATA_OFFSET);
			}
			tx_x += frame_size;
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
	uint32_t tmpVal = 0;
	const struct spi_bflb_cfg *cfg = dev->config;
	struct spi_bflb_data *data = dev->data;
	k_timepoint_t end_timeout =
		sys_timepoint_calc(K_MSEC(SPI_WAIT_TIMEOUT_MS));
	uint32_t rx_cnt = 0, tx_cnt = 0;
	struct spi_buf empty_buf = {0};
	struct spi_buf_set empty_buf_set = {0};

	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	rc = spi_bflb_configure(dev, config);
	if (rc != 0) {
		return rc;
	}

	if (rx_bufs == NULL) {
		rx_bufs = &empty_buf_set;
	}

	if (tx_bufs == NULL) {
		tx_bufs = &empty_buf_set;
	}

	spi_bflb_detrigger_master(dev);

	/*while (spi_bflb_bus_busy(dev) && !sys_timepoint_expired(end_timeout))
	{
	}

	if (sys_timepoint_expired(end_timeout)) {
		return -ETIMEDOUT;
	}*/

	/* clean up */
	tmpVal = sys_read32(cfg->base + SPI_FIFO_CONFIG_0_OFFSET);
	tmpVal |= SPI_TX_FIFO_CLR;
	tmpVal |= SPI_RX_FIFO_CLR;
	sys_write32(tmpVal, cfg->base + SPI_FIFO_CONFIG_0_OFFSET);

	spi_context_cs_control(&data->ctx, true);

	spi_bflb_trigger_master(dev);

	while (rx_cnt < rx_bufs->count || tx_cnt < tx_bufs->count) {
		if (rx_cnt < rx_bufs->count && tx_cnt < tx_bufs->count) {
			spi_bflb_transceive_sync_one(dev,
				&(tx_bufs->buffers[tx_cnt]),
				&(rx_bufs->buffers[rx_cnt]));
			tx_cnt++;
			rx_cnt++;
		} else if (rx_cnt < rx_bufs->count) {
			empty_buf.len = rx_bufs->buffers[rx_cnt].len;
			spi_bflb_transceive_sync_one(dev,
				&empty_buf,
				&(rx_bufs->buffers[rx_cnt]));
			rx_cnt++;
		} else if (tx_cnt < tx_bufs->count) {
			empty_buf.len = tx_bufs->buffers[tx_cnt].len;
			spi_bflb_transceive_sync_one(dev,
				&(tx_bufs->buffers[tx_cnt]),
				&empty_buf);
			tx_cnt++;
		}
	}
	do {
		tmpVal = sys_read32(cfg->base + SPI_FIFO_CONFIG_1_OFFSET);
		tx_cnt = (tmpVal & SPI_TX_FIFO_CNT_MASK)
			>> SPI_TX_FIFO_CNT_SHIFT;
	} while (tx_cnt < SPI_FIFO_SIZE);

	spi_context_cs_control(&data->ctx, false);
	spi_bflb_detrigger_master(dev);
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
		//spi_bflb_isr_END(dev);
		tmpVal |= SPI_CR_SPI_END_CLR;
	}
	if ((tmpVal & SPI_TXF_INT) != 0)
	{
		//spi_bflb_isr_TXF(dev);
	}
	if ((tmpVal & SPI_RXF_INT) != 0)
	{
		//spi_bflb_isr_RXF(dev);
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

#define SPI_BFLB_IRQ_HANDLER_DECL(n)					\
	static void spi_bflb_config_func_##n(const struct device *dev);
#define SPI_BFLB_IRQ_HANDLER_FUNC(n)					\
	.irq_config_func = spi_bflb_config_func_##n
#define SPI_BFLB_IRQ_HANDLER(n)						\
	static void spi_bflb_config_func_##n(const struct device *dev)	\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority),			\
			    spi_bflb_isr,				\
			    DEVICE_DT_INST_GET(n),			\
			    0);						\
		irq_enable(DT_INST_IRQN(n));				\
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
