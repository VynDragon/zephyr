/*
 * Copyright (c) 2024 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_i2c

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>
#include <zephyr/sys/sys_io.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_bflb);

/* Register Offsets */
#include <hardware/glb_reg.h>
#include <common_hardware/i2c_reg.h>

#include "i2c-priv.h"

/* defines */

#define I2C_WAIT_TIMEOUT_MS 150
#define I2C_MAX_PACKET_LENGTH 256

/* Structure declarations */

struct i2c_bflb_cfg {
	const struct pinctrl_dev_config *pincfg;
	uint32_t base;
	uint32_t dts_freq;
};

struct i2c_bflb_data {
	bool is_10_bits_address;
};

/* a buffer for when when dont get whole 32-bit words */

static uint8_t i2c_bflb_temp_buf[4] = {0};
static uint32_t i2c_bflb_temp_buf_cnt = 0;

/* Support Functions */

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


static uint32_t i2c_bflb_get_bclk_clk(void)
{
	uint32_t tmpVal = 0;
	uint32_t i2c_divider = 0;
	uint32_t hclk_divider = 0;

	/* FCLK -> BCLK */
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	hclk_divider = (tmpVal & GLB_REG_HCLK_DIV_MSK) >> GLB_REG_HCLK_DIV_POS;

	/* bclk -> i2cclk */
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG3_OFFSET);
	i2c_divider = (tmpVal & GLB_I2C_CLK_DIV_MSK) >> GLB_I2C_CLK_DIV_POS;

	/* what is fclk */
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_HBN_ROOT_CLK_SEL_MSK) >> GLB_HBN_ROOT_CLK_SEL_POS;

	if (tmpVal == 0)
	{
		/* RC32M clock */
		tmpVal = (32 * 1000 * 1000) / (hclk_divider + 1);
		return (tmpVal / (i2c_divider + 1));
	}
	else if (tmpVal == 1)
	{
		/* Crystal clock */
		tmpVal = uart_bflb_get_crystal_frequency() / (hclk_divider + 1);
		return (tmpVal / (i2c_divider + 1));
	}
	else if (tmpVal > 1)
	{
		/* PLL Clock */
		tmpVal = uart_bflb_get_PLL_frequency() / (hclk_divider + 1);
		return (tmpVal / (i2c_divider + 1));

	}
	return 0;
}

static int32_t i2c_bflb_clamp_phase(int32_t phase)
{
	if (phase < 1)
	{
		return 1;
	}
	if (phase > 256)
	{
		return 256;
	}
	return phase;
}

/*"The i2c module divides the data transmission
 * into 4 phases. Each phase is controlled by a single byte in the register. The number of samples
 * in each phase can be set. "
 */
static void i2c_bflb_set_start_stop(const struct device *dev, uint32_t frequency)
{
	const struct i2c_bflb_cfg *config = dev->config;
	int32_t phase = 0;
	int32_t phase0, phase1, phase2, phase3;
	int32_t bias = 0;
	uint32_t tmpVal = 0;

	phase = (i2c_bflb_get_bclk_clk() + frequency / 2) / frequency;

	if (frequency <= 100 * 1000) {
		/* when SCL clock <= 100KHz, duty cycle is default 50%  */
		phase0 = (phase + 2) / 4;
		phase1 = phase0;
		phase2 = phase / 2 - phase0;
		phase3 = phase - phase0 - phase1 - phase2;
	} else {
		/* when SCL clock > 100KHz, duty cycle isdefault 33% */
		phase0 = (phase + 2) / 3;
		phase1 = (phase + 3) / 6;
		phase2 = (phase + 1) / 3 - phase1;
		phase3 = phase - phase0 - phase1 - phase2;
	}

	/* calculate rectify phase when de-glitch or clock-stretching is enabled */
	tmpVal = sys_read32(config->base + I2C_CONFIG_OFFSET);
	if ((tmpVal & I2C_CR_I2C_DEG_EN) && (tmpVal & I2C_CR_I2C_SCL_SYNC_EN)) {
		bias = (tmpVal & I2C_CR_I2C_DEG_CNT_MASK) >> I2C_CR_I2C_DEG_CNT_SHIFT;
		bias += 1;
	} else {
		bias = 0;
	}
	if (tmpVal & I2C_CR_I2C_SCL_SYNC_EN) {
		bias += 3;
	}

	phase0 = i2c_bflb_clamp_phase(phase0);
	phase1 = i2c_bflb_clamp_phase(phase1);
	phase2 = i2c_bflb_clamp_phase(phase2);
	phase3 = i2c_bflb_clamp_phase(phase3);

	/* calculate data phase */
	tmpVal = (phase0 - 1) << I2C_CR_I2C_PRD_D_PH_0_SHIFT;
	tmpVal |= (((phase1 - bias - 1) <= 0) ? 1 : (phase1 - bias - 1)) <<
I2C_CR_I2C_PRD_D_PH_1_SHIFT;	/* data phase1 must not be 0 */
	tmpVal |= (phase2 - 1) << I2C_CR_I2C_PRD_D_PH_2_SHIFT;
	tmpVal |= (phase3 - 1) << I2C_CR_I2C_PRD_D_PH_3_SHIFT;
	sys_write32(tmpVal, config->base + I2C_PRD_DATA_OFFSET);
	/* calculate start phase */
	tmpVal = (phase0 - 1) << I2C_CR_I2C_PRD_S_PH_0_SHIFT;
	tmpVal |= (((phase0 + phase3 - 1) >= 256) ? 255 : (phase0 + phase3 - 1)) <<
	I2C_CR_I2C_PRD_S_PH_1_SHIFT;
	tmpVal |= (((phase1 + phase2 - 1) >= 256) ? 255 : (phase1 + phase2 - 1)) <<
	I2C_CR_I2C_PRD_S_PH_2_SHIFT;
	tmpVal |= (phase3 - 1) << I2C_CR_I2C_PRD_S_PH_3_SHIFT;
	sys_write32(tmpVal, config->base + I2C_PRD_START_OFFSET);
	/* calculate stop phase */
	tmpVal = (phase0 - 1) << I2C_CR_I2C_PRD_P_PH_0_SHIFT;
	tmpVal |= (((phase1 + phase2 - 1) >= 256) ? 255 : (phase1 + phase2 - 1)) <<
	I2C_CR_I2C_PRD_P_PH_1_SHIFT;
	tmpVal |= (phase0 - 1) << I2C_CR_I2C_PRD_P_PH_2_SHIFT;
	tmpVal |= (phase3 - 1) << I2C_CR_I2C_PRD_P_PH_3_SHIFT;
	sys_write32(tmpVal, config->base + I2C_PRD_STOP_OFFSET);
}

static void i2c_bflb_clear_fifo(const struct device *dev)
{
	uint32_t tmpVal = 0;
	const struct i2c_bflb_cfg *config = dev->config;


	tmpVal = sys_read32(config->base + I2C_FIFO_CONFIG_0_OFFSET);
	tmpVal |= I2C_TX_FIFO_CLR;
	tmpVal |= I2C_RX_FIFO_CLR;
	sys_write32(tmpVal, config->base + I2C_FIFO_CONFIG_0_OFFSET);
}

static void i2c_bflb_clear_int(const struct device *dev)
{
	uint32_t tmpVal = 0;
	const struct i2c_bflb_cfg *config = dev->config;


	tmpVal = sys_read32(config->base + I2C_INT_STS_OFFSET);
	tmpVal |= I2C_CR_I2C_END_CLR;
	tmpVal |= I2C_CR_I2C_NAK_CLR;
	tmpVal |= I2C_CR_I2C_ARB_CLR;
	sys_write32(tmpVal, config->base + I2C_INT_STS_OFFSET);
}


static void i2c_bflb_clock_enabled(const struct device *dev, bool enabled)
{
	uint32_t tmpVal = 0;
	const struct i2c_bflb_cfg *config = dev->config;

	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG3_OFFSET);
	if (enabled)
	{
		tmpVal |= (1 << GLB_I2C_CLK_EN_POS);
	}
	else
	{
		tmpVal &= ~(1 << GLB_I2C_CLK_EN_POS);
	}
	sys_write32(tmpVal, GLB_BASE + GLB_CLK_CFG3_OFFSET);
}

static void i2c_bflb_trigger(const struct device *dev)
{
	uint32_t tmpVal = 0;
	const struct i2c_bflb_cfg *config = dev->config;

	tmpVal = sys_read32(config->base + I2C_CONFIG_OFFSET);
	tmpVal |= I2C_CR_I2C_M_EN;
	sys_write32(tmpVal, config->base + I2C_CONFIG_OFFSET);
}

static void i2c_bflb_enabled(const struct device *dev, bool enabled)
{
	uint32_t tmpVal = 0;
	const struct i2c_bflb_cfg *config = dev->config;

	tmpVal = sys_read32(config->base + I2C_CONFIG_OFFSET);
	if (enabled)
	{
		/* this is really more of a transaction trigger */
		tmpVal |= I2C_CR_I2C_M_EN;
	}
	else
	{
		tmpVal &= ~I2C_CR_I2C_M_EN;
		i2c_bflb_clear_fifo(dev);
		i2c_bflb_clear_int(dev);
	}
	sys_write32(tmpVal, config->base + I2C_CONFIG_OFFSET);
}

/* API Functions */

static int i2c_bflb_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_bflb_cfg *config = dev->config;
	struct i2c_bflb_data *data = dev->data;
	uint32_t speed_nb = I2C_SPEED_GET(dev_config);
	uint32_t speed_freq = 0;
	uint32_t mode = (dev_config & I2C_MODE_CONTROLLER) >> 4;
	uint32_t tenbit_addr = dev_config & I2C_ADDR_10_BITS;
	uint32_t tmpVal = 0;

	switch (I2C_SPEED_GET(dev_config)) {
		case I2C_SPEED_STANDARD:
			speed_freq = 100000U; /* 100 KHz */
			break;
		case I2C_SPEED_FAST:
			speed_freq = 400000U; /* 400 KHz */
			break;
		case I2C_SPEED_FAST_PLUS:
			speed_freq = 1000000U; /* 1 MHz */
			break;
		case I2C_SPEED_HIGH:
			speed_freq = 3400000U; /* 3.4 MHz */
			break;
		case I2C_SPEED_ULTRA:
			speed_freq = 5000000U; /* 5 MHz */
			break;
		case I2C_SPEED_DT:
			speed_freq = config->dts_freq;
			break;
		default:
			LOG_ERR("Unsupported I2C speed requested");
			return -ENOTSUP;
	}

	i2c_bflb_enabled(dev, false);
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG3_OFFSET);
	/* set div to 0 */
	tmpVal = tmpVal & GLB_I2C_CLK_DIV_UMSK;
	sys_write32(tmpVal, GLB_BASE + GLB_CLK_CFG3_OFFSET);


	/* mask all interrupts */
	tmpVal = sys_read32(config->base + I2C_INT_STS_OFFSET);

	tmpVal |= (I2C_CR_I2C_END_MASK |
		I2C_CR_I2C_TXF_MASK |
		I2C_CR_I2C_RXF_MASK |
		I2C_CR_I2C_NAK_MASK |
		I2C_CR_I2C_ARB_MASK |
		I2C_CR_I2C_FER_MASK);

	sys_write32(tmpVal, config->base + I2C_INT_STS_OFFSET);

	i2c_bflb_set_start_stop(dev, speed_freq);
#if !(defined(CONFIG_SOC_SERIES_BL6) || defined(CONFIG_SOC_SERIES_BL7))
	data->is_10_bits_address = tenbit_addr > 0 ? true: false;
#else
	if (tenbit_addr > 0)
	{
		LOG_ERR("10 bits addresses unsupported");
		return -EIO;
	}
	data->is_10_bits_address = false;
#endif
	if (!(mode > 0))
	{
		LOG_ERR("Not Controller Mode unsupported");
		return -EIO;
	}
//i2c_bflb_enabled(dev, true);
	return 0;
}

static void i2c_bflb_set_address(const struct device *dev, uint32_t address)
{
	uint32_t tmpVal = 0;
	const struct i2c_bflb_cfg *config = dev->config;
	struct i2c_bflb_data *data = dev->data;

	tmpVal = sys_read32(config->base + I2C_CONFIG_OFFSET);
	/* no sub addresses */
	tmpVal &= ~I2C_CR_I2C_SUB_ADDR_EN;
	tmpVal &= ~I2C_CR_I2C_SLV_ADDR_MASK;
#if !(defined(CONFIG_SOC_SERIES_BL6) || defined(CONFIG_SOC_SERIES_BL7))
	if (data->is_10_bits_address)
	{
		tmpVal |= I2C_CR_I2C_10B_ADDR_EN;
		tmpVal |= ((address & 0x3FF) << I2C_CR_I2C_SLV_ADDR_SHIFT);
	}
	else
	{
		tmpVal |= ((address & 0x7F) << I2C_CR_I2C_SLV_ADDR_SHIFT);
	}
#else
	tmpVal |= ((address & 0x7F) << I2C_CR_I2C_SLV_ADDR_SHIFT);
#endif
	sys_write32(tmpVal, config->base + I2C_CONFIG_OFFSET);
}

static int i2c_bflb_busy(const struct device *dev)
{
	const struct i2c_bflb_cfg *config = dev->config;
	uint32_t tmpVal = 0;
	int rc = 0;

	tmpVal = sys_read32(config->base + I2C_BUS_BUSY_OFFSET);
	if (tmpVal & I2C_STS_I2C_BUS_BUSY)
	{
		rc += 1;
	}
	return rc;
}

static int i2c_bflb_txready(const struct device *dev)
{
	const struct i2c_bflb_cfg *config = dev->config;
	uint32_t tmpVal = 0;
	int rc = 1;

	tmpVal = sys_read32(config->base + I2C_INT_STS_OFFSET);
	if (tmpVal & I2C_TXF_INT)
	{
		rc -= 1;
	}
	return rc;
}

static int i2c_bflb_rxready(const struct device *dev)
{
	const struct i2c_bflb_cfg *config = dev->config;
	uint32_t tmpVal = 0;
	int rc = 0;

	tmpVal = sys_read32(config->base + I2C_INT_STS_OFFSET);
	if (tmpVal & I2C_RXF_INT > 0)
	{
		rc += 1;
	}
	return rc;
}

static int i2c_bflb_read_bits(const struct device *dev, uint8_t *buf, uint8_t num)
{
	const struct i2c_bflb_cfg *config = dev->config;
	uint32_t timeout = 0;
	uint32_t tmpVal = 0;

	timeout = 0;
	while (i2c_bflb_rxready(dev) == 0 && timeout < I2C_WAIT_TIMEOUT_MS) {
		k_sleep(K_MSEC(1));
		timeout++;
	}
	if (timeout >= I2C_WAIT_TIMEOUT_MS)
	{
		return -ETIMEDOUT;
	}

	tmpVal = sys_read32(config->base + I2C_FIFO_RDATA_OFFSET);

	for (uint8_t i = 0; i < num; i++) {
		buf[i] |= (tmpVal >> ((i % 4) * 8)) & 0xFF;
	}
	return 0;
}

static int i2c_bflb_read_msgs(const struct device *dev,
				struct i2c_msg *msgs,
				uint8_t num_msgs,
				uint16_t addr)
{
	const struct i2c_bflb_cfg *config = dev->config;
	uint32_t tmpVal = 0;
	uint32_t curByte = 0;
	uint32_t timeout = 0;
	uint32_t total_len = 0;
	uint32_t i = 0;
	uint32_t z = 0;
	uint32_t buffered = 0;

	for(i = 0; i < num_msgs; i++)
	{
		total_len = total_len + msgs[i].len;
	}

	if (total_len > I2C_MAX_PACKET_LENGTH) {
		return -EINVAL;
	}


	/* set message length */
	tmpVal = sys_read32(config->base + I2C_CONFIG_OFFSET);
	tmpVal &= ~I2C_CR_I2C_PKT_LEN_MASK;
	tmpVal |= ((total_len - 1) << I2C_CR_I2C_PKT_LEN_SHIFT) & I2C_CR_I2C_PKT_LEN_MASK;
	sys_write32(tmpVal, config->base + I2C_CONFIG_OFFSET);

	/* set read direction */
	tmpVal = sys_read32(config->base + I2C_CONFIG_OFFSET);
	tmpVal |= I2C_CR_I2C_PKT_DIR;
	sys_write32(tmpVal, config->base + I2C_CONFIG_OFFSET);

	i2c_bflb_trigger(dev);
	//i2c_bflb_enabled(dev, true);
	for(i = 0; i < num_msgs; i++)
	{
		
		for(z = 0; z < msgs[i].len; z++)
		{
			if (z % 4 == 3)
			{
				i2c_bflb_read_bits(dev, &(msgs[i].buf[z - 3]), 4);
			}
		}
		if (z % 4 > 0)
		{
			i2c_bflb_read_bits(dev, &(msgs[i].buf[z - z % 4 + 1]), z % 4);
		}
	}
	//i2c_bflb_enabled(dev, false);

	return 0;
}


static uint32_t i2c_bflb_txfifo_available(const struct device *dev)
{
	const struct i2c_bflb_cfg *config = dev->config;
	return ((sys_read32(config->base + I2C_FIFO_CONFIG_1_OFFSET) & I2C_TX_FIFO_CNT_MASK) >>
I2C_TX_FIFO_CNT_SHIFT);

}

static uint32_t i2c_bflb_endint(const struct device *dev)
{
	const struct i2c_bflb_cfg *config = dev->config;
	return (sys_read32(config->base + I2C_INT_STS_OFFSET) & I2C_END_INT);

}

static void i2c_bflb_endint_clear(const struct device *dev)
{
	const struct i2c_bflb_cfg *config = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(config->base + I2C_INT_STS_OFFSET);
	tmpVal &= ~I2C_CR_I2C_END_CLR;
	sys_write32(tmpVal, config->base + I2C_INT_STS_OFFSET);
}


static int i2c_bflb_write_bits(const struct device *dev, uint8_t *buf, uint8_t num)
{
	const struct i2c_bflb_cfg *config = dev->config;
	uint32_t timeout = 0;
	uint32_t tmpVal = 0;

	/* Wait for a previous transfer */
	if (i2c_bflb_txfifo_available(dev) < 1)
	{
		timeout = 0;
		while (i2c_bflb_endint(dev) == 0 && timeout < I2C_WAIT_TIMEOUT_MS)
		{
			k_sleep(K_MSEC(1));
			timeout++;
		}
		if (timeout >= I2C_WAIT_TIMEOUT_MS)
		{
			return -ETIMEDOUT;
		}

	}
	//i2c_bflb_clear_fifo(dev);
	for (uint8_t i = 0; i < num; i++) {
		tmpVal |= (buf[i] << ((i % 4) * 8));
	}

	i2c_bflb_endint_clear(dev);
	sys_write32(tmpVal, config->base + I2C_FIFO_WDATA_OFFSET);
	i2c_bflb_trigger(dev);
	return 0;
}

static int i2c_bflb_write_msgs(const struct device *dev,
				struct i2c_msg *msgs,
				uint8_t num_msgs,
				uint16_t addr)
{
	const struct i2c_bflb_cfg *config = dev->config;
	uint32_t tmpVal = 0;
	uint32_t curByte = 0;
	uint32_t timeout = 0;
	uint32_t total_len = 0;
	uint32_t i = 0;
	uint32_t z = 0;

	for(i = 0; i < num_msgs; i++)
	{
		total_len = total_len + msgs[i].len;
	}

	if (total_len > I2C_MAX_PACKET_LENGTH) {
		return -EINVAL;
	}

	/* set message length */
	tmpVal = sys_read32(config->base + I2C_CONFIG_OFFSET);
	tmpVal &= ~I2C_CR_I2C_PKT_LEN_MASK;
	tmpVal |= ((total_len - 1) << I2C_CR_I2C_PKT_LEN_SHIFT) & I2C_CR_I2C_PKT_LEN_MASK;
	sys_write32(tmpVal, config->base + I2C_CONFIG_OFFSET);

	/* set write direction */
	tmpVal = sys_read32(config->base + I2C_CONFIG_OFFSET);
	tmpVal &= ~I2C_CR_I2C_PKT_DIR;
	sys_write32(tmpVal, config->base + I2C_CONFIG_OFFSET);

	for(i = 0; i < num_msgs; i++)
	{
		for(z = 0; z < msgs[i].len; z++)
		{
			i2c_bflb_temp_buf[i2c_bflb_temp_buf_cnt % 4] = msgs[i].buf[z];
			i2c_bflb_temp_buf_cnt++;
			if (i2c_bflb_temp_buf_cnt % 4 == 0 && i2c_bflb_temp_buf_cnt > 1)
			{
				i2c_bflb_write_bits(dev, i2c_bflb_temp_buf, 4);
			}
		}
	}
	i2c_bflb_temp_buf_cnt = i2c_bflb_temp_buf_cnt % 4;

	/*if (i2c_bflb_txfifo_available(dev) < 2)
	{
		timeout = 0;
		while (i2c_bflb_endint(dev) == 0 && timeout < I2C_WAIT_TIMEOUT_MS)
		{
			k_sleep(K_MSEC(1));
			timeout++;
		}
		if (timeout >= I2C_WAIT_TIMEOUT_MS)
		{
			return -ETIMEDOUT;
		}
	}*/



	return 0;
}

static int i2c_bflb_transfer_finish(const struct device *dev)
{
	uint32_t timeout = 0;

	if (i2c_bflb_temp_buf_cnt != 0)
	{
		i2c_bflb_write_bits(dev, i2c_bflb_temp_buf, i2c_bflb_temp_buf_cnt + 1);
		i2c_bflb_temp_buf_cnt = 0;
	}
	if (i2c_bflb_txfifo_available(dev) < 2)
	{
		timeout = 0;
		while (i2c_bflb_endint(dev) == 0 && timeout < I2C_WAIT_TIMEOUT_MS)
		{
			k_sleep(K_MSEC(1));
			timeout++;
		}
		if (timeout >= I2C_WAIT_TIMEOUT_MS)
		{
			return -ETIMEDOUT;
		}
	}
	i2c_bflb_enabled(dev, false);
}
static int i2c_bflb_transfer(const struct device *dev,
			       struct i2c_msg *msgs,
			       uint8_t num_msgs,
			       uint16_t addr)
{
	int rc = 0;
	uint8_t pass_num = 0;

	/* Check for NULL pointers */
	if (dev == NULL) {
		LOG_ERR("Device handle is NULL");
		return -EINVAL;
	}
	if (dev->config == NULL) {
		LOG_ERR("Device config is NULL");
		return -EINVAL;
	}
	if (dev->data == NULL) {
		LOG_ERR("Device data is NULL");
		return -EINVAL;
	}
	if (msgs == NULL) {
		return -EINVAL;
	}

	//i2c_bflb_enabled(dev, false);
	i2c_bflb_set_address(dev, addr);

	 for (uint8_t i = 0; i < num_msgs; i++) {
		if ((msgs[i].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
			pass_num = 0;
			while ((msgs[i + pass_num].flags & I2C_MSG_RW_MASK) == (msgs[i].flags &
I2C_MSG_RW_MASK) && (msgs[i + pass_num].flags & I2C_MSG_STOP) == 0 && pass_num + i < num_msgs)
			{
				pass_num++;
			}
			rc = i2c_bflb_read_msgs(dev, &(msgs[i]), pass_num + 1, addr);
			i = i + pass_num;
		} else {
			pass_num = 0;
			while ((msgs[i + pass_num].flags & I2C_MSG_RW_MASK) == (msgs[i].flags &
I2C_MSG_RW_MASK) && (msgs[i + pass_num].flags & I2C_MSG_STOP) == 0 && pass_num + i < num_msgs)
			{
				pass_num++;
			}
			rc = i2c_bflb_write_msgs(dev, &(msgs[i]), pass_num + 1, addr);
			i = i + pass_num;
			if ((msgs[i].flags & I2C_MSG_STOP) != 0)
			{
				i2c_bflb_transfer_finish(dev);
			}
		}

		if (rc != 0) {
			LOG_ERR("I2C failed to transfer messages\n");
			return rc;
		}
	 }

	return 0;
}

static int i2c_bflb_init(const struct device *dev)
{
	const struct i2c_bflb_cfg *config = dev->config;
	int rc = 0;
	uint32_t dev_config = 0;

	pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	i2c_bflb_clock_enabled(dev, true);
	dev_config = (I2C_MODE_CONTROLLER | i2c_map_dt_bitrate(config->dts_freq));
	rc = i2c_bflb_configure(dev, dev_config);
	if (rc != 0) {
		LOG_ERR("Failed to configure I2C on init");
		return rc;
	}

	return 0;
}


static const struct i2c_driver_api i2c_bflb_api = {
	.configure = i2c_bflb_configure,
	.transfer = i2c_bflb_transfer,
};

/* Device instantiation */

#define I2C_BFLB_INIT(n) \
	PINCTRL_DT_INST_DEFINE(n);					\
	static struct i2c_bflb_data i2c##n##_bflb_data;			\
	static const struct i2c_bflb_cfg i2c_bflb_cfg_##n = {		\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
		.base = DT_INST_REG_ADDR(n), 				\
		.dts_freq = DT_INST_PROP(n, clock_frequency),		\
	}; 								\
	I2C_DEVICE_DT_INST_DEFINE(n,					\
			    i2c_bflb_init,				\
			    NULL,					\
			    &i2c##n##_bflb_data,			\
			    &i2c_bflb_cfg_##n,				\
			    POST_KERNEL,				\
			    CONFIG_I2C_INIT_PRIORITY,			\
			    &i2c_bflb_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_BFLB_INIT)
