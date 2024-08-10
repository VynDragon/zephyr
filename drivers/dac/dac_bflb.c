/*
 * Copyright (c) 2024 MASSDRIVER EI
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_dac

#include <errno.h>

#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>

#include <soc.h>
#include <bouffalolab/common/dac_reg.h>

#define LOG_LEVEL CONFIG_DAC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dac_bflb);

/* Read-only driver configuration */
struct dac_bflb_cfg {
	uint32_t reg_GPIP;
	uint32_t reg_GLB;
	const struct pinctrl_dev_config *pcfg;
};

static int dac_bflb_write_value(const struct device *dev,
					uint8_t channel, uint32_t value)
{
	const struct dac_bflb_cfg *cfg = dev->config;
	uint32_t tmpVal = 0;

	if (channel > 1) {
		LOG_ERR("Only 2 Channels");
		return -EINVAL;
	}

	tmpVal = sys_read32(cfg->reg_GLB + GLB_GPDAC_DATA_OFFSET);

	if (channel == 0) {
		tmpVal &= ~GLB_GPDAC_A_DATA_MASK;
		tmpVal |= (value << GLB_GPDAC_A_DATA_SHIFT);
	} else if (channel == 1) {
		tmpVal &= ~GLB_GPDAC_B_DATA_MASK;
		tmpVal |= (value << GLB_GPDAC_B_DATA_SHIFT);
	}
	sys_write32(tmpVal, cfg->reg_GLB + GLB_GPDAC_DATA_OFFSET);
	return 0;
}

static int dac_bflb_channel_setup(const struct device *dev,
				   const struct dac_channel_cfg *channel_cfg)
{
	const struct dac_bflb_cfg *cfg = dev->config;
	uint32_t tmpVal = 0;

	if (channel_cfg->resolution != 10) {
		LOG_ERR("Resolution is 10-bits only");
		return -EINVAL;
	}

	if (channel_cfg->channel_id > 1) {
		LOG_ERR("Only 2 Channels");
		return -EINVAL;
	}

	if (channel_cfg->channel_id == 0) {
		tmpVal = sys_read32(cfg->reg_GLB + GLB_GPDAC_ACTRL_OFFSET);
		tmpVal |= (GLB_GPDAC_A_EN | GLB_GPDAC_IOA_EN);
		sys_write32(tmpVal, cfg->reg_GLB + GLB_GPDAC_ACTRL_OFFSET);
	} else if (channel_cfg->channel_id == 1) {
		tmpVal = sys_read32(cfg->reg_GLB + GLB_GPDAC_BCTRL_OFFSET);
		tmpVal |= (GLB_GPDAC_B_EN | GLB_GPDAC_IOB_EN);
		sys_write32(tmpVal, cfg->reg_GLB + GLB_GPDAC_BCTRL_OFFSET);
	}


	return 0;
}


static void dac_bflb_init_clock(const struct device *dev)
{
	const struct dac_bflb_cfg *cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->reg_GLB + GLB_DIG32K_WAKEUP_CTRL_OFFSET);
	tmpVal &= ~GLB_DIG_512K_EN_MSK;
	sys_write32(tmpVal, cfg->reg_GLB + GLB_DIG32K_WAKEUP_CTRL_OFFSET);

	tmpVal &= ~GLB_DIG_512K_COMP_MSK;

	/* clock source is XCLK (crystal or RC32M) */
	tmpVal &= ~GLB_DIG_CLK_SRC_SEL_MSK;
	tmpVal |= 1 << GLB_DIG_CLK_SRC_SEL_POS;

	/* set divider with SDK value, this may be incorrect with a 40M crystal and give
	 * 645K instead of 512K, it close to fine for 32M input.
	 */
	tmpVal &= ~GLB_DIG_512K_DIV_MSK;
	tmpVal |= 0x3E << GLB_DIG_512K_DIV_POS;

	tmpVal |= GLB_DIG_512K_EN_MSK;
	sys_write32(tmpVal, cfg->reg_GLB + GLB_DIG32K_WAKEUP_CTRL_OFFSET);
}

static int dac_bflb_init(const struct device *dev)
{
	const struct dac_bflb_cfg *cfg = dev->config;
	int err;
	uint32_t tmpVal = 0;

	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		LOG_ERR("DAC pinctrl setup failed (%d)", err);
		return err;
	}

	dac_bflb_init_clock(dev);

	/* reset procedure */
	tmpVal = sys_read32(cfg->reg_GLB + GLB_GPDAC_CTRL_OFFSET);
	tmpVal &= ~GLB_GPDACA_RSTN_ANA_MSK;
	tmpVal &= ~GLB_GPDACB_RSTN_ANA_MSK;
	sys_write32(tmpVal, cfg->reg_GLB + GLB_GPDAC_CTRL_OFFSET);

	__asm__ volatile (".rept 15 ; nop ; .endr");

	tmpVal = sys_read32(cfg->reg_GLB + GLB_GPDAC_CTRL_OFFSET);
	tmpVal |= GLB_GPDACA_RSTN_ANA_MSK;
	tmpVal |= GLB_GPDACB_RSTN_ANA_MSK;
	sys_write32(tmpVal, cfg->reg_GLB + GLB_GPDAC_CTRL_OFFSET);

	__asm__ volatile (".rept 15 ; nop ; .endr");

	/* internal reference */
	tmpVal = sys_read32(cfg->reg_GLB + GLB_GPDAC_CTRL_OFFSET);
	tmpVal &= ~GLB_GPDAC_REF_SEL_MSK;
	sys_write32(tmpVal, cfg->reg_GLB + GLB_GPDAC_CTRL_OFFSET);

	tmpVal = sys_read32(cfg->reg_GPIP + GPIP_GPDAC_CONFIG_OFFSET);
	tmpVal |= GPIP_GPDAC_EN;
#if defined(CONFIG_SOC_SERIES_BL60X) || defined(CONFIG_SOC_SERIES_BL70X)
	tmpVal |= GPIP_GPDAC_EN2;
#endif
	/* mode = clock = 32khz */
	tmpVal &= ~GPIP_GPDAC_MODE_MASK;
	/* sel reg */
	tmpVal &= ~GPIP_GPDAC_CH_A_SEL_MASK;
	tmpVal &= ~GPIP_GPDAC_CH_B_SEL_MASK;
	/* 'DSM mode' to 'bypass' */
	tmpVal &= ~GPIP_DSM_MODE_MASK;
	sys_write32(tmpVal, cfg->reg_GPIP + GPIP_GPDAC_CONFIG_OFFSET);

	/* no DMA */
	tmpVal = sys_read32(cfg->reg_GPIP + GPIP_GPDAC_DMA_CONFIG_OFFSET);
	tmpVal &= ~GPIP_GPDAC_DMA_TX_EN;
	sys_write32(tmpVal, cfg->reg_GPIP + GPIP_GPDAC_DMA_CONFIG_OFFSET);


	return 0;
}

static const struct dac_driver_api dac_bflb_api = {
	.channel_setup = dac_bflb_channel_setup,
	.write_value = dac_bflb_write_value
};


#define DAC_BFLB_DEVICE(n)						\
	PINCTRL_DT_INST_DEFINE(n);					\
	static const struct dac_bflb_cfg dac_bflb_config_##n = {	\
		.reg_GPIP = DT_INST_REG_ADDR_BY_IDX(n, 0),		\
		.reg_GLB = DT_INST_REG_ADDR_BY_IDX(n, 1),		\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
	};								\
	DEVICE_DT_INST_DEFINE(n, dac_bflb_init, NULL,			\
			      NULL,					\
			      &dac_bflb_config_##n, POST_KERNEL,	\
			      CONFIG_DAC_INIT_PRIORITY,			\
			      &dac_bflb_api);

DT_INST_FOREACH_STATUS_OKAY(DAC_BFLB_DEVICE)
