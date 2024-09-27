/*
 * Copyright (c) 2024 MASSDRIVER EI
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_adc

#include <soc.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/eeprom.h>

#include <bouffalolab/common/adc_reg.h>
#include <soc.h>


#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(adc_bflb, CONFIG_ADC_LOG_LEVEL);

struct adc_bflb_config {
	uint32_t reg_GPIP;
	uint32_t reg_AON;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(const struct device *dev);
};

struct adc_bflb_data {
	uint8_t channel_count;
	uint8_t channel_p[12];
	uint8_t channel_n[12];
	enum adc_gain gain;
};

static int adc_bflb_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg);
static int adc_bflb_read(const struct device *dev,
			const struct adc_sequence *sequence);

static struct adc_driver_api adc_bflb_api = {
	.channel_setup = adc_bflb_channel_setup,
	.read = adc_bflb_read,
	.ref_internal = 3300,
};


static void adc_bflb_channel_set_channel(const struct device *dev, uint8_t id,
uint8_t channel_number_n, uint8_t channel_number_p)
{
	const struct adc_bflb_config *const cfg = dev->config;
	uint32_t offset_p = AON_GPADC_REG_SCN_POS1_OFFSET;
	uint32_t offset_n = AON_GPADC_REG_SCN_NEG1_OFFSET;
	uint32_t tmpVal = 0;

	if (id > 5) {
		offset_p = AON_GPADC_REG_SCN_POS2_OFFSET;
		offset_n = AON_GPADC_REG_SCN_NEG2_OFFSET;
	}

	tmpVal = sys_read32(cfg->reg_AON + offset_p);
	tmpVal &= ~(0x1F << ((id % 6) * 5));
	tmpVal |= channel_number_p << ((id % 6) * 5);
	sys_write32(tmpVal, cfg->reg_AON + offset_p);

	tmpVal = sys_read32(cfg->reg_AON + offset_n);
	tmpVal &= ~(0x1F << ((id % 6) * 5));
	tmpVal |= channel_number_n << ((id % 6) * 5);
	sys_write32(tmpVal, cfg->reg_AON + offset_n);
}

static int adc_bflb_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	const struct adc_bflb_config *const cfg = dev->config;
	struct adc_bflb_data *data = dev->data;
	uint32_t tmpVal = 0;
	uint8_t channel_id = channel_cfg->channel_id;
	uint8_t gain = 1;

	if (data->channel_count > 12) {
		LOG_ERR("Too many channels");
		return -ENOTSUP;
	}
	if (channel_cfg->input_negative > 0x1F || channel_cfg->input_positive > 0x1F) {
		LOG_ERR("Bad channel number(s)");
		return -EINVAL;
	}

	if (channel_id > 11) {
		LOG_ERR("Bad channel ID");
		return -EINVAL;
	}

	switch (channel_cfg->gain) {
	case ADC_GAIN_1:
		gain = 1;
		break;
	case ADC_GAIN_2:
		gain = 2;
		break;
	case ADC_GAIN_4:
		gain = 3;
		break;
	case ADC_GAIN_8:
		gain = 4;
		break;
	case ADC_GAIN_16:
		gain = 5;
		break;
	case ADC_GAIN_32:
		gain = 6;
		break;
	default:
		LOG_ERR("Gain must be between 1 and 32 includeds, cannot be 3, 6, 12, 24");
		return -EINVAL;
	}

	if (data->gain != ADC_GAIN_128 && data->gain != channel_cfg->gain) {
		LOG_WRN("Gain does not match previously set gain, gain is global for this adc");
	}

	data->gain = channel_cfg->gain;

	if (channel_cfg->differential) {
		data->channel_n[channel_id] = channel_cfg->input_negative;
	} else {
		data->channel_n[channel_id] = 23;
	}
	data->channel_p[channel_id] = channel_cfg->input_positive;

	if (data->channel_count == 0) {
		tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_CONFIG1_OFFSET);
		tmpVal |= AON_GPADC_CONT_CONV_EN;
		tmpVal &= ~AON_GPADC_SCAN_EN;
		tmpVal &= ~AON_GPADC_CLK_ANA_INV;
		sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CONFIG1_OFFSET);

		tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);
		tmpVal &= ~AON_GPADC_POS_SEL_MASK;
		tmpVal &= ~AON_GPADC_NEG_SEL_MASK;

		if (channel_cfg->differential) {
			tmpVal &= ~AON_GPADC_NEG_GND;
			tmpVal |= ((channel_cfg->input_negative & 0x1F) << AON_GPADC_NEG_SEL_SHIFT);
		} else {
			tmpVal |= AON_GPADC_NEG_GND;
			/* GND channel */
			tmpVal |= 23 << AON_GPADC_NEG_SEL_SHIFT;
		}
		tmpVal |= ((channel_cfg->input_positive & 0x1F) << AON_GPADC_POS_SEL_SHIFT);
		sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);
		adc_bflb_channel_set_channel(dev, 0,
data->channel_n[channel_id], data->channel_p[channel_id]);
	} else {
		tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_CONFIG1_OFFSET);
		tmpVal &= ~AON_GPADC_CONT_CONV_EN;
		tmpVal |= AON_GPADC_SCAN_EN;
		tmpVal |= AON_GPADC_CLK_ANA_INV;
		tmpVal &= ~AON_GPADC_SCAN_LENGTH_MASK;
		tmpVal |= (data->channel_count & 0xF) << AON_GPADC_SCAN_LENGTH_SHIFT;
		sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CONFIG1_OFFSET);

		tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);
		tmpVal &= ~AON_GPADC_POS_SEL_MASK;
		tmpVal &= ~AON_GPADC_NEG_SEL_MASK;
		tmpVal |= AON_GPADC_NEG_GND;
		sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);

		adc_bflb_channel_set_channel(dev, data->channel_count, data->channel_n[channel_id],
data->channel_p[channel_id]);
	}

	tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_CONFIG2_OFFSET);
	tmpVal |= (gain << AON_GPADC_PGA1_GAIN_SHIFT);
	tmpVal |= (gain << AON_GPADC_PGA2_GAIN_SHIFT);
	sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CONFIG2_OFFSET);

	data->channel_count++;

	return 0;
}


static uint32_t adc_bflb_read_one(const struct device *dev)
{
	const struct adc_bflb_config *const cfg = dev->config;

	while ((sys_read32(cfg->reg_GPIP + GPIP_GPADC_CONFIG_OFFSET) &
	GPIP_GPADC_FIFO_DATA_COUNT_MASK) == 0) {
		/* plausible wait time at 192M, for ADC freq of 2M */
		__asm__ volatile (".rept 90 ; nop ; .endr");
	}
	return sys_read32(cfg->reg_GPIP + GPIP_GPADC_DMA_RDATA_OFFSET) & GPIP_GPADC_DMA_RDATA_MASK;
}

static void adc_bflb_trigger(const struct device *dev)
{
	const struct adc_bflb_config *const cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);
	tmpVal |= AON_GPADC_CONV_START;
	sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);
}

static void adc_bflb_detrigger(const struct device *dev)
{
	const struct adc_bflb_config *const cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);
	tmpVal &= ~AON_GPADC_CONV_START;
	sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);
}


static int adc_bflb_read(const struct device *dev,
			const struct adc_sequence *sequence)
{
	struct adc_bflb_data *data = dev->data;
	const struct adc_bflb_config *const cfg = dev->config;
	uint32_t tmpVal;
	uint8_t chan_nb = 0;
	uint32_t nb_samples = 0;
	uint8_t sample_chans[12] = {0};

	for (uint8_t i = 0; i < 12; i++) {
		if ((sequence->channels >> i) & 0x1) {
			sample_chans[chan_nb] = i;
			chan_nb += 1;
		}
	}


	nb_samples = sequence->buffer_size / 2 / chan_nb;
	if (nb_samples < 1) {
		LOG_ERR("resolution 12 to 16 bits, buffer size invalid");
		return -EINVAL;
	}

	tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_CONFIG1_OFFSET);
	tmpVal &= AON_GPADC_RES_SEL_UMSK;

	switch (sequence->resolution) {
	case 12:
		tmpVal |= (0 << AON_GPADC_RES_SEL_SHIFT);
		break;
	case 14:
		tmpVal |= (2 << AON_GPADC_RES_SEL_SHIFT);
		break;
	case 16:
		tmpVal |= (4 << AON_GPADC_RES_SEL_SHIFT);
		break;
	default:
		LOG_ERR("resolution 12, 14 or 16 bits, resolution invalid");
		return -EINVAL;
	}
	sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CONFIG1_OFFSET);

	tmpVal = sys_read32(cfg->reg_GPIP + GPIP_GPADC_CONFIG_OFFSET);
	tmpVal |= GPIP_GPADC_FIFO_CLR;
	sys_write32(tmpVal, cfg->reg_GPIP + GPIP_GPADC_CONFIG_OFFSET);

	adc_bflb_trigger(dev);

	for (int i = 0; i < nb_samples; i++) {
		for (int j = 0; j < chan_nb; j++) {
			tmpVal = adc_bflb_read_one(dev);
			while ((tmpVal & 0x3E00000) >> 21 != data->channel_p[sample_chans[j]]) {
				tmpVal = adc_bflb_read_one(dev);
				LOG_INF(
"FIFO sequence miss, prefer requesting all channels at once");
			}
			((uint16_t *)sequence->buffer)[i * chan_nb + j] = (tmpVal & 0xFFFF) >> (16 -
sequence->resolution);
		}
	}

	adc_bflb_detrigger(dev);

	return 0;
}

static void adc_bflb_isr(const struct device *dev)
{
}


#if defined(CONFIG_SOC_SERIES_BL60X)
static void adc_bflb_calibrate(const struct device *dev)
{
	const struct adc_bflb_config *const cfg = dev->config;
	uint32_t tmpVal = 0;
	uint32_t offset = 0;
	bool negative = false;

	tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_CONFIG1_OFFSET);
	/* resolution 16-bits */
	tmpVal |= (4 << AON_GPADC_RES_SEL_SHIFT);
	/* continuous mode */
	tmpVal |= AON_GPADC_CONT_CONV_EN;
	sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CONFIG1_OFFSET);

	tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_CONFIG2_OFFSET);
	tmpVal |= AON_GPADC_DIFF_MODE; /*diff mode yes*/
	tmpVal |= AON_GPADC_VBAT_EN;   /*vbat en*/
	tmpVal &= ~AON_GPADC_VREF_SEL; /* 1.8v ref */
	sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CONFIG2_OFFSET);

	tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);
	tmpVal &= ~AON_GPADC_NEG_GND;
	tmpVal &= ~AON_GPADC_POS_SEL_MASK;
	tmpVal &= ~AON_GPADC_NEG_SEL_MASK;
	/* VBAT/2 channel */
	tmpVal |= (18 << AON_GPADC_POS_SEL_SHIFT);
	/* VBAT/2 channel */
	tmpVal |= (18 << AON_GPADC_NEG_SEL_SHIFT);
	sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);

	adc_bflb_trigger(dev);
	/* 10 samplings */
	for (uint8_t i = 0; i < 10; i++) {
		tmpVal = adc_bflb_read_one(dev);
		/* only consider samples after the first 5 */
		if (i > 4) {
			if (tmpVal & 0x8000) {
				negative = true;
				tmpVal = ~tmpVal;
				tmpVal += 1;
			}
			offset += (tmpVal & 0xffff);
		}
	}

	adc_bflb_detrigger(dev);
	offset = offset / 5;
	if (negative) {
		adc_bflb_api.ref_internal = adc_bflb_api.ref_internal - offset;
	} else {
		adc_bflb_api.ref_internal = adc_bflb_api.ref_internal + offset;
	}

	tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_CONFIG2_OFFSET);
	tmpVal &= ~AON_GPADC_DIFF_MODE; /*diff mode no*/
	tmpVal &= ~AON_GPADC_VBAT_EN;   /*vbat no*/
	tmpVal &= ~AON_GPADC_VREF_SEL; /* 3.3v ref */
	sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CONFIG2_OFFSET);
}

#elif defined(CONFIG_SOC_SERIES_BL70X)
/* this is how it is calibrated from SDK, it gives bad results for BL60x, even when using the SDK */
static int adc_bflb_trim2(const struct device *dev)
{
	uint32_t tmpVal = 0;
	uint32_t trim = 0;
	float coe = 0.0;
	const struct device *efuse = DEVICE_DT_GET(DT_NODELABEL(efuse));

	tmpVal = eeprom_read(efuse, 0x78, &trim, 4);
	if (tmpVal < 0) {
		LOG_ERR("Error: Couldn't read efuses: err: %d.\n", tmpVal);
		return -EINVAL;
	}
	if ((trim & 0x4000) == 0) {
		LOG_ERR("Error: ADC calibration data not present");
		return -EINVAL;
	}
	trim = (trim & 0x1FFE) >> 1;
	if (trim & 0x800) {
		trim = ~trim;
		trim += 1;
		trim = trim & 0xfff;
		coe = ((float)1.0 + ((float)trim / (float)2048.0));
		adc_bflb_api.ref_internal = 3300 / coe;
	} else {
		coe = ((float)1.0 - ((float)trim / (float)2048.0));
		adc_bflb_api.ref_internal = 3300 / coe;
	}
	return 0;
}
#elif defined(CONFIG_SOC_SERIES_BL61X)
static int adc_bflb_trim_bl61x(const struct device *dev)
{
	uint32_t tmpVal = 0;
	uint32_t trim = 0;
	float coe = 0.0;
	const struct device *efuse = DEVICE_DT_GET(DT_NODELABEL(efuse));

	tmpVal = eeprom_read(efuse, 0xF0, &trim, 4);
	if (tmpVal < 0) {
		LOG_ERR("Error: Couldn't read efuses: err: %d.\n", tmpVal);
		return -EINVAL;
	}
	if ((trim & 0x4000000) == 0) {
		LOG_ERR("Error: ADC calibration data not present");
		return -EINVAL;
	}
	trim = (trim & 0x3FFC000) >> 14;
	if (trim & 0x800) {
		trim = ~trim;
		trim += 1;
		trim = trim & 0xfff;
		coe = ((float)1.0 + ((float)trim / (float)2048.0));
		adc_bflb_api.ref_internal = 3300 / coe;
	} else {
		coe = ((float)1.0 - ((float)trim / (float)2048.0));
		adc_bflb_api.ref_internal = 3300 / coe;
	}
	return 0;
}
#endif

#if defined(CONFIG_SOC_SERIES_BL60X)
static void adc_bflb_init_clock(const struct device *dev)
{
	uint32_t	tmpVal = 0;

	/* clock pathing*/
	tmpVal = sys_read32(GLB_BASE + GLB_GPADC_32M_SRC_CTRL_OFFSET);
	/* clock = 96M PLL */
	tmpVal &= ~GLB_GPADC_32M_CLK_SEL_MSK;
	/* div = 3 so ADC gets 32Mhz */
	tmpVal &= ~GLB_GPADC_32M_CLK_DIV_MSK;
	tmpVal |= 2 << GLB_GPADC_32M_CLK_DIV_POS;
	/* enable */
	tmpVal |= 1 << GLB_GPADC_32M_DIV_EN_POS;
	sys_write32(tmpVal, GLB_BASE + GLB_GPADC_32M_SRC_CTRL_OFFSET);
}
#elif defined(CONFIG_SOC_SERIES_BL70X)
static void adc_bflb_init_clock(const struct device *dev)
{
	uint32_t	tmpVal = 0;

	/* clock pathing*/
	tmpVal = sys_read32(GLB_BASE + GLB_GPADC_32M_SRC_CTRL_OFFSET);
	/* clock = XTAL or RC32M (32M) */
	tmpVal |= GLB_GPADC_32M_CLK_SEL_MSK;
	/* div = 1 so ADC gets 32Mhz */
	tmpVal &= ~GLB_GPADC_32M_CLK_DIV_MSK;
	/* enable */
	tmpVal |= 1 << GLB_GPADC_32M_DIV_EN_POS;
	sys_write32(tmpVal, GLB_BASE + GLB_GPADC_32M_SRC_CTRL_OFFSET);
}
#elif defined(CONFIG_SOC_SERIES_BL61X)
static void adc_bflb_init_clock(const struct device *dev)
{
	uint32_t	tmpVal = 0;

	/* clock pathing*/
	tmpVal = sys_read32(GLB_BASE + GLB_ADC_CFG0_OFFSET);
	/* clock = XTAL or RC32M (32M) */
	tmpVal |= GLB_GPADC_32M_CLK_SEL_MSK;
	/* div = 1 so ADC gets 32Mhz */
	tmpVal &= ~GLB_GPADC_32M_CLK_DIV_MSK;
	/* enable */
	tmpVal |= 1 << GLB_GPADC_32M_DIV_EN_POS;
	sys_write32(tmpVal, GLB_BASE + GLB_ADC_CFG0_OFFSET);
}
#endif

static int adc_bflb_init(const struct device *dev)
{
	const struct adc_bflb_config *const cfg = dev->config;
	uint32_t	tmpVal = 0;
	int		ret;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	adc_bflb_init_clock(dev);

	/* peripheral reset sequence */
	tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);
	tmpVal &= ~AON_GPADC_GLOBAL_EN;
	sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);

	tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);
	tmpVal |= AON_GPADC_GLOBAL_EN;
	sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);

	tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);
	tmpVal |= AON_GPADC_SOFT_RST;
	sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);

	__asm__ volatile (".rept 25 ; nop ; .endr");

	tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);
	tmpVal &= ~AON_GPADC_CONV_START;
	sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);

	tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);
	tmpVal &= ~AON_GPADC_SOFT_RST;
	sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);

	tmpVal = 0;
	/* enable power to adc? */
	tmpVal |= (2 << AON_GPADC_V18_SEL_SHIFT);
	tmpVal |= (1 << AON_GPADC_V11_SEL_SHIFT);
	/* set internal clock divider to 32 */
	tmpVal |= (7 << AON_GPADC_CLK_DIV_RATIO_SHIFT);
	/* default resolution (12-bits) */
	tmpVal |= (0 << AON_GPADC_RES_SEL_SHIFT);
	/* continuous mode */
	tmpVal |= AON_GPADC_CONT_CONV_EN;

	sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CONFIG1_OFFSET);

	__asm__ volatile (".rept 25 ; nop ; .endr");

	tmpVal = 0;
	/* ""conversion speed"" */
	tmpVal |= (2 << AON_GPADC_DLY_SEL_SHIFT);
	/* ""Vref AZ and chop on"" */
	tmpVal |= (2 << AON_GPADC_CHOP_MODE_SHIFT);
	/* "gain 1" is 1 */
	tmpVal |= (1 << AON_GPADC_PGA1_GAIN_SHIFT);
	/* "gain 2" is 1 */
	tmpVal |= (1 << AON_GPADC_PGA2_GAIN_SHIFT);
	/* enable gain */
	tmpVal |= AON_GPADC_PGA_EN;
	/* "offset calibration" value */
	tmpVal |= (8 << AON_GPADC_PGA_OS_CAL_SHIFT);
	/* "VCM" is 1.2v */
	tmpVal |= (1 << AON_GPADC_PGA_VCM_SHIFT);
	/* ADC reference (VREF channel) is 3v3 */
	tmpVal &= ~AON_GPADC_VREF_SEL;

	sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CONFIG2_OFFSET);


	tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);
	/* "MIC2" differential mode enable */
	tmpVal |= AON_GPADC_MIC2_DIFF;
	/* single ended mode is achieved by setting differential other end to ground */
	tmpVal |= AON_GPADC_NEG_GND;
	sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_CMD_OFFSET);

	/* clear calibration */
	tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_DEFINE_OFFSET);
	tmpVal &= ~AON_GPADC_OS_CAL_DATA_MASK;
	sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_DEFINE_OFFSET);


	/* interrupts and status setup */
	tmpVal = sys_read32(cfg->reg_GPIP + GPIP_GPADC_CONFIG_OFFSET);
	tmpVal |= (GPIP_GPADC_FIFO_UNDERRUN_MASK | GPIP_GPADC_FIFO_OVERRUN_MASK |
GPIP_GPADC_RDY_MASK | GPIP_GPADC_FIFO_UNDERRUN_CLR | GPIP_GPADC_FIFO_OVERRUN_CLR |
GPIP_GPADC_RDY_CLR);
#ifdef CONFIG_SOC_SERIES_BL70x
	tmpVal |= (GPIP_GPADC_FIFO_RDY_MASK | GPIP_GPADC_FIFO_RDY);
#endif
	tmpVal |= GPIP_GPADC_FIFO_CLR;
	tmpVal &= ~GPIP_GPADC_FIFO_THL_MASK;
	tmpVal &= ~GPIP_GPADC_DMA_EN;
	sys_write32(tmpVal, cfg->reg_GPIP + GPIP_GPADC_CONFIG_OFFSET);

	__asm__ volatile (".rept 5 ; nop ; .endr");

	tmpVal = sys_read32(cfg->reg_GPIP + GPIP_GPADC_CONFIG_OFFSET);
	tmpVal &=	~(GPIP_GPADC_FIFO_UNDERRUN_CLR |
			GPIP_GPADC_FIFO_OVERRUN_CLR |
			GPIP_GPADC_RDY_CLR |
			GPIP_GPADC_FIFO_CLR);
	sys_write32(tmpVal, cfg->reg_GPIP + GPIP_GPADC_CONFIG_OFFSET);

	tmpVal = sys_read32(cfg->reg_AON + AON_GPADC_REG_ISR_OFFSET);
	tmpVal |= AON_GPADC_NEG_SATUR_MASK;
	tmpVal |= AON_GPADC_POS_SATUR_MASK;
	sys_write32(tmpVal, cfg->reg_AON + AON_GPADC_REG_ISR_OFFSET);

#if defined(CONFIG_SOC_SERIES_BL70X)
	adc_bflb_trim2(dev);
#elif defined(CONFIG_SOC_SERIES_BL60X)
	adc_bflb_calibrate(dev);
#elif defined(CONFIG_SOC_SERIES_BL61X)
	adc_bflb_trim_bl61x(dev);
#endif

	cfg->irq_config_func(dev);
	return 0;
}


#define ADC_BFLB_DEVICE(n)						\
	PINCTRL_DT_INST_DEFINE(n);					\
	static void adc_bflb_irq_config_##n(const struct device *dev)	\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority),			\
			    adc_bflb_isr,				\
			    DEVICE_DT_INST_GET(n), 0);			\
		irq_enable(DT_INST_IRQN(n));				\
	}								\
	static const struct adc_bflb_config adc_bflb_config_##n = {	\
		.reg_GPIP = DT_INST_REG_ADDR_BY_IDX(n, 0),		\
		.reg_AON = DT_INST_REG_ADDR_BY_IDX(n, 1),		\
		.irq_config_func = &adc_bflb_irq_config_##n,		\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
	};								\
	static struct adc_bflb_data adc_bflb_data_##n = {		\
		.channel_count = 0,					\
		.gain = ADC_GAIN_128,					\
	};								\
	DEVICE_DT_INST_DEFINE(n, adc_bflb_init, NULL,			\
			      &adc_bflb_data_##n,			\
			      &adc_bflb_config_##n, POST_KERNEL,	\
			      CONFIG_ADC_INIT_PRIORITY,			\
			      &adc_bflb_api);

DT_INST_FOREACH_STATUS_OKAY(ADC_BFLB_DEVICE)
