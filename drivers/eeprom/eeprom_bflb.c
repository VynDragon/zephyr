/*
 * Copyright (c) 2019 Kwon Tae-young <tykwon@m2i.co.kr>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_efuse

#include <zephyr/drivers/eeprom.h>
#include <soc.h>
#include <zephyr/kernel.h>

#define LOG_LEVEL CONFIG_EEPROM_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eeprom_bflb);

struct eeprom_bflb_data {
	uint8_t cache[DT_INST_PROP(0, size)];
	bool cached;
};

struct eeprom_bflb_config {
	uint32_t addr;
	size_t size;
};


static void system_clock_settle(void)
{
	__asm__ volatile (".rept 15 ; nop ; .endr");
}

/* 32 Mhz Oscillator: 0
 * crystal: 1
 * PLL and 32M: 2
 * PLL and crystal: 3
 */
static void system_set_root_clock(uint32_t clock)
{
	uint32_t tmpVal = 0;

	/* invalid value, fallback to internal 32M */
	if (clock < 0 || clock > 3) {
		clock = 0;
	}
	tmpVal = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmpVal = (tmpVal & HBN_ROOT_CLK_SEL_UMSK) | (clock << HBN_ROOT_CLK_SEL_POS);
	sys_write32(tmpVal, HBN_BASE + HBN_GLB_OFFSET);

	system_clock_settle();
}

static void system_clock_delay_32M_ms(uint32_t ms)
{
	uint32_t count = 0;

	do {
		__asm__ volatile (".rept 32 ; nop ; .endr");
		count++;
	} while (count < ms);
}

static uint32_t is_pds_busy(void)
{
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(BFLB_EF_CTRL_BASE + EF_CTRL_EF_IF_CTRL_0_OFFSET);
	if (tmpVal & EF_CTRL_EF_IF_0_BUSY_MSK) {
		return 1;
	}
	return 0;
}

/* /!\ only use when running on 32Mhz Oscillator Clock
 * (system_set_root_clock(0);
 * system_set_root_clock_dividers(0, 0);
 * sys_write32(32 * 1000 * 1000, CORECLOCKREGISTER);)
 * Only Use with IRQs off
 * returns 0 when error
 */
static void system_efuse_read(void)
{
	uint32_t tmpVal = 0;
	uint32_t *pefuse_start = (uint32_t *)(BFLB_EF_CTRL_BASE);
	uint32_t timeout = 0;

	do {
		system_clock_delay_32M_ms(1);
		timeout++;
	} while (timeout < EF_CTRL_DFT_TIMEOUT_VAL && is_pds_busy() > 0);

	/* do a 'ahb clock' setup */
	tmpVal =	EF_CTRL_EFUSE_CTRL_PROTECT |
			(EF_CTRL_OP_MODE_AUTO << EF_CTRL_EF_IF_0_MANUAL_EN_POS) |
			(EF_CTRL_PARA_DFT << EF_CTRL_EF_IF_0_CYC_MODIFY_POS) |
			(EF_CTRL_SAHB_CLK << EF_CTRL_EF_CLK_SAHB_DATA_SEL_POS) |
			(1 << EF_CTRL_EF_IF_AUTO_RD_EN_POS) |
			(0 << EF_CTRL_EF_IF_POR_DIG_POS) |
			(1 << EF_CTRL_EF_IF_0_INT_CLR_POS) |
			(0 << EF_CTRL_EF_IF_0_RW_POS) |
			(0 << EF_CTRL_EF_IF_0_TRIG_POS);

	sys_write32(tmpVal, BFLB_EF_CTRL_BASE + EF_CTRL_EF_IF_CTRL_0_OFFSET);
	system_clock_settle();

	/* clear PDS cache registry */
	for (uint32_t i = 0; i < EF_CTRL_EFUSE_R0_SIZE / 4; i++) {
		pefuse_start[i] = 0;
	}

	/* Load efuse region0 */
	/* not ahb clock setup */
	tmpVal =	EF_CTRL_EFUSE_CTRL_PROTECT |
			(EF_CTRL_OP_MODE_AUTO << EF_CTRL_EF_IF_0_MANUAL_EN_POS) |
			(EF_CTRL_PARA_DFT << EF_CTRL_EF_IF_0_CYC_MODIFY_POS) |
			(EF_CTRL_EF_CLK << EF_CTRL_EF_CLK_SAHB_DATA_SEL_POS) |
			(1 << EF_CTRL_EF_IF_AUTO_RD_EN_POS) |
			(0 << EF_CTRL_EF_IF_POR_DIG_POS) |
			(1 << EF_CTRL_EF_IF_0_INT_CLR_POS) |
			(0 << EF_CTRL_EF_IF_0_RW_POS) |
			(0 << EF_CTRL_EF_IF_0_TRIG_POS);
	sys_write32(tmpVal, BFLB_EF_CTRL_BASE + EF_CTRL_EF_IF_CTRL_0_OFFSET);

	/* trigger read */
	tmpVal =	EF_CTRL_EFUSE_CTRL_PROTECT |
			(EF_CTRL_OP_MODE_AUTO << EF_CTRL_EF_IF_0_MANUAL_EN_POS) |
			(EF_CTRL_PARA_DFT << EF_CTRL_EF_IF_0_CYC_MODIFY_POS) |
			(EF_CTRL_EF_CLK << EF_CTRL_EF_CLK_SAHB_DATA_SEL_POS) |
			(1 << EF_CTRL_EF_IF_AUTO_RD_EN_POS) |
			(0 << EF_CTRL_EF_IF_POR_DIG_POS) |
			(1 << EF_CTRL_EF_IF_0_INT_CLR_POS) |
			(0 << EF_CTRL_EF_IF_0_RW_POS) |
			(1 << EF_CTRL_EF_IF_0_TRIG_POS);
	sys_write32(tmpVal, BFLB_EF_CTRL_BASE + EF_CTRL_EF_IF_CTRL_0_OFFSET);
	system_clock_delay_32M_ms(5);

	/* wait for read to complete */
	do {
		system_clock_delay_32M_ms(1);
		tmpVal = sys_read32(BFLB_EF_CTRL_BASE + EF_CTRL_EF_IF_CTRL_0_OFFSET);
	} while ((tmpVal & EF_CTRL_EF_IF_0_BUSY_MSK) ||
		!(tmpVal && EF_CTRL_EF_IF_0_AUTOLOAD_DONE_MSK));

	/* do a 'ahb clock' setup */
	tmpVal =	EF_CTRL_EFUSE_CTRL_PROTECT |
			(EF_CTRL_OP_MODE_AUTO << EF_CTRL_EF_IF_0_MANUAL_EN_POS) |
			(EF_CTRL_PARA_DFT << EF_CTRL_EF_IF_0_CYC_MODIFY_POS) |
			(EF_CTRL_SAHB_CLK << EF_CTRL_EF_CLK_SAHB_DATA_SEL_POS) |
			(1 << EF_CTRL_EF_IF_AUTO_RD_EN_POS) |
			(0 << EF_CTRL_EF_IF_POR_DIG_POS) |
			(1 << EF_CTRL_EF_IF_0_INT_CLR_POS) |
			(0 << EF_CTRL_EF_IF_0_RW_POS) |
			(0 << EF_CTRL_EF_IF_0_TRIG_POS);

	sys_write32(tmpVal, BFLB_EF_CTRL_BASE + EF_CTRL_EF_IF_CTRL_0_OFFSET);
}

static void eeprom_bflb_cache(const struct device *dev)
{
	struct eeprom_bflb_data *data = dev->data;
	const struct eeprom_bflb_config *config = dev->config;
	uint32_t tmpVal = 0;
	uint8_t old_clock_root;

	tmpVal = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	old_clock_root = (tmpVal & HBN_ROOT_CLK_SEL_MSK) >> HBN_ROOT_CLK_SEL_POS;

	system_set_root_clock(0);
	system_clock_settle();

	system_efuse_read();
	/* reads *must* be 32-bits aligned AND does not work with the method memcpy uses */
	for (int i = 0; i < config->size / sizeof(uint32_t); i++) {
		tmpVal = sys_read32(BFLB_EF_CTRL_BASE + i * 4);
		data->cache[i * sizeof(uint32_t) + 3] = (tmpVal & 0xFF000000) >> 24;
		data->cache[i * sizeof(uint32_t) + 2] = (tmpVal & 0x00FF0000) >> 16;
		data->cache[i * sizeof(uint32_t) + 1] = (tmpVal & 0x0000FF00) >> 8;
		data->cache[i * sizeof(uint32_t) + 0] = (tmpVal & 0x000000FF);
	}

	system_set_root_clock(old_clock_root);
	system_clock_settle();
	data->cached = true;
}

static int eeprom_bflb_read(const struct device *dev, off_t offset,
				void *buf,
				size_t len)
{
	struct eeprom_bflb_data *data = dev->data;

	if (!data->cached) {
		eeprom_bflb_cache(dev);
	}

	memcpy(buf, data->cache + offset, len);
	return 0;
}

static int eeprom_bflb_write(const struct device *dev, off_t offset,
				const void *buf, size_t len)
{
	return -ENOTSUP;
}

static size_t eeprom_bflb_size(const struct device *dev)
{
	const struct eeprom_bflb_config *config = dev->config;

	return config->size;
}

static const struct eeprom_driver_api eeprom_bflb_api = {
	.read = eeprom_bflb_read,
	.write = eeprom_bflb_write,
	.size = eeprom_bflb_size,
};

static const struct eeprom_bflb_config eeprom_config = {
	.addr = DT_INST_REG_ADDR(0),
	.size = DT_INST_PROP(0, size),
};

static struct eeprom_bflb_data eeprom_data = {
	.cached = false,
	.cache = {0},
};


DEVICE_DT_INST_DEFINE(0, NULL, NULL, &eeprom_data, &eeprom_config, POST_KERNEL,
		      CONFIG_EEPROM_INIT_PRIORITY, &eeprom_bflb_api);
