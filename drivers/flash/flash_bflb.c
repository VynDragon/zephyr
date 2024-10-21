/*
 * Copyright (c) 2024 MASSDRIVER EI
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_flash_controller

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/drivers/uart.h>

#include <soc.h>


#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_bflb, CONFIG_FLASH_LOG_LEVEL);

#define ERASE_VALUE 0xFF
#define WRITE_SIZE DT_PROP(DT_CHOSEN(zephyr_flash), write_block_size)
#define ERASE_SIZE DT_PROP(DT_CHOSEN(zephyr_flash), erase_block_size)
#define TOTAL_SIZE DT_REG_SIZE(DT_CHOSEN(zephyr_flash))

#if defined(CONFIG_SOC_SERIES_BL70X) || defined(CONFIG_SOC_SERIES_BL60X)
#define BL_XIP_BASE 0x23000000
#elif defined(CONFIG_SOC_SERIES_BL61X)
#define BL_XIP_BASE 0xA0000000
#endif

#define BFLB_FLASH_CONTROLLER_BUSY_TIMEOUT_MS 200
#define BFLB_FLASH_CHIP_BUSY_TIMEOUT_MS 5000

#define FLASH_READ32(address) (*((uint32_t *)(address)))
#define FLASH_WRITE32(value, address) (*((uint32_t *)(address))) = value;

struct flash_bflb_config {
	uint32_t reg;
	void (*irq_config_func)(const struct device *dev);
};

/* Raw flash configuration data structure */
typedef struct {
/* Serial flash interface mode, bit0-3:spi mode, bit4:unwrap, bit5:32-bits addr mode support */
	uint8_t  io_mode;
/* Support continuous read mode, bit0:continuous read mode support, bit1:read mode cfg */
	uint8_t  c_read_support;
/* SPI clock delay, bit0-3:delay,bit4-6:pad delay */
	uint8_t  clk_delay;
/* SPI clock phase invert, bit0:clck invert, bit1:rx invert, bit2-4:pad delay, bit5-7:pad delay */
	uint8_t  clk_invert;
/* Flash enable reset command */
	uint8_t  reset_en_cmd;
/* Flash reset command */
	uint8_t  reset_cmd;
/* Flash reset continuous read command */
	uint8_t  reset_c_read_cmd;
/* Flash reset continuous read command size */
	uint8_t  reset_c_read_cmd_size;
/* JEDEC ID command */
	uint8_t  jedec_id_cmd;
/* JEDEC ID command dummy clock */
	uint8_t  jedec_id_cmd_dmy_clk;
	#if defined(CONFIG_SOC_SERIES_BL70X) || defined(CONFIG_SOC_SERIES_BL60X)
/* QPI JEDEC ID command */
	uint8_t  qpi_jedec_id_cmd;
/* QPI JEDEC ID command dummy clock */
	uint8_t  qpi_jedec_id_cmd_dmy_clk;
	#else
/* Enter 32-bits addr command */
	uint8_t  enter_32bits_addr_cmd;
/* Exit 32-bits addr command */
	uint8_t  exit_32bits_addr_cmd;
	#endif
/* (x * 1024) bytes */
	uint8_t  sector_size;
/* Manufacturer ID */
	uint8_t  mid;
/* Page size */
	uint16_t page_size;
/* Chip erase cmd */
	uint8_t  chip_erase_cmd;
/* Sector erase command */
	uint8_t  sector_erase_cmd;
/* Block 32K erase command*/
	uint8_t  blk32_erase_cmd;
/* Block 64K erase command */
	uint8_t  blk64_erase_cmd;
/* Write enable command, needed before every erase or program, or register write */
	uint8_t  write_enable_cmd;
/* Page program command */
	uint8_t  page_program_cmd;
/* QIO page program cmd */
	uint8_t  qpage_program_cmd;
/* QIO page program address mode */
	uint8_t  qpp_addr_mode;
/* Fast read command */
	uint8_t  fast_read_cmd;
/* Fast read command dummy clock */
	uint8_t  fr_dmy_clk;
/* QPI fast read command */
	uint8_t  qpi_fast_read_cmd;
/* QPI fast read command dummy clock */
	uint8_t  qpi_fr_dmy_clk;
/* Fast read dual output command */
	uint8_t  fast_read_do_cmd;
/* Fast read dual output command dummy clock */
	uint8_t  fr_do_dmy_clk;
/* Fast read dual io command */
	uint8_t  fast_read_dio_cmd;
/* Fast read dual io command dummy clock */
	uint8_t  fr_dio_dmy_clk;
/* Fast read quad output command */
	uint8_t  fast_read_qo_cmd;
/* Fast read quad output command dummy clock */
	uint8_t  fr_qo_dmy_clk;
/* Fast read quad io command */
	uint8_t  fast_read_qio_cmd;
/* Fast read quad io command dummy clock */
	uint8_t  fr_qio_dmy_clk;
/* QPI fast read quad io command */
	uint8_t  qpi_fast_read_qio_cmd;
/* QPI fast read QIO dummy clock */
	uint8_t  qpi_fr_qio_dmy_clk;
/* QPI program command */
	uint8_t  qpi_page_program_cmd;
/* Enable write volatile reg */
	uint8_t  write_vreg_enable_cmd;
/* Write enable register index */
	uint8_t  wr_enable_index;
/* Quad mode enable register index */
	uint8_t  qe_index;
/* Busy status register index */
	uint8_t  busy_index;
/* Write enable register bit pos */
	uint8_t  wr_enable_bit;
/* Quad enable register bit pos */
	uint8_t  qe_bit;
/* Busy status register bit pos */
	uint8_t  busy_bit;
/* Register length of write enable */
	uint8_t  wr_enable_write_reg_len;
/* Register length of write enable status */
	uint8_t  wr_enable_read_reg_len;
/* Register length of quad enable */
	uint8_t  qe_write_reg_len;
/* Register length of quad enable status */
	uint8_t  qe_read_reg_len;
/* Release power down command */
	uint8_t  release_powerdown;
/* Register length of contain busy status */
	uint8_t  busy_read_reg_len;
/* Read register command buffer */
	uint8_t  read_reg_cmd[4];
/* Write register command buffer */
	uint8_t  write_reg_cmd[4];
/* Enter qpi command */
	uint8_t  enter_qpi;
/* Exit qpi command */
	uint8_t  exit_qpi;
/* Config data for continuous read mode */
	uint8_t  c_read_mode;
/* Config data for exit continuous read mode */
	uint8_t  c_rexit;
/* Enable burst wrap command */
	uint8_t  burst_wrap_cmd;
/* Enable burst wrap command dummy clock */
	uint8_t  burst_wrap_cmd_dmy_clk;
/* Data and address mode for this command */
	uint8_t  burst_wrap_data_mode;
/* Data to enable burst wrap */
	uint8_t  burst_wrap_data;
/* Disable burst wrap command */
	uint8_t  de_burst_wrap_cmd;
/* Disable burst wrap command dummy clock */
	uint8_t  de_burst_wrap_cmd_dmy_clk;
/* Data and address mode for this command */
	uint8_t  de_burst_wrap_data_mode;
/* Data to disable burst wrap */
	uint8_t  de_burst_wrap_data;
/* Typical 4K(usually) erase time */
	uint16_t time_e_sector;
/* Typical 32K erase time */
	uint16_t time_e_32k;
/* Typical 64K erase time */
	uint16_t time_e_64k;
/* Typical Page program time */
	uint16_t time_page_pgm;
/* Typical Chip erase time in ms */
	uint16_t time_ce;
/* Release power down command delay time for wake up */
	uint8_t  pd_delay;
/* QE set data */
	uint8_t  qe_data;
} __packed bflb_flash_cfg;

typedef struct {
/* Read write 0: read 1 : write */
	uint8_t rw;
/* Command mode 0: 1 line, 1: 4 lines */
	uint8_t cmd_mode;
/* SPI mode 0: IO 1: DO 2: QO 3: DIO 4: QIO */
	uint8_t spi_mode;
/* Address size */
	uint8_t addr_size;
/* Dummy clocks */
	uint8_t dummy_clks;
/* Transfer number of bytes */
	uint32_t nb_data;
/* Command buffer */
	uint32_t cmd_buf[2];
} bflb_flash_command;

struct flash_bflb_data {
	bflb_flash_cfg flash_cfg;
	bflb_flash_cfg flash_cfg_backup;
	uint32_t last_flash_offset;
	uint32_t reg_copy;
	uint32_t jedec_id;
};


#if defined(CONFIG_SOC_SERIES_BL70X) || defined(CONFIG_SOC_SERIES_BL60X)

/* will using function cause error ? */
static bool flash_bflb_guard(void *func)
{
	if ((uint32_t)func > BL_XIP_BASE && (uint32_t)func < 0x24000000) {
		LOG_ERR("function at %d is in XIP and will crash the device", (uint32_t)func);
		return true;
	}
	return false;
}

static void flash_bflb_cache_invalidate(void)
{
	uint32_t tmpVal = 0;

	tmpVal = *((uint32_t *)(L1C_BASE + L1C_CONFIG_OFFSET));
	tmpVal |= 1 << L1C_BYPASS_POS;
	tmpVal &= ~(1 << L1C_CACHEABLE_POS);
	tmpVal &= ~L1C_WAY_DIS_MSK;
	tmpVal &= ~(1 << L1C_CNT_EN_POS);
	*((uint32_t *)(L1C_BASE + L1C_CONFIG_OFFSET)) = tmpVal;

	tmpVal &= ~(1 << L1C_INVALID_EN_POS);
	*((uint32_t *)(L1C_BASE + L1C_CONFIG_OFFSET)) = tmpVal;
	__asm__ volatile (".rept 4 ; nop ; .endr");

	tmpVal |= (1 << L1C_INVALID_EN_POS);
	*((uint32_t *)(L1C_BASE + L1C_CONFIG_OFFSET)) = tmpVal;
	__asm__ volatile (".rept 4 ; nop ; .endr");

	while ((*((uint32_t *)(L1C_BASE + L1C_CONFIG_OFFSET)) & L1C_INVALID_DONE_MSK) == 0) {
		__asm__ volatile (".rept 50 ; nop ; .endr");
	}

	tmpVal = *((uint32_t *)(L1C_BASE + L1C_CONFIG_OFFSET));
	tmpVal |= 1 << L1C_BYPASS_POS;
	*((uint32_t *)(L1C_BASE + L1C_CONFIG_OFFSET)) = tmpVal;

	tmpVal &= ~(1 << L1C_BYPASS_POS);
	tmpVal |= 1 << L1C_CNT_EN_POS;
	*((uint32_t *)(L1C_BASE + L1C_CONFIG_OFFSET)) = tmpVal;

	tmpVal &= ~L1C_WAY_DIS_MSK;
	*((uint32_t *)(L1C_BASE + L1C_CONFIG_OFFSET)) = tmpVal;

	tmpVal |= 1 << L1C_CACHEABLE_POS;
	*((uint32_t *)(L1C_BASE + L1C_CONFIG_OFFSET)) = tmpVal;

}

static void flash_bflb_l1c_wrap(bool yes)
{
	uint32_t tmpVal = 0;
	bool caching = false;

	tmpVal = FLASH_READ32(L1C_BASE + L1C_CONFIG_OFFSET);
	/* disable cache */
	if ((tmpVal & L1C_CACHEABLE_MSK) != 0) {
		caching = true;
		tmpVal &= ~(1 << L1C_CACHEABLE_POS);
		FLASH_WRITE32(tmpVal, L1C_BASE + L1C_CONFIG_OFFSET);
	}

	tmpVal = FLASH_READ32(L1C_BASE + L1C_CONFIG_OFFSET);

	if (yes) {
		tmpVal &= ~L1C_WRAP_DIS_MSK;
	} else {
		tmpVal |= L1C_WRAP_DIS_MSK;
	}

	FLASH_WRITE32(tmpVal, L1C_BASE + L1C_CONFIG_OFFSET);

	if (caching) {
		tmpVal |= (1 << L1C_CACHEABLE_POS);
		FLASH_WRITE32(tmpVal, L1C_BASE + L1C_CONFIG_OFFSET);
	}
}


#elif defined(CONFIG_SOC_SERIES_BL61X)

static bool flash_bflb_guard(void *func)
{
	if ((uint32_t)func > BL_XIP_BASE && (uint32_t)func < 0xA8000000) {
		LOG_ERR("function at %d is in XIP and will crash the device", (uint32_t)func);
		return true;
	}
	return false;
}

static void clean_dcache(void)
{
	__asm__ volatile (
		"fence\n"
		/* th.dcache.ciall*/
		".insn 0x30000B\n"
		"fence\n"
	);
}

static void clean_icache(void)
{
	__asm__ volatile (
		"fence\n"
		"fence.i\n"
		/* th.icache.iall */
		".insn 0x100000B\n"
		"fence\n"
		"fence.i\n"
	);
}

static void flash_bflb_cache_invalidate(void)
{
	clean_dcache();
	clean_icache();
}

static void flash_bflb_l1c_wrap(bool yes)
{
}


#endif

static void flash_bflb_xip_memcpy(uint8_t *address_from, uint8_t *address_to, uint32_t size)
{
	for (unsigned int i = 0; i < size; i++) {
		address_to[i] = address_from[i];
	}
}

static bool flash_bflb_busy_wait(struct flash_bflb_data *data)
{
	uint32_t counter = 0;

	while ((FLASH_READ32(data->reg_copy + SF_CTRL_SF_IF_SAHB_0_OFFSET + 0) &
SF_CTRL_SF_IF_BUSY_MSK) != 0 && counter < BFLB_FLASH_CONTROLLER_BUSY_TIMEOUT_MS * 20000) {
		__asm__ volatile (".rept 10 ; nop ; .endr");
		counter++;
	}

	if ((FLASH_READ32(data->reg_copy + SF_CTRL_SF_IF_SAHB_0_OFFSET + 0) &
SF_CTRL_SF_IF_BUSY_MSK) != 0) {
		return true;
	}

	return false;
}


/* Sets which AHB the flash controller is being talked to from
 * 0: System AHB (AHB connected to everything, E24 System Port)
 * 1: Instruction AHB (a dedicated bus between flash controller and L1C)
 */
static int flash_bflb_set_bus(struct flash_bflb_data *data, uint8_t bus)
{
	uint32_t tmpVal = 0;

	if (flash_bflb_busy_wait(data)) {
		return -EBUSY;
	}

	tmpVal = FLASH_READ32(data->reg_copy + SF_CTRL_1_OFFSET);
	if (bus == 1) {
		tmpVal |= SF_CTRL_SF_IF_FN_SEL_MSK;
		tmpVal |= SF_CTRL_SF_AHB2SIF_EN_MSK;
	} else {
		tmpVal &= ~SF_CTRL_SF_IF_FN_SEL_MSK;
		tmpVal &= ~SF_CTRL_SF_AHB2SIF_EN_MSK;
	}
	FLASH_WRITE32(tmpVal, data->reg_copy + SF_CTRL_1_OFFSET);
	return 0;
}

static uint8_t flash_bflb_admode_to_spimode(uint8_t addr_mode, uint8_t data_mode)
{
	if (addr_mode == 0) {
		if (data_mode == 0) {
			return 0;
		} else if (data_mode == 1) {
			return 1;
		} else if (data_mode == 2) {
			return 2;
		}
	} else if (addr_mode == 1) {
		return 3;
	} else if (addr_mode == 2) {
		return 4;
	}
	return 0;
}

static int flash_bflb_set_command_iahb(struct flash_bflb_data *data, bflb_flash_command *command,
bool doing_cmd)
{
	uint32_t tmpVal = 0;
	uint32_t bank_offset = data->reg_copy + SF_CTRL_SF_IF_IAHB_0_OFFSET;

	if (flash_bflb_busy_wait(data)) {
		return -EBUSY;
	}

	tmpVal = FLASH_READ32(data->reg_copy + SF_CTRL_1_OFFSET);
	if ((tmpVal & SF_CTRL_SF_IF_FN_SEL_MSK) == 0) {
		LOG_ERR("Flash's Bus must be Instruction AHB and not System AHB");
		return -EINVAL;
	}

	FLASH_WRITE32(command->cmd_buf[0], bank_offset + 0x4);
	FLASH_WRITE32(command->cmd_buf[1], bank_offset + 0x8);

	tmpVal = FLASH_READ32(bank_offset + 0);

	/* 4 lines or 1 line commands */
	if (command->cmd_mode == 0) {
		tmpVal &= ~SF_CTRL_SF_IF_1_QPI_MODE_EN_MSK;
	} else {
		tmpVal |= SF_CTRL_SF_IF_1_QPI_MODE_EN_MSK;
	}

	/* set SPI mode*/
	tmpVal &= ~SF_CTRL_SF_IF_1_SPI_MODE_MSK;
	tmpVal |= command->spi_mode << SF_CTRL_SF_IF_1_SPI_MODE_POS;

	tmpVal &= ~SF_CTRL_SF_IF_1_CMD_BYTE_MSK;
	/* we are doing a command */
	if (doing_cmd) {
		tmpVal |= SF_CTRL_SF_IF_1_CMD_EN_MSK;
	} else {
		tmpVal &= ~SF_CTRL_SF_IF_1_CMD_EN_MSK;
	}

	/* configure address */
	tmpVal &= ~SF_CTRL_SF_IF_1_ADR_BYTE_MSK;
	if (command->addr_size != 0) {
		tmpVal |= SF_CTRL_SF_IF_1_ADR_EN_MSK;
		tmpVal |= ((command->addr_size - 1) << SF_CTRL_SF_IF_1_ADR_BYTE_POS);
	} else {
		tmpVal &= ~SF_CTRL_SF_IF_1_ADR_EN_MSK;
	}

	/* configure dummy */
	tmpVal &= ~SF_CTRL_SF_IF_1_DMY_BYTE_MSK;
	if (command->dummy_clks != 0) {
		tmpVal |= SF_CTRL_SF_IF_1_DMY_EN_MSK;
		tmpVal |= ((command->dummy_clks - 1) << SF_CTRL_SF_IF_1_DMY_BYTE_POS);
	} else {
		tmpVal &= ~SF_CTRL_SF_IF_1_DMY_EN_MSK;
	}

	/* configure data */
	if (command->nb_data != 0) {
		tmpVal |= SF_CTRL_SF_IF_1_DAT_EN_MSK;
	} else {
		tmpVal &= ~SF_CTRL_SF_IF_1_DAT_EN_MSK;
	}

	/* writing ? */
	if (command->rw) {
		tmpVal |= SF_CTRL_SF_IF_1_DAT_RW_MSK;
	} else {
		tmpVal &= ~SF_CTRL_SF_IF_1_DAT_RW_MSK;
	}
	FLASH_WRITE32(tmpVal, bank_offset + 0);

	return 0;
}

static int flash_bflb_set_command_sahb(struct flash_bflb_data *data, bflb_flash_command *command,
bool doing_cmd)
{
	uint32_t tmpVal = 0;
	uint32_t bank_offset = data->reg_copy + SF_CTRL_SF_IF_SAHB_0_OFFSET;

	if (flash_bflb_busy_wait(data)) {
		return -EBUSY;
	}

	FLASH_WRITE32(command->cmd_buf[0], bank_offset + 0x4);
	FLASH_WRITE32(command->cmd_buf[1], bank_offset + 0x8);

	tmpVal = FLASH_READ32(bank_offset + 0);

	/* 4 lines or 1 line commands */
	if (command->cmd_mode == 0) {
		tmpVal &= ~SF_CTRL_SF_IF_0_QPI_MODE_EN_MSK;
	} else {
		tmpVal |= SF_CTRL_SF_IF_0_QPI_MODE_EN_MSK;
	}

	/* set SPI mode*/
	tmpVal &= ~SF_CTRL_SF_IF_0_SPI_MODE_MSK;
	tmpVal |= command->spi_mode << SF_CTRL_SF_IF_0_SPI_MODE_POS;

	tmpVal &= ~SF_CTRL_SF_IF_0_CMD_BYTE_MSK;
	/* we are doing a command */
	if (doing_cmd) {
		tmpVal |= SF_CTRL_SF_IF_0_CMD_EN_MSK;
	} else {
		tmpVal &= ~SF_CTRL_SF_IF_0_CMD_EN_MSK;
	}


	/* configure address */
	tmpVal &= ~SF_CTRL_SF_IF_0_ADR_BYTE_MSK;
	if (command->addr_size != 0) {
		tmpVal |= SF_CTRL_SF_IF_0_ADR_EN_MSK;
		tmpVal |= ((command->addr_size - 1) << SF_CTRL_SF_IF_0_ADR_BYTE_POS);
	} else {
		tmpVal &= ~SF_CTRL_SF_IF_0_ADR_EN_MSK;
	}

	/* configure dummy */
	tmpVal &= ~SF_CTRL_SF_IF_0_DMY_BYTE_MSK;
	if (command->dummy_clks != 0) {
		tmpVal |= SF_CTRL_SF_IF_0_DMY_EN_MSK;
		tmpVal |= ((command->dummy_clks - 1) << SF_CTRL_SF_IF_0_DMY_BYTE_POS);
	} else {
		tmpVal &= ~SF_CTRL_SF_IF_0_DMY_EN_MSK;
	}

	/* configure data */
	tmpVal &= ~SF_CTRL_SF_IF_0_DAT_BYTE_MSK;
	if (command->nb_data != 0) {
		tmpVal |= SF_CTRL_SF_IF_0_DAT_EN_MSK;
		tmpVal |= ((command->nb_data - 1) << SF_CTRL_SF_IF_0_DAT_BYTE_POS);
	} else {
		tmpVal &= ~SF_CTRL_SF_IF_0_DAT_EN_MSK;
	}

	/* writing ? */
	if (command->rw) {
		tmpVal |= SF_CTRL_SF_IF_0_DAT_RW_MSK;
	} else {
		tmpVal &= ~SF_CTRL_SF_IF_0_DAT_RW_MSK;
	}
	FLASH_WRITE32(tmpVal, bank_offset + 0);

	return 0;
}

static int flash_bflb_send_command(struct flash_bflb_data *data, bflb_flash_command *command)
{
	uint32_t tmpVal = 0;
	/* TODO: multiple flash support
	 * Danger: headers does not match names between bl6/70x and bl61x
	 */
	uint32_t bank_offset = data->reg_copy + SF_CTRL_SF_IF_SAHB_0_OFFSET;

	if (flash_bflb_guard(&flash_bflb_send_command)) {
		return -ENOTSUP;
	}

	if (flash_bflb_busy_wait(data)) {
		return -EBUSY;
	}

	tmpVal = FLASH_READ32(data->reg_copy + SF_CTRL_1_OFFSET);
	if (tmpVal & SF_CTRL_SF_IF_FN_SEL_MSK) {
		LOG_ERR("Flash's Bus must be System AHB and not Instruction AHB");
		return -EINVAL;
	}

	/* make sure command detriggered */
	tmpVal = FLASH_READ32(bank_offset + 0);
	tmpVal &= ~SF_CTRL_SF_IF_0_TRIG_MSK;
	FLASH_WRITE32(tmpVal, bank_offset + 0);

	tmpVal = flash_bflb_set_command_sahb(data, command, true);
	if (tmpVal != 0) {
		return tmpVal;
	}

#if defined(CONFIG_SOC_SERIES_BL70X) || defined(CONFIG_SOC_SERIES_BL60X)
	tmpVal = FLASH_READ32(data->reg_copy + SF_CTRL_0_OFFSET);
	tmpVal |= SF_CTRL_SF_CLK_SAHB_SRAM_SEL_MSK;
	FLASH_WRITE32(tmpVal, data->reg_copy + SF_CTRL_0_OFFSET);
#endif

	/* trigger command */
	tmpVal = FLASH_READ32(bank_offset + 0);
	tmpVal |= SF_CTRL_SF_IF_0_TRIG_MSK;
	FLASH_WRITE32(tmpVal, bank_offset + 0);

	if (flash_bflb_busy_wait(data)) {
#if defined(CONFIG_SOC_SERIES_BL70X) || defined(CONFIG_SOC_SERIES_BL60X)
		tmpVal = FLASH_READ32(data->reg_copy + SF_CTRL_0_OFFSET);
		tmpVal &= ~SF_CTRL_SF_CLK_SAHB_SRAM_SEL_MSK;
		FLASH_WRITE32(tmpVal, data->reg_copy + SF_CTRL_0_OFFSET);
#endif
		return -EBUSY;
	}

#if defined(CONFIG_SOC_SERIES_BL70X) || defined(CONFIG_SOC_SERIES_BL60X)
	tmpVal = FLASH_READ32(data->reg_copy + SF_CTRL_0_OFFSET);
	tmpVal &= ~SF_CTRL_SF_CLK_SAHB_SRAM_SEL_MSK;
	FLASH_WRITE32(tmpVal, data->reg_copy + SF_CTRL_0_OFFSET);
#endif

	return 0;
}

static int flash_bflb_flash_read_register(struct flash_bflb_data *data, uint8_t index, uint8_t *out,
uint8_t len)
{
	bflb_flash_command read_reg = {0};

	read_reg.cmd_buf[0] = (data->flash_cfg.read_reg_cmd[index]) << 24;
	read_reg.nb_data = len;
	flash_bflb_send_command(data, &read_reg);

	if (flash_bflb_busy_wait(data)) {
		return -EBUSY;
	}

	flash_bflb_xip_memcpy((uint8_t *)SF_CTRL_BUF_BASE, out, len);
	return 0;
}


static int flash_bflb_flash_write_register(struct flash_bflb_data *data, uint8_t index, uint8_t *in,
uint8_t len)
{
	bflb_flash_command write_reg = {0};

	flash_bflb_xip_memcpy(in, (uint8_t *)SF_CTRL_BUF_BASE, len);

	write_reg.cmd_buf[0] = (data->flash_cfg.write_reg_cmd[index]) << 24;
	write_reg.nb_data = len;
	write_reg.rw = 1;
	flash_bflb_send_command(data, &write_reg);


	return flash_bflb_send_command(data, &write_reg);
}

static int flash_bflb_flash_disable_continuous_read(struct flash_bflb_data *data)
{
	bflb_flash_command disable_continuous_read = {0};

	disable_continuous_read.addr_size = data->flash_cfg.reset_c_read_cmd_size;
	disable_continuous_read.cmd_buf[0] = data->flash_cfg.reset_c_read_cmd << 24 |
	data->flash_cfg.reset_c_read_cmd << 16 | data->flash_cfg.reset_c_read_cmd << 8 |
	data->flash_cfg.reset_c_read_cmd;
	return flash_bflb_send_command(data, &disable_continuous_read);
}

static int flash_bflb_flash_disable_burst(struct flash_bflb_data *data)
{
	uint32_t tmpVal = 0;
	bflb_flash_command disable_burstwrap = {0};

	disable_burstwrap.dummy_clks = data->flash_cfg.de_burst_wrap_cmd_dmy_clk;
	disable_burstwrap.spi_mode =
flash_bflb_admode_to_spimode(data->flash_cfg.de_burst_wrap_data_mode,
data->flash_cfg.de_burst_wrap_data_mode);
	disable_burstwrap.cmd_buf[0] = data->flash_cfg.de_burst_wrap_cmd << 24;
	disable_burstwrap.nb_data = 1;
	disable_burstwrap.rw = 1;
	tmpVal = data->flash_cfg.de_burst_wrap_data;
	*(uint32_t *)SF_CTRL_BUF_BASE = tmpVal;
	return flash_bflb_send_command(data, &disable_burstwrap);
}

static int flash_bflb_flash_enable_burst(struct flash_bflb_data *data)
{
	uint32_t tmpVal = 0;
	bflb_flash_command enable_burstwrap = {0};

	enable_burstwrap.dummy_clks = data->flash_cfg.burst_wrap_cmd_dmy_clk;
	enable_burstwrap.spi_mode =
flash_bflb_admode_to_spimode(data->flash_cfg.burst_wrap_data_mode,
data->flash_cfg.burst_wrap_data_mode);
	enable_burstwrap.cmd_buf[0] = data->flash_cfg.burst_wrap_cmd << 24;
	enable_burstwrap.nb_data = 1;
	enable_burstwrap.rw = 1;
	tmpVal = data->flash_cfg.burst_wrap_data;
	*(uint32_t *)SF_CTRL_BUF_BASE = tmpVal;
	return flash_bflb_send_command(data, &enable_burstwrap);
}


static int flash_bflb_enable_writable(struct flash_bflb_data *data)
{
	bflb_flash_command write_enable = {0};
	int ret = 0;
	uint32_t write_reg = 0;

	write_enable.cmd_buf[0] = (data->flash_cfg.write_enable_cmd) << 24;
	ret = flash_bflb_send_command(data, &write_enable);
	if (ret != 0) {
		return ret;
	}

	/* check writable */
	ret = flash_bflb_flash_read_register(data, data->flash_cfg.wr_enable_index,
(uint8_t *)&write_reg, data->flash_cfg.wr_enable_read_reg_len);
	if (ret != 0) {
		return ret;
	}

	if ((write_reg & (1 << data->flash_cfg.wr_enable_bit)) != 0) {
		return 0;
	}
	return -EINVAL;
}


/* TODO: consider removing never used options (qe_data never being not 0, ever)
 * /!\ UNTESTED (no relevant hardware)
 */
static int flash_bflb_enable_qspi(struct flash_bflb_data *data)
{
	int ret = 0;
	uint32_t tmpVal = 0;
	/* qe = quad enable */

	/* writable command also enables writing to configuration registers, not just data*/
	ret = flash_bflb_enable_writable(data);
	if (ret != 0) {
		return ret;
	}

	if (data->flash_cfg.qe_read_reg_len == 0) {
		/* likely to write nothing (len = 0) */
		flash_bflb_flash_write_register(data, data->flash_cfg.qe_index, (uint8_t *)&tmpVal,
		data->flash_cfg.qe_write_reg_len);
		return 0;
	}

	/* get quad enable register value */
	ret = flash_bflb_flash_read_register(data, data->flash_cfg.qe_index, (uint8_t *)&tmpVal,
	data->flash_cfg.qe_read_reg_len);
	if (ret != 0) {
		return ret;
	}

	/* qe is a bit */
	if (data->flash_cfg.qe_data == 0) {
		/* qe is already enable*/
		if ((tmpVal & (1 << data->flash_cfg.qe_bit)) != 0) {
			return 0;
		}
	/* qe is a specific value, not encountered in available flash chip configs  */
	} else {
		if (((tmpVal >> (data->flash_cfg.qe_bit & 0x08)) & 0xff) ==
		data->flash_cfg.qe_data) {
			return 0;
		}
	}

	/* all status registers must be read and written */
	if (data->flash_cfg.qe_write_reg_len != 1) {
		ret = flash_bflb_flash_read_register(data, 0, (uint8_t *)&tmpVal, 1);
		if (ret != 0) {
			return ret;
		}
		ret = flash_bflb_flash_read_register(data, 1, (uint8_t *)&tmpVal + 1, 1);
		if (ret != 0) {
			return ret;
		}

		if (data->flash_cfg.qe_data == 0) {
			tmpVal |= (1 << (data->flash_cfg.qe_bit + 8 * data->flash_cfg.qe_index));
		} else {
			tmpVal = tmpVal & (~(0xff << (8 * data->flash_cfg.qe_index)));
			tmpVal |= (data->flash_cfg.qe_data << (8 * data->flash_cfg.qe_index));
		}

	/* we only need to read and write the appropriate register (usually the second one) */
	} else {
		if (data->flash_cfg.qe_data == 0) {
			tmpVal |= (1 << (data->flash_cfg.qe_bit % 8));
		} else {
			tmpVal = data->flash_cfg.qe_data;
		}
	}

	flash_bflb_flash_write_register(data, data->flash_cfg.qe_index, (uint8_t *)&tmpVal,
data->flash_cfg.qe_write_reg_len);
	flash_bflb_flash_read_register(data, data->flash_cfg.qe_index, (uint8_t *)&tmpVal,
data->flash_cfg.qe_write_reg_len);

	/* check Quad is Enabled */
	if (data->flash_cfg.qe_data == 0) {
		if ((tmpVal & (1 << data->flash_cfg.qe_bit)) != 0) {
			return 0;
		}
	} else {
		if (((tmpVal >> (data->flash_cfg.qe_bit & 0x08)) & 0xff) ==
		data->flash_cfg.qe_data) {
			return 0;
		}
	}
	return -EINVAL;
}

static uint32_t flash_bflb_get_offset(uint32_t base_reg)
{
	uint32_t tmpVal = 0;

	tmpVal = FLASH_READ32(base_reg + SF_CTRL_SF_ID0_OFFSET_OFFSET);
	tmpVal &= SF_CTRL_SF_ID0_OFFSET_MSK;
	tmpVal = tmpVal >> SF_CTRL_SF_ID0_OFFSET_POS;
	return tmpVal;
}

static void flash_bflb_set_offset(uint32_t base_reg, uint32_t offset)
{
	uint32_t tmpVal = 0;

	tmpVal = FLASH_READ32(base_reg + SF_CTRL_SF_ID0_OFFSET_OFFSET);
	tmpVal &= ~SF_CTRL_SF_ID0_OFFSET_MSK;
	tmpVal |= offset << SF_CTRL_SF_ID0_OFFSET_POS;
	*((uint32_t *)(base_reg + SF_CTRL_SF_ID0_OFFSET_OFFSET)) = tmpVal;
}


static int flash_bflb_save_xip_state(const struct device *dev)
{
	const struct flash_bflb_config *cfg = dev->config;
	struct flash_bflb_data *data = dev->data;
	int ret = 0;

	data->reg_copy = cfg->reg;

	/* bus to system AHB */
	ret = flash_bflb_set_bus(data, 0);
	if (ret != 0) {
		return ret;
	}

	/* command to disable continuous read */
	ret = flash_bflb_flash_disable_continuous_read(data);
	if (ret != 0) {
		return ret;
	}

	/* disable burst with wrap*/
	ret = flash_bflb_flash_disable_burst(data);
	if (ret != 0) {
		return ret;
	}

	/* enable quad previous command could've disabled it
	 * 0: io 1: do 2: qo 3: dio 4: qio
	 */
	if ((data->flash_cfg.io_mode & 0xf) == 2 || (data->flash_cfg.io_mode & 0xf) == 4) {
		ret = flash_bflb_enable_qspi(data);
		if (ret != 0) {
			return ret;
		}
	}

	/* disable burst with wrap*/
	ret = flash_bflb_flash_disable_burst(data);
	if (ret != 0) {
		return ret;
	}

	data->last_flash_offset = flash_bflb_get_offset(data->reg_copy);
	flash_bflb_cache_invalidate();
	flash_bflb_set_offset(data->reg_copy, 0);

	return 0;
}

static int flash_bflb_xip_init(struct flash_bflb_data *data)
{
	bflb_flash_command xip_cmd = {0};
	bool no_command = false;
	int ret = 0;

	/* bus to instruction AHB */
	ret = flash_bflb_set_bus(data, 1);
	if (ret != 0) {
		return ret;
	}

	xip_cmd.spi_mode = data->flash_cfg.io_mode & 0xf;

	switch (data->flash_cfg.io_mode & 0xf) {
	default:
	case 0:
		xip_cmd.cmd_buf[0] = data->flash_cfg.fast_read_cmd << 24;
		xip_cmd.dummy_clks = data->flash_cfg.fr_dmy_clk;
	break;
	case 1:
		xip_cmd.cmd_buf[0] = data->flash_cfg.fast_read_do_cmd << 24;
		xip_cmd.dummy_clks = data->flash_cfg.fr_do_dmy_clk;
	break;
	case 2:
		xip_cmd.cmd_buf[0] = data->flash_cfg.fast_read_qo_cmd << 24;
		xip_cmd.dummy_clks = data->flash_cfg.fr_qo_dmy_clk;
	break;
	case 3:
		xip_cmd.cmd_buf[0] = data->flash_cfg.fast_read_dio_cmd << 24;
		xip_cmd.dummy_clks = data->flash_cfg.fr_dio_dmy_clk;
	break;
	case 4:
		xip_cmd.cmd_buf[0] = data->flash_cfg.fast_read_qio_cmd << 24;
		xip_cmd.dummy_clks = data->flash_cfg.fr_qio_dmy_clk;
	break;
	}
	xip_cmd.addr_size = 3;

	/* continuous read for qo and qio */
	if ((data->flash_cfg.io_mode & 0xf) == 2 || (data->flash_cfg.io_mode & 0xf) == 4) {

		if ((data->flash_cfg.c_read_support & 0x02) == 0) {
			if ((data->flash_cfg.c_read_support & 0x01) == 0) {
				/* "Not support cont read,but we still need set read mode(winbond
				 * 80dv)"
				 */
				xip_cmd.cmd_buf[1] = data->flash_cfg.c_read_mode << 24;
			} else {
				no_command = true;
				xip_cmd.cmd_buf[0] =  data->flash_cfg.c_read_mode;
			}
			xip_cmd.addr_size = 4;
		}
	}
	xip_cmd.nb_data = 32;

	flash_bflb_set_command_iahb(data, &xip_cmd, !no_command);

	return 0;

}

static void show_it_to_me(int value)
{
	value++;
}

static bool flash_bflb_flash_busy_wait(struct flash_bflb_data *data)
{
	uint8_t tmpBus = 0xFF;
	uint32_t counter = 0;

	while ((tmpBus & (1 << data->flash_cfg.busy_bit)) != 0 && counter <
	BFLB_FLASH_CHIP_BUSY_TIMEOUT_MS * 20000) {
		flash_bflb_flash_read_register(data, data->flash_cfg.busy_index, &tmpBus,
data->flash_cfg.busy_read_reg_len);
		show_it_to_me(tmpBus);
		__asm__ volatile (".rept 10 ; nop ; .endr");
		counter++;
	}


	if ((tmpBus & (1 << data->flash_cfg.busy_bit)) != 0) {
		return true;
	}
	return false;
}


static int flash_bflb_restore_xip_state(struct flash_bflb_data *data)
{
	int ret = 0;

	flash_bflb_cache_invalidate();
	flash_bflb_set_offset(data->reg_copy, data->last_flash_offset);

	/* reenable burt read */
	if ((data->flash_cfg.io_mode & 0x10) != 0) {
		if ((data->flash_cfg.io_mode & 0xf) == 2 || (data->flash_cfg.io_mode & 0xf) == 4) {
			ret = flash_bflb_flash_enable_burst(data);
			if (ret != 0) {
				return ret;
			}
		}
	}

	ret = flash_bflb_xip_init(data);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

#if defined(CONFIG_SOC_FLASH_BFLB_DIRECT_ACCESS)

static int flash_bflb_read_sahb_do(struct flash_bflb_data *data, off_t address, void *buffer,
				   size_t length)
{
	int ret = 0;
	bflb_flash_command read_cmd = {0};
	int i = 0, cur_len = 0;

	read_cmd.spi_mode = data->flash_cfg.io_mode & 0xf;

	switch (data->flash_cfg.io_mode & 0xf) {
	default:
	case 0:
		read_cmd.cmd_buf[0] = data->flash_cfg.fast_read_cmd << 24;
		read_cmd.dummy_clks = data->flash_cfg.fr_dmy_clk;
	break;
	case 1:
		read_cmd.cmd_buf[0] = data->flash_cfg.fast_read_do_cmd << 24;
		read_cmd.dummy_clks = data->flash_cfg.fr_do_dmy_clk;
	break;
	case 2:
		read_cmd.cmd_buf[0] = data->flash_cfg.fast_read_qo_cmd << 24;
		read_cmd.dummy_clks = data->flash_cfg.fr_qo_dmy_clk;
	break;
	case 3:
		read_cmd.cmd_buf[0] = data->flash_cfg.fast_read_dio_cmd << 24;
		read_cmd.dummy_clks = data->flash_cfg.fr_dio_dmy_clk;
	break;
	case 4:
		read_cmd.cmd_buf[0] = data->flash_cfg.fast_read_qio_cmd << 24;
		read_cmd.dummy_clks = data->flash_cfg.fr_qio_dmy_clk;
	break;
	}
	read_cmd.addr_size = 3;

	/* continuous read for qo and qio */
	if ((data->flash_cfg.io_mode & 0xf) == 2 || (data->flash_cfg.io_mode & 0xf) == 4) {

		if ((data->flash_cfg.c_read_support & 0x02) == 0) {
			if ((data->flash_cfg.c_read_support & 0x01) == 0) {
				/* "Not support cont read,but we still need set read mode(winbond
				 * 80dv)"
				 */
				read_cmd.cmd_buf[1] = data->flash_cfg.c_read_mode << 24;
			} else {
				read_cmd.cmd_buf[1] =  data->flash_cfg.c_read_mode << 24;
			}
			read_cmd.addr_size = 4;
		}
	}


	i = 0;
	while (i < length) {

		cur_len = data->flash_cfg.page_size - ((address + i) % data->flash_cfg.page_size);

		if (cur_len > length - i) {
			cur_len = length - i;
		}

		read_cmd.cmd_buf[0] &= ~0xFFFFFF;
		read_cmd.cmd_buf[0] |= (address + i);
		read_cmd.nb_data = cur_len;

		flash_bflb_send_command(data, &read_cmd);

		flash_bflb_xip_memcpy((uint8_t *)SF_CTRL_BUF_BASE, (uint8_t *)(buffer) + i,
		cur_len);

		i += cur_len;

		ret = flash_bflb_busy_wait(data);
		if (ret != 0) {
			return ret;
		}
		ret = flash_bflb_flash_busy_wait(data);
		if (ret != 0) {
			return ret;
		}
	}
	return 0;
}

/* copies flash data using direct access */
static int flash_bflb_read(const struct device *dev, off_t address, void *buffer, size_t
length)
{
	struct flash_bflb_data *data = dev->data;
	unsigned int	locker;
	int ret = 0;


	if (flash_bflb_guard(&flash_bflb_read)) {
		return -ENOTSUP;
	}

	/* interrupting would break, likely to access XIP*/
	locker = irq_lock();

	ret = flash_bflb_save_xip_state(dev);
	if (ret != 0) {
		irq_unlock(locker);
		return ret;
	}


	flash_bflb_read_sahb_do(data, address, buffer, length);

	ret = flash_bflb_restore_xip_state(data);
	irq_unlock(locker);
	return ret;
}

#else
/* copies flash data using XIP access */
static int flash_bflb_read(const struct device *dev, off_t address, void *buffer, size_t length)
{
	struct flash_bflb_data *data = dev->data;
	uint32_t	img_offset;
	unsigned int	locker;

	if (flash_bflb_guard(&flash_bflb_read)) {
		return -ENOTSUP;
	}

	/* interrupting would break, likely to access XIP*/
	locker = irq_lock();
	/* get XIP offset / where code really is in flash, usually 0x2000 */
	img_offset = flash_bflb_get_offset(data->reg_copy);
	/* need set offset to 0 to access? */
	if (address < img_offset) {

		flash_bflb_cache_invalidate();

		/* set offset to 0 to access first (likely)0x2000 of flash */
		flash_bflb_set_offset(data->reg_copy, 0);

		/* copy data we need */
		flash_bflb_xip_memcpy((uint8_t *)(address + BL_XIP_BASE), (uint8_t *)buffer, length);

		flash_bflb_cache_invalidate();

		flash_bflb_set_offset(data->reg_copy, img_offset);
	} else {
		/* copy data we need */
		flash_bflb_xip_memcpy((uint8_t *)(address + BL_XIP_BASE - img_offset),
		(uint8_t *)buffer, length);
	}

	/* done with interrupt breaking stuffs */
	irq_unlock(locker);

	return 0;
}
#endif

static int flash_bflb_write(const struct device *dev,
			     off_t address,
			     const void *buffer,
			     size_t length)
{
	struct flash_bflb_data *data = dev->data;
	uint32_t	tmpVal = 0;
	unsigned int	locker;
	int		ret = 0;
	uint32_t	cur_len = 0, i = 0;
	bflb_flash_command write_cmd = {0};

	if (flash_bflb_guard(&flash_bflb_write)) {
		return -ENOTSUP;
	}

	/* interrupting would break, likely to access XIP*/
	locker = irq_lock();

	ret = flash_bflb_save_xip_state(dev);
	if (ret != 0) {
		irq_unlock(locker);
		return ret;
	}

	flash_bflb_flash_read_register(data, 0, (uint8_t *)(&tmpVal), 1);
	if ((tmpVal & 0x1C) != 0) {
		return -EINVAL;
	}

	if ((data->flash_cfg.io_mode & 0xf) == 0
	|| (data->flash_cfg.io_mode & 0xf) == 1
	|| (data->flash_cfg.io_mode & 0xf) == 3) {
		write_cmd.cmd_buf[0] = data->flash_cfg.page_program_cmd << 24;
	/* quad mode */
	} else {
		write_cmd.cmd_buf[0] = data->flash_cfg.qpage_program_cmd << 24;
		write_cmd.spi_mode =
flash_bflb_admode_to_spimode(data->flash_cfg.qpp_addr_mode, 2);
	}
	write_cmd.rw = 1;
	write_cmd.addr_size = 3;

	i = 0;
	while (i < length) {
		/* Write enable is needed for every write */
		ret = flash_bflb_enable_writable(data);
		if (ret != 0) {
			flash_bflb_restore_xip_state(data);
			irq_unlock(locker);
			return ret;
		}

		/* Get current position within page size,
		 * this assumes page_size <= CTRL_BUF_SIZE (512?)
		 */
		cur_len = data->flash_cfg.page_size - ((address + i) % data->flash_cfg.page_size);

		if (cur_len > length - i) {
			cur_len = length - i;
		}

		flash_bflb_xip_memcpy((uint8_t *)(buffer) +  i, (uint8_t *)SF_CTRL_BUF_BASE,
cur_len);
		write_cmd.cmd_buf[0] &= ~0xFFFFFF;
		write_cmd.cmd_buf[0] |= (address + i);
		write_cmd.nb_data = cur_len;

		flash_bflb_send_command(data, &write_cmd);

		i += cur_len;


		flash_bflb_busy_wait(data);
		flash_bflb_flash_busy_wait(data);

	}


	ret = flash_bflb_restore_xip_state(data);
	irq_unlock(locker);
	return ret;
}

static int flash_bflb_erase(const struct device *dev, off_t start, size_t len)
{
	struct flash_bflb_data *data = dev->data;
	unsigned int	locker;
	int		ret = 0;
	bflb_flash_command erase_cmd = {0};

	if (flash_bflb_guard(&flash_bflb_write)) {
		return -ENOTSUP;
	}

	if ((len % ERASE_SIZE) != 0) {
		return -EINVAL;
	}

	if ((start % ERASE_SIZE) != 0) {
		return -EINVAL;
	}
	/* interrupting would break, likely to access XIP*/
	locker = irq_lock();

	ret = flash_bflb_save_xip_state(dev);
	if (ret != 0) {
		irq_unlock(locker);
		return ret;
	}

	erase_cmd.rw = 0;
	erase_cmd.addr_size = 3;

	for (uint32_t i = start / ERASE_SIZE; i < ((len / ERASE_SIZE) + start / ERASE_SIZE); i++) {
		/* Write enable is needed for every write */
		ret = flash_bflb_enable_writable(data);
		if (ret != 0) {
			flash_bflb_restore_xip_state(data);
			irq_unlock(locker);
			return ret;
		}

		erase_cmd.cmd_buf[0] = data->flash_cfg.sector_erase_cmd << 24;
		erase_cmd.cmd_buf[0] |= (i * data->flash_cfg.sector_size * 1024);

		flash_bflb_send_command(data, &erase_cmd);
		ret = data->flash_cfg.time_e_sector * 3;
		flash_bflb_busy_wait(data);
		flash_bflb_flash_busy_wait(data);
	}

	ret = flash_bflb_restore_xip_state(data);
	irq_unlock(locker);
	return ret;
}


#if CONFIG_FLASH_PAGE_LAYOUT
static struct flash_pages_layout flash_bflb_pages_layout = {
	.pages_count = TOTAL_SIZE / ERASE_SIZE,
	.pages_size = ERASE_SIZE,
};

void flash_bflb_page_layout(const struct device *dev,
			     const struct flash_pages_layout **layout,
			     size_t *layout_size)
{
	*layout = &flash_bflb_pages_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters flash_bflb_parameters = {
	.write_block_size = WRITE_SIZE,
	.erase_value = ERASE_VALUE,
};

static const struct flash_parameters *flash_bflb_get_parameters(const struct device *dev)
{
	return &flash_bflb_parameters;
}

static void flash_bflb_isr(const struct device *dev)
{
}

static struct flash_driver_api flash_bflb_api = {
	.read = flash_bflb_read,
	.write = flash_bflb_write,
	.erase = flash_bflb_erase,
	.get_parameters = flash_bflb_get_parameters,
	.page_layout = flash_bflb_page_layout,
};



/* from SDK because the zephyr CRC are no good for the bflb flash crc, this is supposedly a
 * implementation of ZIP crc32
 */
static uint32_t bflb_soft_crc32(uint32_t initial, void *in, uint32_t len)
{
	uint8_t i;
	uint32_t crc = ~initial;
	uint8_t *data = (uint8_t *)in;

	while (len--) {
		crc ^= *data++;
		for (i = 0; i < 8; ++i) {
			if (crc & 1) {
				/* 0xEDB88320 = reverse 0x04C11DB7 */
				crc = (crc >> 1) ^ 0xEDB88320;
			} else {
				crc = (crc >> 1);
			}
		}
	}
	return ~crc;
}

#if defined(CONFIG_SOC_SERIES_BL70X) || defined(CONFIG_SOC_SERIES_BL60X)

/* /!\ this function cannot run from XIP! */
static int flash_bflb_config_init_e24(const struct device *dev)
{
	struct flash_bflb_data *data = dev->data;
	const struct flash_bflb_config *cfg = dev->config;
	uint32_t	tmpVal = 0;
	uint32_t	img_offset;
	unsigned int	locker;
	uint8_t		buff[sizeof(bflb_flash_cfg) + 8] = {0};
	char		*buffchar = buff;

	if (flash_bflb_guard(&flash_bflb_config_init_e24)) {
		return -ENOTSUP;
	}

	/* copy cfg reg to memory as cfg will not be in it and inaccessible */
	data->reg_copy = cfg->reg;

	/* get flash config using xip access */

	/* interrupting would break, likely to access XIP*/
	locker = irq_lock();
	/* get XIP offset / where code really is in flash, usually 0x2000 */
	img_offset = flash_bflb_get_offset(data->reg_copy);

	flash_bflb_cache_invalidate();

	/* set offset to 0 to access first (likely)0x2000 of flash */
	flash_bflb_set_offset(data->reg_copy, 0);

	/* copy data we need */
	flash_bflb_xip_memcpy((uint8_t *)(8 + BL_XIP_BASE), buff, sizeof(bflb_flash_cfg) + 8);

	flash_bflb_cache_invalidate();

	flash_bflb_set_offset(data->reg_copy, img_offset);

	/* done with interrupt breaking stuffs */
	irq_unlock(locker);

	/* magic */
	if (!(buffchar[0] == 'F' && buffchar[1] == 'C' && buffchar[2] == 'F'
	&& buffchar[3] == 'G')) {
		LOG_ERR("Flash data magic is incorrect");
		return -EINVAL;
	}

	tmpVal = *((uint32_t *)(buff + 4 + sizeof(bflb_flash_cfg)));
	img_offset = bflb_soft_crc32(0, (uint8_t *)(buff + 4), sizeof(bflb_flash_cfg));
	if (tmpVal != img_offset) {
		LOG_ERR("Flash data crc is incorrect %d vs %d", tmpVal, img_offset);
		return -EINVAL;
	}
	flash_bflb_xip_memcpy(buff + 4, (uint8_t *)&(data->flash_cfg), sizeof(bflb_flash_cfg));

	return 0;
}
#elif defined(CONFIG_SOC_SERIES_BL61X)

static uint32_t flash_bflb_get_jedecid_glb(void)
{
    uint32_t tmpVal = 0;

    tmpVal = sys_read32(GLB_BASE + GLB_HW_RSV1_OFFSET);
    if ((tmpVal & 0x7F000000) == 0x5A000000) {
        return (tmpVal & 0x00FFFFFF);
    }
    return 0x00000000;
}

/* /!\ this function cannot run from XIP! */
static int flash_bflb_config_init_e907(const struct device *dev)
{
	struct flash_bflb_data *data = dev->data;
	const struct flash_bflb_config *cfg = dev->config;
	uint32_t	tmpVal = 0;
	uint32_t	img_offset;
	unsigned int	locker;
	uint8_t		buff[sizeof(bflb_flash_cfg) + 8] = {0};
	char		*buffchar = buff;

	if (flash_bflb_guard(&flash_bflb_config_init_e907)) {
		return -ENOTSUP;
	}

	/* copy cfg reg to memory as cfg will not be in it and inaccessible */
	data->reg_copy = cfg->reg;

	/* get flash config using xip access */

	/* interrupting would break, likely to access XIP*/
	locker = irq_lock();
	/* get XIP offset / where code really is in flash, usually 0x2000 */
	img_offset = flash_bflb_get_offset(data->reg_copy);

	flash_bflb_cache_invalidate();

	/* set offset to 0 to access first (likely)0x2000 of flash */
	flash_bflb_set_offset(data->reg_copy, 0);

	/* copy data we need */
	flash_bflb_xip_memcpy((uint8_t *)(8 + BL_XIP_BASE), buff, sizeof(bflb_flash_cfg) + 8);

	flash_bflb_cache_invalidate();

	flash_bflb_set_offset(data->reg_copy, img_offset);

	/* done with interrupt breaking stuffs */
	irq_unlock(locker);

	/* magic */
	if (!(buffchar[0] == 'F' && buffchar[1] == 'C' && buffchar[2] == 'F'
	&& buffchar[3] == 'G')) {
		LOG_ERR("Flash data magic is incorrect");
		return -EINVAL;
	}

	tmpVal = *((uint32_t *)(buff + 4 + sizeof(bflb_flash_cfg)));
	img_offset = bflb_soft_crc32(0, (uint8_t *)(buff + 4), sizeof(bflb_flash_cfg));
	if (tmpVal != img_offset) {
		LOG_ERR("Flash data crc is incorrect %d vs %d", tmpVal, img_offset);
		return -EINVAL;
	}
	flash_bflb_xip_memcpy(buff + 4, (uint8_t *)&(data->flash_cfg), sizeof(bflb_flash_cfg));

	return 0;
}
#endif

/* grabs the id with byte order inverted (LSB to MSB) */
static uint32_t flash_bflb_get_jedecid_live(struct flash_bflb_data
*data)
{
	bflb_flash_command get_jedecid = {0};
	uint32_t ret = 0;

	get_jedecid.dummy_clks = data->flash_cfg.jedec_id_cmd_dmy_clk;
	get_jedecid.cmd_buf[0] = data->flash_cfg.jedec_id_cmd << 24;
	get_jedecid.nb_data = 3;
	ret = flash_bflb_send_command(data, &get_jedecid);
	if (ret != 0) {
		return 0;
	}
	ret = FLASH_READ32(SF_CTRL_BUF_BASE);

	return ret;
}

static int flash_bflb_init(const struct device *dev)
{
	const struct flash_bflb_config *cfg = dev->config;
	struct flash_bflb_data *data = dev->data;
	unsigned int	locker;
	int ret = 0;

#if defined(CONFIG_SOC_SERIES_BL70X) || defined(CONFIG_SOC_SERIES_BL60X)
	flash_bflb_config_init_e24(dev);
#else
	flash_bflb_config_init_e907(dev);
#endif

	ret = flash_bflb_save_xip_state(dev);
	if (ret != 0) {
		return ret;
	}
	locker = irq_lock();
	/* TODO: AES flash support goes here */
	data->jedec_id = flash_bflb_get_jedecid_live(data);

	/* operations done here in bflb driver but not here:
	 * - reenable qspi (already done in flash_bflb_save_xip_state in bflb driver too)
	 * - reenable flash-side burstwrap (already done in restore state, possibly need to be done
	 * before l1c wrap side)
	 */
	if ((data->flash_cfg.io_mode & 0x10) == 0) {
		flash_bflb_l1c_wrap(true);
	} else {
		flash_bflb_l1c_wrap(false);
	}

	ret = flash_bflb_restore_xip_state(data);
	if (ret != 0) {
		return ret;
	}

	irq_unlock(locker);

	cfg->irq_config_func(dev);
	return 0;
}


#define FLASH_BFLB_DEVICE(n)						\
	static void flash_bflb_irq_config_##n(const struct device *dev)	\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority),			\
			    flash_bflb_isr,				\
			    DEVICE_DT_INST_GET(n), 0);			\
		irq_enable(DT_INST_IRQN(n));				\
	}								\
	static const struct flash_bflb_config flash_bflb_config_##n = {	\
		.reg = DT_INST_REG_ADDR_BY_IDX(n, 0),			\
		.irq_config_func = &flash_bflb_irq_config_##n,		\
	};								\
	static struct flash_bflb_data flash_bflb_data_##n;		\
	DEVICE_DT_INST_DEFINE(n, flash_bflb_init, NULL,			\
			      &flash_bflb_data_##n,			\
			      &flash_bflb_config_##n, POST_KERNEL,	\
			      CONFIG_FLASH_INIT_PRIORITY,		\
			      &flash_bflb_api);

DT_INST_FOREACH_STATUS_OKAY(FLASH_BFLB_DEVICE)
