/*
 * Copyright (c) 2024-2026 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The bouffalolab serial flash controller provides 1 (BL60x only), or 2 banks of SPI controls.
 * There are two interactions modes: Memory-mapped (XIP) and direct, which are mutually exclusive.
 * Memory-mapping is achieved by providing read (and write) commands to the SF at the area
 * corresponding to the chosen bank, which will then automatically talk to the device.
 * The control is done via the Instruction bus which runs directly between the CPU ('s cache)
 * and the SF controller.
 * Direct mode enables the CPU to directly control the SF's interactions with the device by passing
 * commands and writing/reading data directly. The control is done via the System bus (normal bus
 * used to talk to peripherals).
 * Direct mode is achieved in different ways:
 * On E24 cpus SoCs (BL60x, BL70x/L), only one System bus interace is available, and so
 * memory-mapped access must be disabled whenever manual control is required. Switching is done via
 * a bank selection flag.
 * On e907 CPUs (BL61x/CL, BL808...), two system buses are available. The second bus is selected
 * by enabling it and turning on its selection flag, then enabling system bus mode on the interface.
 *
 * Devices are available via pads ('SF' registers), which indicate the mapping between pins and
 * interfaces. Some provide the ability to swap pins.
 *
 * As a summary:
 * - One Instruction Bus (IAHB) interface which handles reading both banks from memory access.
 * - One or Two System Bus (SAHB) interfaces which handles reading both or one bank with the CPU
 *
 *
 * Burst wrap:
 * - Defines the size of a continuous read command's wrap, eg the minimum read size
 * to fill a cache line that will be automatically sent when a continuous read is done
 * - The setting itself is passed on/off to the same command.
 *
 * Continuous read: Read data from device without sending commands,
 * saving 8 cycles (1 in SF dummy cycles terms) of clock each read and is ideal for XIP. To enable,
 * the controller must first send the needed read command, then immediatly pass into IBus the
 * continuous read flags as the set XIP command and tell the controller to not send out commands.
 *
 * Writable and Quad enable:
 * There is large variations in the registers format and the ways to interact with it.
 *
 */

#define DT_DRV_COMPAT bflb_sf_controller

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/arch/common/sys_io.h>
#include <zephyr/cache.h>
#include <zephyr/sys/byteorder.h>

#include <soc.h>
#include <bflb_soc.h>
#include <glb_reg.h>
#include <sf_ctrl_reg.h>
#include <common_defines.h>
#include <hbn_reg.h>
#include <zephyr/drivers/clock_control/clock_control_bflb_common.h>

#include "spi_nor.h"

#if defined(CONFIG_SOC_SERIES_BL60X) || defined(CONFIG_SOC_SERIES_BL70X) || \
	defined(CONFIG_SOC_SERIES_BL70XL)
#include <l1c_reg.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_bflb, CONFIG_FLASH_LOG_LEVEL);

#define ERASE_VALUE	0xFF

#ifdef CONFIG_SOC_SERIES_BL60X
#define BFLB_XIP_BASE_BANK1	BL602_FLASH_XIP_BASE
#define BFLB_XIP_END_BANK1	BL602_FLASH_XIP_END
#define BFLB_XIP_BASE_BANK2	-1
#define BFLB_XIP_END_BANK2	-1
#elif defined(CONFIG_SOC_SERIES_BL70X)
#define BFLB_XIP_BASE_BANK1	BL702_FLASH_XIP_BASE
#define BFLB_XIP_END_BANK1	BL702_FLASH_XIP_END
#define BFLB_XIP_BASE_BANK2	BL702_PSRAM_XIP_BASE
#define BFLB_XIP_END_BANK2	BL702_PSRAM_XIP_END
#elif defined(CONFIG_SOC_SERIES_BL70XL)
#define BFLB_XIP_BASE_BANK1	BL70XL_FLASH_XIP_BASE
#define BFLB_XIP_END_BANK1	BL70XL_FLASH_XIP_END
#define BFLB_XIP_BASE_BANK2	BL702L_PSRAM_XIP_BASE
#define BFLB_XIP_END_BANK2	BL702L_PSRAM_XIP_END
#elif defined(CONFIG_SOC_SERIES_BL61X)
#define BFLB_XIP_BASE_BANK1	BL616_FLASH_XIP_BASE
#define BFLB_XIP_END_BANK1	BL616_FLASH_XIP_END
#define BFLB_XIP_BASE_BANK2	BL616_FLASH2_XIP_BUSREMAP_BASE
#define BFLB_XIP_END_BANK2	BL616_FLASH2_XIP_BUSREMAP_END
#endif

#define BFLB_FLASH_CONTROLLER_BUSY_TIMEOUT	200
#define BFLB_FLASH_CHIP_BUSY_TIMEOUT		5000

#define BFLB_FLASH_FLASH_BLOCK_PROTECT_MSK	0x1C

#define BFLB_FLASH_SF_BUF_SIZE			256

#define BFLB_FLASH_ADDR_SIZE			3
#define BFLB_FLASH_ADDR_SIZE_32B		4
#define BFLB_FLASH_ADDR_SIZE_CONTREAD_ADD	1

#define BFLB_FLASH_AUTO_READ_DEFAULT		SPI_NOR_CMD_READ_FAST
#define BFLB_FLASH_AUTO_READ_DMYCY_DEFAULT	1

#define FLASH_READ32(address)		(*((volatile uint32_t *)(address)))
#define FLASH_WRITE32(value, address)	(*((volatile uint32_t *)(address))) = value;

#define BFLB_FLASH_MAGIC_1 "BFNP"
#define BFLB_FLASH_MAGIC_2 "FCFG"

struct bflb_flash_magic_1 {
	char magic[4];
	uint32_t revision;
} __packed;

struct bflb_flash_magic_2 {
	char magic[4];
} __packed;

/* Raw flash configuration data structure */
struct bflb_header_flash_cfg {
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
#if defined(CONFIG_SOC_SERIES_BL70X) || defined(CONFIG_SOC_SERIES_BL60X) || \
	defined(CONFIG_SOC_SERIES_BL70XL)
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
} __packed;

struct bflb_flash_header {
	struct bflb_flash_magic_1 magic_1;
	struct bflb_flash_magic_2 magic_2;
	struct bflb_header_flash_cfg flash_cfg;
	uint32_t flash_cfg_crc;
} __packed;

struct bflb_flash_command {
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
};

struct flash_bflb_device_commands {
	/* Auto = commands used with instruction bus (= XIP / memory mapped access) */
	uint8_t auto_read;
	uint8_t auto_read_dmycy;
	uint8_t auto_write;
	uint8_t auto_write_dmycy;
	/* Manual = commands used with system bus (= CPU command access) */
	uint8_t manual_read;
	/* Left so fast read can be used */
	uint8_t manual_read_dmycy;
	/* Up to 4 read/write registers commands */
	uint8_t read_reg[4];
	uint8_t write_reg[4];
	uint8_t contread_on;
	uint8_t contread_off;
	uint8_t burstwrap;
	uint8_t burstwrap_dmycy;
	uint8_t burstwrap_on_data;
	uint8_t burstwrap_off_data;
	uint8_t write_enable;
	uint8_t page_program;
	uint8_t quad_page_program;
	uint8_t sector_erase;
	uint8_t block_erase;
	uint8_t enter_32bits_addr;
	uint8_t exit_32bits_addr;
	/* Extended ops */
	uint8_t reset_enable;
	uint8_t reset;
	uint8_t powerdown;
	uint8_t release_powerdown;
};

/* Register access
 * Index indicates which register command to use
 * Bit indicates position of the bit in the register of size len
 * Len indicates the length to interact with: Some flash require reading each register
 * separately, but write all at once (MX25Lxxx45G for example).
 */
struct flash_bflb_device_registers {
	uint8_t write_enable_index;
	uint8_t write_enable_bit;
	uint8_t write_enable_read_len;
	uint8_t quad_enable_index;
	uint8_t quad_enable_bit;
	uint8_t quad_enable_read_len;
	uint8_t quad_enable_write_len;
	uint8_t busy_index;
	uint8_t busy_bit;
	uint8_t busy_read_len;
};

struct flash_bflb_device_cfg {
	/* SPI modes to use */
	uint8_t auto_spi_mode;
	uint8_t manual_spi_mode;
	struct flash_bflb_device_commands cmd;
	struct flash_bflb_device_registers reg;
	uint32_t size;
	uint32_t page_size;
	uint32_t sector_size;
	uint32_t block_size;
	uint8_t jedec_id[3];
	bool addr_32bits;
};

enum flash_bflb_nxip_message_id {
	NXIP_MSG_NONE = -1,
	NXIP_MSG_READ_INVALID = 0,
	NXIP_MSG_BAD_BUS_IAHB,
	NXIP_MSG_BAD_BUS_SAHB,
	NXIP_MSG_BAD_QE,
	NXIP_MSG_BUSY,
	NXIP_MSG_BUSY_FLASH,
	NXIP_MSG_MAX
};

struct flash_bflb_controller_data {
	bool override_bank1;
	struct bflb_header_flash_cfg flash_header_cfg;
};

enum flash_bflb_bank {
	BANK1 = 1,
	BANK2 = 2,
	BANKMAX
};

enum flash_bflb_pad {
	PAD1 = 1,
	PAD2 = 2,
	PAD3 = 3,
	PADMAX
};

enum flash_bflb_bus_mode {
	BUS_NIO = 0,
	BUS_DO = 1,
	BUS_QO = 2,
	BUS_DIO = 3,
	BUS_QIO = 4,
};

struct flash_bflb_data {
	uintptr_t reg;
	struct flash_bflb_controller_data *controller;
	struct flash_bflb_device_cfg cfg;
	enum flash_bflb_nxip_message_id nxip_message;
	uint32_t nxip_message_args[3];
	uint32_t last_flash_offset;
	enum flash_bflb_bank bank;
	enum flash_bflb_pad pad;
	uintptr_t xip_base;
	uintptr_t xip_end;
	struct k_mutex sahb_mutex;
	struct flash_pages_layout layout;
	struct flash_parameters parameters;
};

struct flash_bflb_config {
	const struct pinctrl_dev_config *pincfg;
};

typedef void (*flash_bflb_nxip_message)(uint32_t arg1, uint32_t arg2, uint32_t arg3);

static void flash_bflb_nxip_message_read_invalid(uint32_t arg1, uint32_t arg2, uint32_t arg3)
{
	LOG_WRN("Header read command (%x) doesn't match DTS read command (%x)", arg1, arg2);
}

static void flash_bflb_nxip_message_bad_bus_iahb(uint32_t arg1, uint32_t arg2, uint32_t arg3)
{
	LOG_WRN("Flash's Bus must be Instruction AHB and not System AHB");
}

static void flash_bflb_nxip_message_bad_bus_sahb(uint32_t arg1, uint32_t arg2, uint32_t arg3)
{
	LOG_WRN("Flash's Bus must be System AHB and not Instruction AHB");
}

static void flash_bflb_nxip_message_bad_qe(uint32_t arg1, uint32_t arg2, uint32_t arg3)
{
	LOG_ERR("Quad enable setup is not supported");
}

static void flash_bflb_nxip_message_busy(uint32_t arg1, uint32_t arg2, uint32_t arg3)
{
	LOG_ERR("Controller is busy!");
}

static void flash_bflb_nxip_message_busy_flash(uint32_t arg1, uint32_t arg2, uint32_t arg3)
{
	LOG_ERR("Flash is busy!");
}

static const flash_bflb_nxip_message flash_bflb_nxip_messages[NXIP_MSG_MAX] = {
	[NXIP_MSG_READ_INVALID] = flash_bflb_nxip_message_read_invalid,
	[NXIP_MSG_BAD_BUS_IAHB] = flash_bflb_nxip_message_bad_bus_iahb,
	[NXIP_MSG_BAD_BUS_SAHB] = flash_bflb_nxip_message_bad_bus_sahb,
	[NXIP_MSG_BAD_QE] = flash_bflb_nxip_message_bad_qe,
	[NXIP_MSG_BUSY] = flash_bflb_nxip_message_busy,
	[NXIP_MSG_BUSY_FLASH] = flash_bflb_nxip_message_busy_flash,
};

static void flash_bflb_nxip_message_set(struct flash_bflb_data *data,
					enum flash_bflb_nxip_message_id id,
					uint32_t arg1, uint32_t arg2, uint32_t arg3)
{
	data->nxip_message = id;
	data->nxip_message_args[0] = arg1;
	data->nxip_message_args[1] = arg2;
	data->nxip_message_args[2] = arg3;
}

static void flash_bflb_nxip_message_clear(struct flash_bflb_data *data)
{
	if (data->nxip_message > NXIP_MSG_NONE) {
		flash_bflb_nxip_messages[data->nxip_message](data->nxip_message_args[0], data->nxip_message_args[1], data->nxip_message_args[2]);
		data->nxip_message = NXIP_MSG_NONE;
	}
}

/* Will using function cause error ? */
static bool flash_bflb_is_in_xip(struct flash_bflb_data *data, void *func)
{
	if ((uint32_t)func > data->xip_base && (uint32_t)func < data->xip_end) {
		LOG_ERR("function at %p is in XIP and will crash the device", func);
		return true;
	}

	return false;
}

/* Are we doing something that makes sense? ? */
static int flash_bflb_is_valid_range(struct flash_bflb_data *data, off_t offset, size_t len)
{
	if (offset < 0) {
		LOG_WRN("0x%lx: before start of flash", (long)offset);
		return -EINVAL;
	}
	if ((data->cfg.size - offset) < len || len > data->cfg.size) {
		LOG_WRN("0x%lx: ends past the end of flash", (long)offset);
		return -EINVAL;
	}

	return 0;
}

static void flash_bflb_set_default_read_header(struct flash_bflb_data *data)
{
	switch (data->cfg.auto_spi_mode) {
	default:
	case BUS_NIO:
		data->cfg.cmd.auto_read = data->controller->flash_header_cfg.fast_read_cmd;
		data->cfg.cmd.auto_read_dmycy = data->controller->flash_header_cfg.fr_dmy_clk;
	break;
	case BUS_DO:
		data->cfg.cmd.auto_read = data->controller->flash_header_cfg.fast_read_do_cmd;
		data->cfg.cmd.auto_read_dmycy = data->controller->flash_header_cfg.fr_do_dmy_clk;
	break;
	case BUS_QO:
		data->cfg.cmd.auto_read = data->controller->flash_header_cfg.fast_read_qo_cmd;
		data->cfg.cmd.auto_read_dmycy = data->controller->flash_header_cfg.fr_qo_dmy_clk;
	break;
	case BUS_DIO:
		data->cfg.cmd.auto_read = data->controller->flash_header_cfg.fast_read_dio_cmd;
		data->cfg.cmd.auto_read_dmycy = data->controller->flash_header_cfg.fr_dio_dmy_clk;
	break;
	case BUS_QIO:
		data->cfg.cmd.auto_read = data->controller->flash_header_cfg.fast_read_qio_cmd;
		data->cfg.cmd.auto_read_dmycy = data->controller->flash_header_cfg.fr_qio_dmy_clk;
	break;
	}
}

static void flash_bflb_set_default_read_default(struct flash_bflb_data *data)
{
	switch (data->cfg.auto_spi_mode) {
	default:
	case BUS_NIO:
		data->cfg.cmd.auto_read = SPI_NOR_CMD_READ_FAST;
		data->cfg.cmd.auto_read_dmycy = 1;
	break;
	case BUS_DO:
		data->cfg.cmd.auto_read = SPI_NOR_CMD_DREAD;
		data->cfg.cmd.auto_read_dmycy = 1;
	break;
	case BUS_QO:
		data->cfg.cmd.auto_read = SPI_NOR_CMD_QREAD;
		data->cfg.cmd.auto_read_dmycy = 1;
	break;
	case BUS_DIO:
		data->cfg.cmd.auto_read = SPI_NOR_CMD_2READ;
		data->cfg.cmd.auto_read_dmycy = 2;
	break;
	case BUS_QIO:
		data->cfg.cmd.auto_read = SPI_NOR_CMD_4READ;
		data->cfg.cmd.auto_read_dmycy = 2;
	break;
	}
}

#if defined(CONFIG_SOC_SERIES_BL70X) || defined(CONFIG_SOC_SERIES_BL60X) || \
	defined(CONFIG_SOC_SERIES_BL70XL)

static void flash_bflb_l1c_wrap(bool enable)
{
	uint32_t tmp;
	bool caching = false;

	tmp = FLASH_READ32(L1C_BASE + L1C_CONFIG_OFFSET);
	/* disable cache */
	if ((tmp & L1C_CACHEABLE_MSK) != 0) {
		caching = true;
		tmp &= ~(1 << L1C_CACHEABLE_POS);
		FLASH_WRITE32(tmp, L1C_BASE + L1C_CONFIG_OFFSET);
	}

	tmp = FLASH_READ32(L1C_BASE + L1C_CONFIG_OFFSET);

	if (enable) {
		tmp &= ~L1C_WRAP_DIS_MSK;
	} else {
		tmp |= L1C_WRAP_DIS_MSK;
	}

	FLASH_WRITE32(tmp, L1C_BASE + L1C_CONFIG_OFFSET);

	if (caching) {
		tmp |= (1 << L1C_CACHEABLE_POS);
		FLASH_WRITE32(tmp, L1C_BASE + L1C_CONFIG_OFFSET);
	}
}


#elif defined(CONFIG_SOC_SERIES_BL61X)

static void flash_bflb_l1c_wrap(bool enable)
{
	/* Do nothing on Bl61x: no L1C */
	ARG_UNUSED(enable);
}

#endif

#if defined(CONFIG_SOC_SERIES_BL61X)

static void flash_bflb_if2_enable(struct flash_bflb_data *data, bool enable)
{
	uint32_t tmp;

	if (data->bank != BANK2) {
		return;
	}
	if (enable) {
		tmp = FLASH_READ32(data->reg + SF_CTRL_SF_IF2_CTRL_1_OFFSET);
		tmp |= SF_CTRL_SF_IF2_EN_MSK;
		FLASH_WRITE32(tmp, data->reg + SF_CTRL_SF_IF2_CTRL_1_OFFSET);

		/* This sets IF2 as the IF currently controlled by SAHB, once setup IF2 can
		 * be disabled.
		 */
		tmp = FLASH_READ32(data->reg + SF_CTRL_SF_IF2_CTRL_1_OFFSET);
		tmp |= SF_CTRL_SF_IF2_FN_SEL_MSK;
		FLASH_WRITE32(tmp, data->reg + SF_CTRL_SF_IF2_CTRL_1_OFFSET);

		tmp = FLASH_READ32(data->reg + SF_CTRL_SF_IF2_CTRL_0_OFFSET);
		tmp &= ~(SF_CTRL_SF_IF2_REPLACE_SF1_MSK
				| SF_CTRL_SF_IF2_REPLACE_SF2_MSK
				| SF_CTRL_SF_IF2_REPLACE_SF3_MSK);

		if (data->pad == PAD1) {
			tmp |= SF_CTRL_SF_IF2_REPLACE_SF1_MSK;
		} else if (data->pad == PAD2) {
			tmp |= SF_CTRL_SF_IF2_REPLACE_SF2_MSK;
		} else {
			tmp |= SF_CTRL_SF_IF2_REPLACE_SF3_MSK;
		}
		tmp &= SF_CTRL_SF_IF2_PAD_SEL_UMSK;
		tmp |= (data->pad - 1U) << SF_CTRL_SF_IF2_PAD_SEL_POS;
		FLASH_WRITE32(tmp, data->reg + SF_CTRL_SF_IF2_CTRL_0_OFFSET);
	} else {
		tmp = FLASH_READ32(data->reg + SF_CTRL_SF_IF2_CTRL_1_OFFSET);
		tmp &= ~SF_CTRL_SF_IF2_FN_SEL_MSK;
		FLASH_WRITE32(tmp, data->reg + SF_CTRL_SF_IF2_CTRL_1_OFFSET);

		tmp = FLASH_READ32(data->reg + SF_CTRL_SF_IF2_CTRL_0_OFFSET);
		tmp &= ~(SF_CTRL_SF_IF2_REPLACE_SF1_MSK
			 | SF_CTRL_SF_IF2_REPLACE_SF2_MSK
			 | SF_CTRL_SF_IF2_REPLACE_SF3_MSK);
		FLASH_WRITE32(tmp, data->reg + SF_CTRL_SF_IF2_CTRL_0_OFFSET);

		tmp = FLASH_READ32(data->reg + SF_CTRL_SF_IF2_CTRL_1_OFFSET);
		tmp &= ~SF_CTRL_SF_IF2_EN_MSK;
		FLASH_WRITE32(tmp, data->reg + SF_CTRL_SF_IF2_CTRL_1_OFFSET);
	}
}

static uintptr_t flash_bflb_set_sahb(struct flash_bflb_data *data)
{
	k_mutex_lock(&data->sahb_mutex, K_FOREVER);

	if (data->bank == BANK2){

		flash_bflb_if2_enable(data, true);

		return data->reg + SF_CTRL_SF_IF2_SAHB_0_OFFSET;
	} else {

		flash_bflb_if2_enable(data, false);

		return data->reg + SF_CTRL_SF_IF_SAHB_0_OFFSET;
	}
}

static void flash_bflb_release_sahb(struct flash_bflb_data *data)
{
	if (data->bank == BANK2){
		flash_bflb_if2_enable(data, false);
	}

	k_mutex_unlock(&data->sahb_mutex);
}

#else

#if defined(CONFIG_SOC_SERIES_BL60X)

static uintptr_t flash_bflb_set_sahb(struct flash_bflb_data *data)
{
	k_mutex_lock(&data->sahb_mutex, K_FOREVER);

	return data->reg + SF_CTRL_SF_IF_SAHB_0_OFFSET;
}

#else

static uintptr_t flash_bflb_set_sahb(struct flash_bflb_data *data)
{
	uint32_t tmp;

	k_mutex_lock(&data->sahb_mutex, K_FOREVER);

	tmp = FLASH_READ32(data->reg + SF_CTRL_2_OFFSET);
	if (data->bank == BANK2) {
		tmp |= SF_CTRL_SF_IF_0_BK_SEL_MSK;
	else {
		tmp &= ~SF_CTRL_SF_IF_0_BK_SEL_MSK;
	}
	FLASH_WRITE32(tmp, data->reg + SF_CTRL_2_OFFSET);

	return data->reg + SF_CTRL_SF_IF_SAHB_0_OFFSET;
}

#endif

static void flash_bflb_release_sahb(struct flash_bflb_data *data)
{
	k_mutex_unlock(&data->sahb_mutex);
}

#endif

#if defined(CONFIG_SOC_SERIES_BL61X)

static uintptr_t flash_bflb_get_if(struct flash_bflb_data *data)
{
	if (data->bank == BANK2){
		return data->reg + SF_CTRL_SF_IF2_CTRL_0_OFFSET;
	} else {
		return data->reg;
	}
}

#else

static uintptr_t flash_bflb_get_if(struct flash_bflb_data *data)
{
	return data->reg;
}

#endif

/* Memcpy will not be in ram */
static void flash_bflb_xip_memcpy(volatile uint8_t *address_from, volatile uint8_t *address_to,
				  size_t size)
{
	for (size_t i = 0; i < size; i++) {
		address_to[i] = address_from[i];
	}
}

static bool flash_bflb_busy_wait(struct flash_bflb_data *data)
{
	uint32_t counter = 0;

	while ((FLASH_READ32(flash_bflb_get_if(data) + SF_CTRL_SF_IF_SAHB_0_OFFSET)
		& SF_CTRL_SF_IF_BUSY_MSK) != 0
		&& counter < BFLB_FLASH_CONTROLLER_BUSY_TIMEOUT * 20000) {
		clock_bflb_settle();
		counter++;
	}

	if ((FLASH_READ32(flash_bflb_get_if(data) + SF_CTRL_SF_IF_SAHB_0_OFFSET)
		& SF_CTRL_SF_IF_BUSY_MSK) != 0) {
		flash_bflb_nxip_message_set(data, NXIP_MSG_BUSY, 0, 0, 0);
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
	uint32_t tmp;

	if (flash_bflb_busy_wait(data)) {
		return -EBUSY;
	}

	tmp = FLASH_READ32(data->reg + SF_CTRL_1_OFFSET);

	if (bus == 1) {
		tmp |= SF_CTRL_SF_IF_FN_SEL_MSK;
		tmp |= SF_CTRL_SF_AHB2SIF_EN_MSK;
	} else {
		tmp &= ~SF_CTRL_SF_IF_FN_SEL_MSK;
		tmp &= ~SF_CTRL_SF_AHB2SIF_EN_MSK;
	}

	FLASH_WRITE32(tmp, data->reg + SF_CTRL_1_OFFSET);

	return 0;
}

static uint8_t flash_bflb_admode_to_spimode(uint8_t addr_mode, uint8_t data_mode)
{
	__ASSERT(addr_mode < 3, "addr_mode unhandled");
	__ASSERT(data_mode < 3, "data_mode unhandled");

	if (addr_mode == 0) {
		return data_mode;
	} else if (addr_mode == 1) {
		return 3;
	} else if (addr_mode == 2) {
		return 4;
	}

	return 0;
}

static int flash_bflb_set_command_iahb(struct flash_bflb_data *data,
				       struct bflb_flash_command *command,
				       bool doing_cmd)
{
	uint32_t tmp;
	uintptr_t reg_off = data->bank == BANK2 ?
		SF_CTRL_SF_IF_IAHB_9_OFFSET : SF_CTRL_SF_IF_IAHB_0_OFFSET;

	if (flash_bflb_busy_wait(data)) {
		return -EBUSY;
	}

	tmp = FLASH_READ32(data->reg + SF_CTRL_1_OFFSET);

	if ((tmp & SF_CTRL_SF_IF_FN_SEL_MSK) == 0) {
		flash_bflb_nxip_message_set(data, NXIP_MSG_BAD_BUS_IAHB, 0, 0, 0);
		return -EINVAL;
	}

	if (data->bank == BANK2) {
		FLASH_WRITE32(command->cmd_buf[0], data->reg + SF_CTRL_SF_IF_IAHB_10_OFFSET);
		FLASH_WRITE32(command->cmd_buf[1], data->reg + SF_CTRL_SF_IF_IAHB_11_OFFSET);
	} else {
		FLASH_WRITE32(command->cmd_buf[0], data->reg + SF_CTRL_SF_IF_IAHB_1_OFFSET);
		FLASH_WRITE32(command->cmd_buf[1], data->reg + SF_CTRL_SF_IF_IAHB_2_OFFSET);
	}

	tmp = FLASH_READ32(data->reg + reg_off);

	/* 4 lines or 1 line commands */
	if (command->cmd_mode == 0) {
		tmp &= ~SF_CTRL_SF_IF_1_QPI_MODE_EN_MSK;
	} else {
		tmp |= SF_CTRL_SF_IF_1_QPI_MODE_EN_MSK;
	}

	/* set SPI mode*/
	tmp &= ~SF_CTRL_SF_IF_1_SPI_MODE_MSK;
	tmp |= command->spi_mode << SF_CTRL_SF_IF_1_SPI_MODE_POS;

	tmp &= ~SF_CTRL_SF_IF_1_CMD_BYTE_MSK;
	/* we are doing a command */
	if (doing_cmd) {
		tmp |= SF_CTRL_SF_IF_1_CMD_EN_MSK;
	} else {
		tmp &= ~SF_CTRL_SF_IF_1_CMD_EN_MSK;
	}

	/* configure address */
	tmp &= ~SF_CTRL_SF_IF_1_ADR_BYTE_MSK;
	if (command->addr_size != 0) {
		tmp |= SF_CTRL_SF_IF_1_ADR_EN_MSK;
		tmp |= ((command->addr_size - 1) << SF_CTRL_SF_IF_1_ADR_BYTE_POS);
	} else {
		tmp &= ~SF_CTRL_SF_IF_1_ADR_EN_MSK;
	}

	/* configure dummy */
	tmp &= ~SF_CTRL_SF_IF_1_DMY_BYTE_MSK;
	if (command->dummy_clks != 0) {
		tmp |= SF_CTRL_SF_IF_1_DMY_EN_MSK;
		tmp |= ((command->dummy_clks - 1) << SF_CTRL_SF_IF_1_DMY_BYTE_POS);
	} else {
		tmp &= ~SF_CTRL_SF_IF_1_DMY_EN_MSK;
	}

	/* configure data */
	if (command->nb_data != 0) {
		tmp |= SF_CTRL_SF_IF_1_DAT_EN_MSK;
	} else {
		tmp &= ~SF_CTRL_SF_IF_1_DAT_EN_MSK;
	}

	/* are we writing ? */
	if (command->rw) {
		tmp |= SF_CTRL_SF_IF_1_DAT_RW_MSK;
	} else {
		tmp &= ~SF_CTRL_SF_IF_1_DAT_RW_MSK;
	}

	FLASH_WRITE32(tmp, data->reg + reg_off);

	return 0;
}

static int flash_bflb_set_command_iahb_write(struct flash_bflb_data *data,
				       struct bflb_flash_command *command,
				       bool doing_cmd)
{
	uint32_t tmp;

	if (flash_bflb_busy_wait(data)) {
		return -EBUSY;
	}

	tmp = FLASH_READ32(data->reg + SF_CTRL_1_OFFSET);

	if ((tmp & SF_CTRL_SF_IF_FN_SEL_MSK) == 0) {
		flash_bflb_nxip_message_set(data, NXIP_MSG_BAD_BUS_IAHB, 0, 0, 0);
		return -EINVAL;
	}

	FLASH_WRITE32(command->cmd_buf[0], data->reg + SF_CTRL_SF_IF_IAHB_4_OFFSET);
	FLASH_WRITE32(command->cmd_buf[1], data->reg + SF_CTRL_SF_IF_IAHB_5_OFFSET);

	tmp = FLASH_READ32(data->reg + SF_CTRL_SF_IF_IAHB_3_OFFSET);

	/* 4 lines or 1 line commands */
	if (command->cmd_mode == 0) {
		tmp &= ~SF_CTRL_SF_IF_2_QPI_MODE_EN_MSK;
	} else {
		tmp |= SF_CTRL_SF_IF_2_QPI_MODE_EN_MSK;
	}

	/* set SPI mode*/
	tmp &= ~SF_CTRL_SF_IF_2_SPI_MODE_MSK;
	tmp |= command->spi_mode << SF_CTRL_SF_IF_2_SPI_MODE_POS;

	tmp &= ~SF_CTRL_SF_IF_2_CMD_BYTE_MSK;
	/* we are doing a command */
	if (doing_cmd) {
		tmp |= SF_CTRL_SF_IF_2_CMD_EN_MSK;
	} else {
		tmp &= ~SF_CTRL_SF_IF_2_CMD_EN_MSK;
	}

	/* configure address */
	tmp &= ~SF_CTRL_SF_IF_2_ADR_BYTE_MSK;
	if (command->addr_size != 0) {
		tmp |= SF_CTRL_SF_IF_2_ADR_EN_MSK;
		tmp |= ((command->addr_size - 1) << SF_CTRL_SF_IF_2_ADR_BYTE_POS);
	} else {
		tmp &= ~SF_CTRL_SF_IF_2_ADR_EN_MSK;
	}

	/* configure dummy */
	tmp &= ~SF_CTRL_SF_IF_2_DMY_BYTE_MSK;
	if (command->dummy_clks != 0) {
		tmp |= SF_CTRL_SF_IF_2_DMY_EN_MSK;
		tmp |= ((command->dummy_clks - 1) << SF_CTRL_SF_IF_2_DMY_BYTE_POS);
	} else {
		tmp &= ~SF_CTRL_SF_IF_2_DMY_EN_MSK;
	}

	/* configure data */
	if (command->nb_data != 0) {
		tmp |= SF_CTRL_SF_IF_2_DAT_EN_MSK;
	} else {
		tmp &= ~SF_CTRL_SF_IF_2_DAT_EN_MSK;
	}

	/* are we writing ? */
	if (command->rw) {
		tmp |= SF_CTRL_SF_IF_2_DAT_RW_MSK;
	} else {
		tmp &= ~SF_CTRL_SF_IF_2_DAT_RW_MSK;
	}

	FLASH_WRITE32(tmp, data->reg + SF_CTRL_SF_IF_IAHB_3_OFFSET);

	return 0;
}

static int flash_bflb_set_command_sahb(struct flash_bflb_data *data,
				       struct bflb_flash_command *command,
				       bool doing_cmd)
{
	uint32_t tmp;
	uint32_t bank_offset = flash_bflb_get_if(data) + SF_CTRL_SF_IF_SAHB_0_OFFSET;

	FLASH_WRITE32(command->cmd_buf[0], bank_offset + 0x4);
	FLASH_WRITE32(command->cmd_buf[1], bank_offset + 0x8);

	tmp = FLASH_READ32(bank_offset + 0);

	/* 4 lines or 1 line commands */
	if (command->cmd_mode == 0) {
		tmp &= ~SF_CTRL_SF_IF_0_QPI_MODE_EN_MSK;
	} else {
		tmp |= SF_CTRL_SF_IF_0_QPI_MODE_EN_MSK;
	}

	/* set SPI mode */
	tmp &= ~SF_CTRL_SF_IF_0_SPI_MODE_MSK;
	tmp |= command->spi_mode << SF_CTRL_SF_IF_0_SPI_MODE_POS;

	tmp &= ~SF_CTRL_SF_IF_0_CMD_BYTE_MSK;
	/* we are doing a command */
	if (doing_cmd) {
		tmp |= SF_CTRL_SF_IF_0_CMD_EN_MSK;
	} else {
		tmp &= ~SF_CTRL_SF_IF_0_CMD_EN_MSK;
	}

	/* configure address */
	tmp &= ~SF_CTRL_SF_IF_0_ADR_BYTE_MSK;
	if (command->addr_size != 0) {
		tmp |= SF_CTRL_SF_IF_0_ADR_EN_MSK;
		tmp |= ((command->addr_size - 1U) << SF_CTRL_SF_IF_0_ADR_BYTE_POS);
	} else {
		tmp &= ~SF_CTRL_SF_IF_0_ADR_EN_MSK;
	}

	/* configure dummy */
	tmp &= ~SF_CTRL_SF_IF_0_DMY_BYTE_MSK;
	if (command->dummy_clks != 0) {
		tmp |= SF_CTRL_SF_IF_0_DMY_EN_MSK;
		tmp |= ((command->dummy_clks - 1U) << SF_CTRL_SF_IF_0_DMY_BYTE_POS);
	} else {
		tmp &= ~SF_CTRL_SF_IF_0_DMY_EN_MSK;
	}

	/* configure data */
	tmp &= ~SF_CTRL_SF_IF_0_DAT_BYTE_MSK;
	if (command->nb_data != 0) {
		tmp |= SF_CTRL_SF_IF_0_DAT_EN_MSK;
		tmp |= ((command->nb_data - 1U) << SF_CTRL_SF_IF_0_DAT_BYTE_POS);
	} else {
		tmp &= ~SF_CTRL_SF_IF_0_DAT_EN_MSK;
	}

	/* are we writing ? */
	if (command->rw) {
		tmp |= SF_CTRL_SF_IF_0_DAT_RW_MSK;
	} else {
		tmp &= ~SF_CTRL_SF_IF_0_DAT_RW_MSK;
	}
	FLASH_WRITE32(tmp, bank_offset + 0);

	return 0;
}

static int flash_bflb_send_command(struct flash_bflb_data *data, struct bflb_flash_command *command)
{
	uint32_t tmp;
	int ret;
	uint32_t bank_offset = flash_bflb_get_if(data) + SF_CTRL_SF_IF_SAHB_0_OFFSET;

	if (flash_bflb_is_in_xip(data, &flash_bflb_send_command)) {
		return -ENOTSUP;
	}

	if (flash_bflb_busy_wait(data)) {
		return -EBUSY;
	}

	tmp = FLASH_READ32(data->reg + SF_CTRL_1_OFFSET);
	if (tmp & SF_CTRL_SF_IF_FN_SEL_MSK) {
		flash_bflb_nxip_message_set(data, NXIP_MSG_BAD_BUS_SAHB, 0, 0, 0);
		return -EINVAL;
	}

	/* make sure command detriggered */
	tmp = FLASH_READ32(bank_offset + 0);
	tmp &= ~SF_CTRL_SF_IF_0_TRIG_MSK;
	FLASH_WRITE32(tmp, bank_offset + 0);

	ret = flash_bflb_set_command_sahb(data, command, true);
	if (ret != 0) {
		return ret;
	}

#if defined(CONFIG_SOC_SERIES_BL70X) || defined(CONFIG_SOC_SERIES_BL60X)
	tmp = FLASH_READ32(data->reg + SF_CTRL_0_OFFSET);
	tmp |= SF_CTRL_SF_CLK_SAHB_SRAM_SEL_MSK;
	FLASH_WRITE32(tmp, data->reg + SF_CTRL_0_OFFSET);
#endif

	/* trigger command */
	tmp = FLASH_READ32(bank_offset + 0);
	tmp |= SF_CTRL_SF_IF_0_TRIG_MSK;
	FLASH_WRITE32(tmp, bank_offset + 0);

	if (flash_bflb_busy_wait(data)) {
		ret = -EBUSY;
	}

#if defined(CONFIG_SOC_SERIES_BL70X) || defined(CONFIG_SOC_SERIES_BL60X)
	tmp = FLASH_READ32(data->reg + SF_CTRL_0_OFFSET);
	tmp &= ~SF_CTRL_SF_CLK_SAHB_SRAM_SEL_MSK;
	FLASH_WRITE32(tmp, data->reg + SF_CTRL_0_OFFSET);
#endif

	return ret;
}


static int flash_bflb_flash_read_register(struct flash_bflb_data *data, uint8_t index, uint8_t *out,
	uint8_t len)
{
	struct bflb_flash_command read_reg = {0};
	int ret;

	read_reg.spi_mode = data->cfg.manual_spi_mode;
	read_reg.cmd_buf[0] = (data->cfg.cmd.read_reg[index]) << 24;
	read_reg.nb_data = len;
	ret = flash_bflb_send_command(data, &read_reg);
	if (ret != 0) {
		return ret;
	}

	if (flash_bflb_busy_wait(data)) {
		return -EBUSY;
	}

	flash_bflb_xip_memcpy((uint8_t *)SF_CTRL_BUF_BASE, out, len);

	return 0;
}


static int flash_bflb_flash_write_register(struct flash_bflb_data *data, uint8_t index, uint8_t *in,
	uint8_t len)
{
	struct bflb_flash_command write_reg = {0};

	flash_bflb_xip_memcpy(in, (uint8_t *)SF_CTRL_BUF_BASE, len);

	write_reg.spi_mode = data->cfg.manual_spi_mode;
	write_reg.cmd_buf[0] = (data->cfg.cmd.write_reg[index]) << 24;
	write_reg.nb_data = len;
	write_reg.rw = 1;

	return flash_bflb_send_command(data, &write_reg);
}

static int flash_bflb_flash_disable_continuous_read(struct flash_bflb_data *data)
{
	struct bflb_flash_command disable_continuous_read = {0};

	/* Effectively send the stop continuous read command 4 times, this is fine
	 * and the correct thing to do according to flash datasheets.
	 */
	disable_continuous_read.spi_mode = data->cfg.manual_spi_mode;
	disable_continuous_read.addr_size = 3;
	disable_continuous_read.cmd_buf[0] = data->cfg.cmd.contread_off << 24 |
	data->cfg.cmd.contread_off << 16 | data->cfg.cmd.contread_off << 8 |
	data->cfg.cmd.contread_off;

	return flash_bflb_send_command(data, &disable_continuous_read);
}

static int flash_bflb_flash_disable_burst(struct flash_bflb_data *data)
{
	uint32_t tmp;
	struct bflb_flash_command disable_burstwrap = {0};

	flash_bflb_l1c_wrap(false);

	disable_burstwrap.dummy_clks = data->cfg.cmd.burstwrap_dmycy;
	disable_burstwrap.spi_mode = data->cfg.auto_spi_mode;
	disable_burstwrap.cmd_buf[0] = data->cfg.cmd.burstwrap << 24;
	disable_burstwrap.nb_data = 1;
	disable_burstwrap.rw = 1;
	tmp = data->cfg.cmd.burstwrap_off_data;
	FLASH_WRITE32(tmp, SF_CTRL_BUF_BASE);

	return flash_bflb_send_command(data, &disable_burstwrap);
}

static int flash_bflb_flash_enable_burst(struct flash_bflb_data *data)
{
	uint32_t tmp;
	struct bflb_flash_command enable_burstwrap = {0};

	flash_bflb_l1c_wrap(true);

	enable_burstwrap.dummy_clks = data->cfg.cmd.burstwrap_dmycy;
	enable_burstwrap.spi_mode = data->cfg.auto_spi_mode;
	enable_burstwrap.cmd_buf[0] = data->cfg.cmd.burstwrap << 24;
	enable_burstwrap.nb_data = 1;
	enable_burstwrap.rw = 1;
	tmp = data->cfg.cmd.burstwrap_on_data;
	FLASH_WRITE32(tmp, SF_CTRL_BUF_BASE);

	return flash_bflb_send_command(data, &enable_burstwrap);
}

static int flash_bflb_enable_writable(struct flash_bflb_data *data)
{
	struct bflb_flash_command write_enable = {0};
	int ret;
	uint32_t write_reg;

	write_enable.spi_mode = data->cfg.manual_spi_mode;
	write_enable.cmd_buf[0] = (data->cfg.cmd.write_enable) << 24;
	ret = flash_bflb_send_command(data, &write_enable);
	if (ret != 0) {
		return ret;
	}

	/* check writable */
	ret = flash_bflb_flash_read_register(data, data->cfg.reg.write_enable_index,
		(uint8_t *)&write_reg, data->cfg.reg.write_enable_read_len);
	if (ret != 0) {
		return ret;
	}

	if ((write_reg & BIT(data->cfg.reg.write_enable_bit)) != 0) {
		return 0;
	}

	return -EIO;
}

static int flash_bflb_enable_qspi(struct flash_bflb_data *data)
{
	int ret;
	uint32_t tmp = 0;

	/* No Quad Enable (!= QPI enable) */
	if (data->cfg.reg.quad_enable_read_len == 0) {
		return 0;
	}

	/* If read length is not the same as write length, write all registers.
	 * No cases where more than 2 registers must be written
	 */
	if (data->cfg.reg.quad_enable_write_len < 1 || data->cfg.reg.quad_enable_write_len > 2
	    || data->cfg.reg.quad_enable_index > 1 || data->cfg.reg.quad_enable_read_len != 1
	    || (data->cfg.reg.quad_enable_write_len > 1 && !(data->cfg.reg.quad_enable_index == 1))
	    || data->cfg.reg.quad_enable_bit > 7
	) {
		flash_bflb_nxip_message_set(data, NXIP_MSG_BAD_QE, 0, 0, 0);
		return -EINVAL;
	}

	/* writable command also enables writing to configuration registers, not just data*/
	ret = flash_bflb_enable_writable(data);
	if (ret != 0) {
		return ret;
	}

	/* get quad enable register value */
	ret = flash_bflb_flash_read_register(data, data->cfg.reg.quad_enable_index,
		(uint8_t *)&tmp, data->cfg.reg.quad_enable_read_len);
	if (ret != 0) {
		return ret;
	}

	/* qe is already enable*/
	if ((*(uint8_t *)&tmp & BIT(data->cfg.reg.quad_enable_bit)) != 0) {
		return 0;
	}

	if (data->cfg.reg.quad_enable_write_len > 1) {
		/* We assume quad enable is in second register.
		 * Other configurations are unsupported and not yet observed.
		 */

		/* First register (status) */
		ret = flash_bflb_flash_read_register(data, 0, (uint8_t *)&tmp, 1);
		if (ret != 0) {
			return ret;
		}
		/* Second register (configuration) */
		ret = flash_bflb_flash_read_register(data, 1, &(((uint8_t *)&tmp)[1]), 1);
		if (ret != 0) {
			return ret;
		}

		((uint8_t *)&tmp)[1] |= BIT(data->cfg.reg.quad_enable_bit);

	/* we only need to read and write the appropriate register (usually the second one) */
	} else {
		tmp |= BIT(data->cfg.reg.quad_enable_bit);
	}

	ret = flash_bflb_flash_write_register(data, data->cfg.reg.quad_enable_index,
					      (uint8_t *)&tmp, data->cfg.reg.quad_enable_write_len);
	if (ret != 0) {
		return ret;
	}

	ret = flash_bflb_flash_read_register(data, data->cfg.reg.quad_enable_index, (uint8_t *)&tmp,
					     data->cfg.reg.quad_enable_read_len);
	if (ret != 0) {
		return ret;
	}

	/* check Quad is Enabled */
	if ((*(uint8_t *)&tmp & BIT(data->cfg.reg.quad_enable_bit)) != 0) {
		return 0;
	}

	return -EIO;
}

/* ID0 for CPU 0, ID1 for cpu 1 */
static uint32_t flash_bflb_get_offset(struct flash_bflb_data *data)
{
	uint32_t tmp;
	uintptr_t reg = data->bank == BANK2 ?
		SF_CTRL_SF_BK2_ID0_OFFSET_OFFSET : SF_CTRL_SF_ID0_OFFSET_OFFSET;

	tmp = FLASH_READ32(data->reg + reg);
	tmp &= SF_CTRL_SF_ID0_OFFSET_MSK;
	tmp = tmp >> SF_CTRL_SF_ID0_OFFSET_POS;

	return tmp;
}

static void flash_bflb_set_offset(struct flash_bflb_data *data, uintptr_t offset)
{
	uint32_t tmp;
	uintptr_t reg = data->bank == BANK2 ?
		SF_CTRL_SF_BK2_ID0_OFFSET_OFFSET : SF_CTRL_SF_ID0_OFFSET_OFFSET;

	tmp = FLASH_READ32(data->reg + reg);
	tmp &= ~SF_CTRL_SF_ID0_OFFSET_MSK;
	tmp |= offset << SF_CTRL_SF_ID0_OFFSET_POS;
	FLASH_WRITE32(tmp, data->reg + reg);
}

static int flash_bflb_save_xip_state(const struct device *dev)
{
	struct flash_bflb_data *data = dev->data;
	int ret;

	flash_bflb_set_sahb(data);

	/* Bus to system AHB, effectively immediately disables XIP access *for all* */
	ret = flash_bflb_set_bus(data, 0);
	if (ret != 0) {
		goto exit_here;
	}

	/* Disable continuous read */
	if (data->cfg.cmd.contread_on != 0) {
		ret = flash_bflb_flash_disable_continuous_read(data);
		if (ret != 0) {
			goto exit_here;
		}
	}

	/* Disable burst with wrap*/
	if (data->cfg.cmd.burstwrap != 0) {
		ret = flash_bflb_flash_disable_burst(data);
		if (ret != 0) {
			goto exit_here;
		}
	}

	/* enable quad previous command could've disabled it */
	if (data->cfg.manual_spi_mode == BUS_QIO || data->cfg.manual_spi_mode == BUS_QO) {
		ret = flash_bflb_enable_qspi(data);
		if (ret != 0) {
			goto exit_here;
		}
	}

	/* disable burst with wrap*/
	if (data->cfg.cmd.burstwrap != 0) {
		ret = flash_bflb_flash_disable_burst(data);
		if (ret != 0) {
			goto exit_here;
		}
	}

exit_here:
	if (ret != 0) {
		LOG_ERR("Failed to save XIP state: %d", ret);
		flash_bflb_nxip_message_clear(data);
	}
	return ret;
}

static bool flash_bflb_flash_busy_wait(struct flash_bflb_data *data)
{
	uint8_t tmp_bus = 0xFF;
	uint32_t counter = 0;

	while ((tmp_bus & BIT(data->cfg.reg.busy_bit)) != 0 && counter <
		BFLB_FLASH_CHIP_BUSY_TIMEOUT * 20000) {
		flash_bflb_flash_read_register(data, data->cfg.reg.busy_index, &tmp_bus,
					       data->cfg.reg.busy_read_len);
		clock_bflb_settle();
		counter++;
	}


	if ((tmp_bus & BIT(data->cfg.reg.busy_bit)) != 0) {
		flash_bflb_nxip_message_set(data, NXIP_MSG_BUSY_FLASH, 0, 0, 0);
		return true;
	}

	return false;
}

static int flash_bflb_xip_init(struct flash_bflb_data *data)
{
	struct bflb_flash_command xip_cmd = {0};
	struct bflb_flash_command cont_read_init_cmd = {0};
	bool is_command = true;
	uint32_t buf;
	int ret;

	xip_cmd.spi_mode = data->cfg.auto_spi_mode;
	xip_cmd.cmd_buf[0] = data->cfg.cmd.auto_read << 24;
	xip_cmd.dummy_clks = data->cfg.cmd.auto_read_dmycy;
	/* IAHB reads 32 bytes at once */
	xip_cmd.nb_data = 32;

	/* 3 for 24 bits, 4 for 32 bits */
	if (data->cfg.addr_32bits) {
		xip_cmd.addr_size = BFLB_FLASH_ADDR_SIZE_32B;
	} else {
		xip_cmd.addr_size = BFLB_FLASH_ADDR_SIZE;
	}

	if ((data->cfg.auto_spi_mode == BUS_DIO || data->cfg.auto_spi_mode == BUS_QIO)
	    && data->cfg.cmd.contread_on != 0
	) {
		is_command = false;
		xip_cmd.addr_size += BFLB_FLASH_ADDR_SIZE_CONTREAD_ADD;
		if (data->cfg.addr_32bits) {
			xip_cmd.cmd_buf[0] = 0;
			xip_cmd.cmd_buf[1] = data->cfg.cmd.contread_on << 24;
		} else {
			xip_cmd.cmd_buf[0] = data->cfg.cmd.contread_on;
		}

		flash_bflb_xip_memcpy((uint8_t *)&xip_cmd, (uint8_t *)&cont_read_init_cmd, sizeof(xip_cmd));
		/* Align */
		cont_read_init_cmd.nb_data = 4;
		cont_read_init_cmd.cmd_buf[0] = data->cfg.cmd.auto_read << 24;
		if (data->cfg.addr_32bits) {
			cont_read_init_cmd.cmd_buf[1] = data->cfg.cmd.contread_on << 16;
		} else {
			cont_read_init_cmd.cmd_buf[1] = data->cfg.cmd.contread_on << 24;
		}

		ret = flash_bflb_set_bus(data, 0);
		if (ret != 0) {
			return ret;
		}

		ret = flash_bflb_send_command(data, &cont_read_init_cmd);
		if (ret != 0) {
			return ret;
		}

		if (flash_bflb_busy_wait(data)) {
			return -EBUSY;
		}

		if (flash_bflb_flash_busy_wait(data)) {
			return -EBUSY;
		}

		flash_bflb_xip_memcpy((uint8_t *)SF_CTRL_BUF_BASE, (uint8_t *)(&buf), 4);
	}

	/* Bus to instruction AHB */
	ret = flash_bflb_set_bus(data, 1);
	if (ret != 0) {
		return ret;
	}

	// if (data->bank == BANK2) {
	// 	xip_cmd.addr_size--;
	// }

	return flash_bflb_set_command_iahb(data, &xip_cmd, is_command);
}

static int flash_bflb_autowrite_init(struct flash_bflb_data *data)
{
	struct bflb_flash_command autowrite_cmd = {0};
	int ret;

	autowrite_cmd.spi_mode = data->cfg.auto_spi_mode;
	autowrite_cmd.cmd_buf[0] = data->cfg.cmd.auto_write << 24;
	autowrite_cmd.dummy_clks = data->cfg.cmd.auto_write_dmycy;
	autowrite_cmd.rw = 1;
	/* IAHB writes 32 bytes at once */
	autowrite_cmd.nb_data = 32;

	/* 3 for 24 bits, 4 for 32 bits */
	if (data->cfg.addr_32bits) {
		autowrite_cmd.addr_size = BFLB_FLASH_ADDR_SIZE_32B;
	} else {
		autowrite_cmd.addr_size = BFLB_FLASH_ADDR_SIZE;
	}

	/* Bus to instruction AHB */
	ret = flash_bflb_set_bus(data, 1);
	if (ret != 0) {
		return ret;
	}

	return flash_bflb_set_command_iahb_write(data, &autowrite_cmd, true);
}

static int flash_bflb_restore_xip_state(struct flash_bflb_data *data)
{
	int ret;

	/* Enable quad if relevant */
	if (data->cfg.auto_spi_mode == BUS_QIO || data->cfg.auto_spi_mode == BUS_QO) {
		ret = flash_bflb_enable_qspi(data);
		if (ret != 0) {
			goto exit_here;
		}
	}

	/* reenable burst read */
	if (data->cfg.cmd.burstwrap != 0) {
		ret = flash_bflb_flash_enable_burst(data);
		if (ret != 0) {
			goto exit_here;
		}
	}

	ret = flash_bflb_xip_init(data);
	if (ret != 0) {
		goto exit_here;
	}

	if (data->bank == BANK2 && data->cfg.cmd.auto_write != 0) {
		ret = flash_bflb_autowrite_init(data);
		if (ret != 0) {
			goto exit_here;
		}
	}

exit_here:

	if (ret != 0) {
		/* Attempt to restore still functional XIP bank (Fine if it is bank 2 failing) */
		flash_bflb_set_bus(data, 1);
	}

	flash_bflb_release_sahb(data);

	flash_bflb_nxip_message_clear(data);

	return ret;
}

#if defined(CONFIG_SOC_FLASH_BFLB_DIRECT_ACCESS)

static int flash_bflb_read_sahb_do(struct flash_bflb_data *data, off_t address, void *buffer,
				   size_t length)
{
	int ret;
	struct bflb_flash_command read_cmd = {0};
	size_t i, cur_len;


	read_cmd.spi_mode = data->cfg.manual_spi_mode;
	read_cmd.dummy_clks = data->cfg.cmd.manual_read_dmycy;
	read_cmd.cmd_buf[0] = data->cfg.cmd.manual_read << 24;

	if (data->cfg.addr_32bits) {
		read_cmd.addr_size = BFLB_FLASH_ADDR_SIZE_32B;
	} else {
		read_cmd.addr_size = BFLB_FLASH_ADDR_SIZE;
	}

	i = 0;
	while (i < length) {

		cur_len = BFLB_FLASH_SF_BUF_SIZE - ((address + i) % BFLB_FLASH_SF_BUF_SIZE);

		if (cur_len > length - i) {
			cur_len = length - i;
		}

		read_cmd.cmd_buf[0] &= ~0xFFFFFF;

		if (data->cfg.addr_32bits) {
			read_cmd.cmd_buf[0] |= (address + i) >> 8;
			read_cmd.cmd_buf[1] = (address + i) << 24;
		} else {
			read_cmd.cmd_buf[0] |= (address + i);
		}

		read_cmd.nb_data = cur_len;

		ret = flash_bflb_send_command(data, &read_cmd);
		if (ret != 0) {
			return ret;
		}

		flash_bflb_xip_memcpy((uint8_t *)SF_CTRL_BUF_BASE, (uint8_t *)(buffer) + i,
				      cur_len);

		i += cur_len;

		if (flash_bflb_busy_wait(data)) {
			return -EBUSY;
		}

		if (flash_bflb_flash_busy_wait(data)) {
			return -EBUSY;
		}
	}

	return 0;
}

/* copies flash data using direct access */
static int flash_bflb_read(const struct device *dev, off_t address, void *buffer, size_t length)
{
	struct flash_bflb_data *data = dev->data;
	unsigned int	locker;
	int ret;

	if (length == 0) {
		return 0;
	}

	ret = flash_bflb_is_valid_range(data, address, length);
	if (ret != 0) {
		return ret;
	}

	if (flash_bflb_is_in_xip(data, &flash_bflb_read)) {
		return -ENOTSUP;
	}

	/* interrupting would break, likely to access XIP*/
	locker = irq_lock();

	ret = flash_bflb_save_xip_state(dev);
	if (ret != 0) {
		irq_unlock(locker);
		return ret;
	}

	ret = flash_bflb_read_sahb_do(data, address, buffer, length);

	if (ret != 0) {
		flash_bflb_restore_xip_state(data);
	} else {
		ret = flash_bflb_restore_xip_state(data);
	}
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
	int ret;

	if (length == 0) {
		return 0;
	}

	ret = flash_bflb_is_valid_range(data, address, length);
	if (ret != 0) {
		return ret;
	}

	if (flash_bflb_is_in_xip(data, &flash_bflb_read)) {
		return -ENOTSUP;
	}

	/* interrupting would break, likely to access XIP*/
	locker = irq_lock();

	/* get XIP offset / where code really is in flash, usually 0x2000 */
	img_offset = flash_bflb_get_offset(data);

	/* need set offset to 0 to access? */
	if (address < img_offset) {

		sys_cache_data_flush_and_invd_all();

		/* set offset to 0 to access first (likely)0x2000 of flash */
		flash_bflb_set_offset(data, 0);

		/* copy data we need */
		flash_bflb_xip_memcpy((uint8_t *)(address + data->xip_base),
				      (uint8_t *)buffer, length);

		sys_cache_data_flush_and_invd_all();

		flash_bflb_set_offset(data, img_offset);
	} else {
		/* copy data we need */
		flash_bflb_xip_memcpy((uint8_t *)(address + data->xip_base - img_offset),
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
	uint32_t	tmp;
	unsigned int	locker;
	int		ret, rete;
	uint32_t	cur_len, i;
	struct bflb_flash_command write_cmd = {0};
	uint32_t	img_offset;

	if (length == 0) {
		return 0;
	}

	ret = flash_bflb_is_valid_range(data, address, length);
	if (ret != 0) {
		return ret;
	}

	if (flash_bflb_is_in_xip(data, &flash_bflb_write)) {
		return -ENOTSUP;
	}

	/* No need for commands if we have automatic write */
	if (data->cfg.cmd.auto_write != 0) {
		/* get XIP offset / where code really is in flash, usually 0x0 when writable */
		img_offset = flash_bflb_get_offset(data);

		/* need set offset to 0 to access? */
		if (address < img_offset) {
			sys_cache_data_flush_and_invd_all();

			/* set offset to 0 to access first (likely)0x2000 of flash */
			flash_bflb_set_offset(data, 0);

			/* copy data we need */
			flash_bflb_xip_memcpy((uint8_t *)buffer,
					(uint8_t *)(address + data->xip_base), length);

			sys_cache_data_flush_and_invd_all();

			flash_bflb_set_offset(data, img_offset);
		} else {
			/* copy data we need */
			flash_bflb_xip_memcpy((uint8_t *)buffer,
					      (uint8_t *)(address + data->xip_base - img_offset),
					      length);
		}

		return 0;
	}

	/* interrupting would break, likely to access XIP*/
	locker = irq_lock();

	ret = flash_bflb_save_xip_state(dev);
	if (ret != 0) {
		irq_unlock(locker);
		return ret;
	}

	/* Check if the flash chip is OK to write to */
	ret = flash_bflb_flash_read_register(data, 0, (uint8_t *)(&tmp), 1);
	if (ret != 0) {
		goto exit_here;
	}
	if ((tmp & BFLB_FLASH_FLASH_BLOCK_PROTECT_MSK) != 0) {
		ret = -EINVAL;
		goto exit_here;
	}

	if (data->cfg.manual_spi_mode == BUS_QIO || data->cfg.manual_spi_mode == BUS_QO) {
		write_cmd.cmd_buf[0] = data->cfg.cmd.quad_page_program << 24;
	} else {
		write_cmd.cmd_buf[0] = data->cfg.cmd.page_program << 24;
	}

	write_cmd.rw = 1;
	if (data->cfg.addr_32bits) {
		write_cmd.addr_size = BFLB_FLASH_ADDR_SIZE_32B;
	} else {
		write_cmd.addr_size = BFLB_FLASH_ADDR_SIZE;
	}

	i = 0;
	while (i < length) {
		/* Write enable is needed for every write */
		ret = flash_bflb_enable_writable(data);
		if (ret != 0) {
			goto exit_here;
		}

		/* Get current position within page size,
		 * this assumes page_size <= CTRL_BUF_SIZE
		 */
		cur_len = data->cfg.page_size - ((address + i) % data->cfg.page_size);

		if (cur_len > length - i) {
			cur_len = length - i;
		}

		flash_bflb_xip_memcpy((uint8_t *)(buffer) +  i, (uint8_t *)SF_CTRL_BUF_BASE,
				      cur_len);

		write_cmd.cmd_buf[0] &= ~0xFFFFFF;

		if (data->cfg.addr_32bits) {
			write_cmd.cmd_buf[0] |= (address + i) >> 8;
			write_cmd.cmd_buf[1] = (address + i) << 24;
		} else {
			write_cmd.cmd_buf[0] |= (address + i);
		}

		write_cmd.nb_data = cur_len;
		ret = flash_bflb_send_command(data, &write_cmd);
		if (ret != 0) {
			goto exit_here;
		}

		i += cur_len;

		flash_bflb_busy_wait(data);
		flash_bflb_flash_busy_wait(data);
	}

exit_here:
	rete = flash_bflb_restore_xip_state(data);
	irq_unlock(locker);

	return (ret != 0 ? ret : rete);
}

static int flash_bflb_erase(const struct device *dev, off_t start, size_t len)
{
	struct flash_bflb_data *data = dev->data;
	unsigned int	locker;
	int		ret, rete;
	struct bflb_flash_command erase_cmd = {0};
	uint32_t erase_start = start / data->cfg.sector_size;
	uint32_t erase_end = (len / data->cfg.sector_size)
		+ start / data->cfg.sector_size;

	if (len == 0) {
		return 0;
	}

	/* No explicit erase needed if auto-writing */
	if (data->cfg.cmd.auto_write != 0) {
		return 0;
	}

	ret = flash_bflb_is_valid_range(data, start, len);
	if (ret != 0) {
		return ret;
	}

	if (flash_bflb_is_in_xip(data, &flash_bflb_write)) {
		return -ENOTSUP;
	}

	if ((len % data->cfg.sector_size) != 0) {
		LOG_WRN("Length is not a multiple of minimal erase block size");
		return -EINVAL;
	}

	if ((start % data->cfg.sector_size) != 0) {
		LOG_WRN("Start address is not a multiple of minimal erase block size");
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
	if (data->cfg.addr_32bits) {
		erase_cmd.addr_size = BFLB_FLASH_ADDR_SIZE_32B;
	} else {
		erase_cmd.addr_size = BFLB_FLASH_ADDR_SIZE;
	}

	for (uint32_t i = erase_start; i < erase_end; i++) {
		/* Write enable is needed for every write */
		ret = flash_bflb_enable_writable(data);
		if (ret != 0) {
			goto exit_here;
		}

		erase_cmd.cmd_buf[0] = data->cfg.cmd.sector_erase << 24;
		if (data->cfg.addr_32bits) {
			erase_cmd.cmd_buf[0] |= (i * data->cfg.sector_size) >> 8;
			erase_cmd.cmd_buf[1] = (i * data->cfg.sector_size) << 24;
		} else {
			erase_cmd.cmd_buf[0] |= (i * data->cfg.sector_size);
		}

		ret = flash_bflb_send_command(data, &erase_cmd);
		if (ret != 0) {
			goto exit_here;
		}

		flash_bflb_busy_wait(data);
		flash_bflb_flash_busy_wait(data);
	}

exit_here:
	rete = flash_bflb_restore_xip_state(data);
	irq_unlock(locker);

	return (ret != 0 ? ret : rete);
}


#ifdef CONFIG_FLASH_PAGE_LAYOUT
void flash_bflb_page_layout(const struct device *dev,
			     const struct flash_pages_layout **layout,
			     size_t *layout_size)
{
	struct flash_bflb_data *data = dev->data;

	data->layout.pages_size = data->cfg.sector_size;
	data->layout.pages_count = data->cfg.size / data->cfg.sector_size;

	*layout = &data->layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *flash_bflb_get_parameters(const struct device *dev)
{
	struct flash_bflb_data *data = dev->data;
	struct flash_parameters parameters = {
		.write_block_size = data->cfg.page_size,
		.erase_value = ERASE_VALUE,
	};

	/* Why is this thing const in a way that propagates ?
	 * What if SFPD or other discovery finds the setting is wrong?
	 * Workaround for not having to do a really stupid malloc.
	 */
	memcpy(&data->parameters, &parameters, sizeof(struct flash_parameters));

	return &data->parameters;
}

/* from SDK because there is no matching zephyr crc for the bflb flash crc, this is supposedly a
 * implementation of ZIP crc32
 */
static uint32_t bflb_soft_crc32(uint32_t initial, void *in, uint32_t len)
{
	uint32_t crc = ~initial;
	uint8_t *data = (uint8_t *)in;

	while (len--) {
		crc ^= *data++;
		for (uint8_t i = 0; i < 8; ++i) {
			if (crc & 1) {
				/* 0xEDB88320 = reverse 0x04C11DB7 */
				crc = (crc >> 1) ^ 0xEDB88320U;
			} else {
				crc = (crc >> 1);
			}
		}
	}

	return ~crc;
}

static int flash_bflb_get_jedec_id_internal(struct flash_bflb_data *data, uint8_t *out)
{
	struct bflb_flash_command get_jedecid = {0};
	int ret;
	uint32_t tmp;

	get_jedecid.spi_mode = BUS_NIO;
	get_jedecid.cmd_buf[0] = SPI_NOR_CMD_RDID << 24;
	get_jedecid.nb_data = 3;
	get_jedecid.addr_size = 3;

	ret = flash_bflb_send_command(data, &get_jedecid);
	if (ret < 0) {
		return ret;
	}

	tmp = FLASH_READ32(SF_CTRL_BUF_BASE);

	flash_bflb_xip_memcpy((uint8_t *)&tmp, out, 3);

	return 0;
}

static int flash_bflb_reset(struct flash_bflb_data *data)
{
	struct bflb_flash_command reset = {0};
	int ret;

	reset.spi_mode = BUS_NIO;
	reset.cmd_buf[0] = 0x66 << 24;

	ret = flash_bflb_send_command(data, &reset);
	if (ret < 0) {
		return ret;
	}

	reset.cmd_buf[0] = 0x99 << 24;
	ret = flash_bflb_send_command(data, &reset);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

#if defined(CONFIG_FLASH_JESD216_API)

static int flash_bflb_get_jedec_id(const struct device *dev, uint8_t *id)
{
	struct flash_bflb_data *data = dev->data;
	unsigned int locker;
	int ret;

	if (flash_bflb_is_in_xip(data, &flash_bflb_get_jedec_id)) {
		return -ENOTSUP;
	}

	locker = irq_lock();

	ret = flash_bflb_save_xip_state(dev);
	if (ret != 0) {
		irq_unlock(locker);
		return ret;
	}

	ret = flash_bflb_get_jedec_id_internal(data, id);
	if (ret != 0) {
		goto exit_here;
	}

exit_here:
	flash_bflb_restore_xip_state(data);
	irq_unlock(locker);

	return ret;
}

int flash_bflb_read_sfdp(const struct device *dev, off_t offset,
			      void *data, size_t len)
{
	return 0;
}

#endif

/* /!\ this function cannot run from XIP! */
static int flash_bflb_header_fetch(struct flash_bflb_data *data)
{
	uint32_t tmp;
	uint32_t img_offset;
	unsigned int locker;
	struct bflb_flash_header header;

	if (flash_bflb_is_in_xip(data, &flash_bflb_header_fetch)) {
		return -ENOTSUP;
	}

	/* get flash config using xip access */

	/* interrupting would break, likely to access XIP*/
	locker = irq_lock();
	/* get XIP offset / where code really is in flash, usually 0x2000 */
	img_offset = flash_bflb_get_offset(data);

	sys_cache_data_flush_and_invd_all();

	/* set offset to 0 to access first (likely)0x2000 of flash */
	flash_bflb_set_offset(data, 0);

	/* copy data we need */
	flash_bflb_xip_memcpy((uint8_t *)(data->xip_base),
			      (uint8_t *)&header, sizeof(struct bflb_flash_header));

	sys_cache_data_flush_and_invd_all();

	flash_bflb_set_offset(data, img_offset);

	/* done with interrupt breaking stuffs */
	irq_unlock(locker);

	/* magic */
	if (!(header.magic_2.magic[0] == BFLB_FLASH_MAGIC_2[0]
		&& header.magic_2.magic[1] == BFLB_FLASH_MAGIC_2[1]
		&& header.magic_2.magic[2] == BFLB_FLASH_MAGIC_2[2]
		&& header.magic_2.magic[3] == BFLB_FLASH_MAGIC_2[3])) {
		LOG_ERR("Flash data magic is incorrect");
		return -EINVAL;
	}

	tmp = bflb_soft_crc32(0, (uint8_t *)(&header.flash_cfg), sizeof(struct bflb_header_flash_cfg));
	if (tmp != header.flash_cfg_crc) {
		LOG_ERR("Flash data crc is incorrect %d vs %d", tmp, header.flash_cfg_crc);
		return -EINVAL;
	}
	flash_bflb_xip_memcpy((uint8_t *)&(header.flash_cfg),
			      (uint8_t *)&(data->controller->flash_header_cfg),
			      sizeof(struct bflb_header_flash_cfg));

	return 0;
}

/* The boot bank (bank1) has its settings provided to the bootrom via the boot header */
static int flash_bflb_init_bootbank(struct flash_bflb_data *data)
{
	int ret;

	if (flash_bflb_is_in_xip(data, &flash_bflb_init_bootbank)) {
		return -ENOTSUP;
	}

	ret = flash_bflb_header_fetch(data);
	if (ret < 0) {
		return ret;
	}

	if (!data->controller->override_bank1) {
		flash_bflb_set_default_read_header(data);
		data->cfg.page_size = data->controller->flash_header_cfg.page_size;
		data->cfg.sector_size =
			data->controller->flash_header_cfg.sector_size * 1024;

	}
	memcpy(data->cfg.cmd.read_reg,
		data->controller->flash_header_cfg.read_reg_cmd,
		sizeof(uint8_t) * 4);
	memcpy(data->cfg.cmd.write_reg,
		data->controller->flash_header_cfg.write_reg_cmd,
		sizeof(uint8_t) * 4);
	/* Continuous read supported */
	if ((data->controller->flash_header_cfg.c_read_support & 0x1) != 0
		/* Continuous read enabled */
		&& (data->controller->flash_header_cfg.c_read_support & 0x2) == 0)
	{
		if (data->cfg.cmd.contread_on == 0) {
			data->cfg.cmd.contread_on =
				data->controller->flash_header_cfg.c_read_mode;
		}
		data->cfg.cmd.contread_off = data->controller->flash_header_cfg.c_rexit;
	}
	data->cfg.cmd.burstwrap = data->controller->flash_header_cfg.burst_wrap_cmd;
	data->cfg.cmd.burstwrap_dmycy =
		data->controller->flash_header_cfg.burst_wrap_cmd_dmy_clk;
	data->cfg.cmd.burstwrap_on_data =
		data->controller->flash_header_cfg.burst_wrap_data;
	data->cfg.cmd.burstwrap_off_data =
		data->controller->flash_header_cfg.de_burst_wrap_data;
	data->cfg.cmd.write_enable = data->controller->flash_header_cfg.write_enable_cmd;
	data->cfg.cmd.page_program = data->controller->flash_header_cfg.page_program_cmd;
	data->cfg.cmd.quad_page_program =
		data->controller->flash_header_cfg.qpage_program_cmd;
	data->cfg.cmd.sector_erase = data->controller->flash_header_cfg.sector_erase_cmd;
	data->cfg.cmd.block_erase = data->controller->flash_header_cfg.blk32_erase_cmd;
#if defined(CONFIG_SOC_SERIES_BL61X)
	data->cfg.cmd.enter_32bits_addr =
		data->controller->flash_header_cfg.enter_32bits_addr_cmd;
	data->cfg.cmd.exit_32bits_addr =
		data->controller->flash_header_cfg.exit_32bits_addr_cmd;
#endif
	data->cfg.cmd.reset_enable = data->controller->flash_header_cfg.reset_en_cmd;
	data->cfg.cmd.reset = data->controller->flash_header_cfg.reset_cmd;
	data->cfg.cmd.release_powerdown =
		data->controller->flash_header_cfg.release_powerdown;

	data->cfg.reg.write_enable_index =
		data->controller->flash_header_cfg.wr_enable_index;
	data->cfg.reg.write_enable_bit =
		data->controller->flash_header_cfg.wr_enable_bit;
	data->cfg.reg.write_enable_read_len =
		data->controller->flash_header_cfg.wr_enable_read_reg_len;

	data->cfg.reg.quad_enable_index =
		data->controller->flash_header_cfg.qe_index;
	data->cfg.reg.quad_enable_bit =
		data->controller->flash_header_cfg.qe_bit;
	data->cfg.reg.quad_enable_read_len =
		data->controller->flash_header_cfg.qe_read_reg_len;
	data->cfg.reg.quad_enable_write_len =
		data->controller->flash_header_cfg.qe_write_reg_len;

	data->cfg.reg.busy_index =
		data->controller->flash_header_cfg.busy_index;
	data->cfg.reg.busy_bit =
		data->controller->flash_header_cfg.busy_bit;
	data->cfg.reg.busy_read_len =
		data->controller->flash_header_cfg.busy_read_reg_len;

	return 0;
}

static void SF_Ctrl_Set_IO_Delay(struct flash_bflb_data *data, uint8_t doDelay, uint8_t diDelay, uint8_t oeDelay)
{
	uint32_t tmp = 0;
	uint32_t offset = 0;

	if (data->pad == PAD1) {
		offset = SF_CTRL_BASE + SF_CTRL_IF_IO_DLY_1_OFFSET;
	} else if (data->pad == PAD2) {
		offset = SF_CTRL_BASE + SF_CTRL_IF_IO_DLY_2_OFFSET;
	} else {
		offset = SF_CTRL_BASE + SF_CTRL_IF_IO_DLY_3_OFFSET;
	}

	/* Set do di and oe delay */
	tmp = FLASH_READ32(offset + SF_CTRL_IO_DLY_1_OFFSET);
	tmp &= ~SF_CTRL_IO_0_DO_DLY_SEL_MSK;
	tmp |= (uint32_t)doDelay << SF_CTRL_IO_0_DO_DLY_SEL_POS;
	tmp &= ~SF_CTRL_IO_0_DI_DLY_SEL_MSK;
	tmp |= (uint32_t)diDelay << SF_CTRL_IO_0_DI_DLY_SEL_POS;
	tmp &= ~SF_CTRL_IO_0_OE_DLY_SEL_MSK;
	tmp |= (uint32_t)oeDelay << SF_CTRL_IO_0_OE_DLY_SEL_POS;
	FLASH_WRITE32(tmp, offset + SF_CTRL_IO_DLY_1_OFFSET);

	tmp = FLASH_READ32(offset + SF_CTRL_IO_DLY_2_OFFSET);
	tmp &= ~SF_CTRL_IO_0_DO_DLY_SEL_MSK;
	tmp |= (uint32_t)doDelay << SF_CTRL_IO_0_DO_DLY_SEL_POS;
	tmp &= ~SF_CTRL_IO_0_DI_DLY_SEL_MSK;
	tmp |= (uint32_t)diDelay << SF_CTRL_IO_0_DI_DLY_SEL_POS;
	tmp &= ~SF_CTRL_IO_0_OE_DLY_SEL_MSK;
	tmp |= (uint32_t)oeDelay << SF_CTRL_IO_0_OE_DLY_SEL_POS;
	FLASH_WRITE32(tmp, offset + SF_CTRL_IO_DLY_2_OFFSET);

	tmp = FLASH_READ32(offset + SF_CTRL_IO_DLY_3_OFFSET);
	tmp &= ~SF_CTRL_IO_0_DO_DLY_SEL_MSK;
	tmp |= (uint32_t)doDelay << SF_CTRL_IO_0_DO_DLY_SEL_POS;
	tmp &= ~SF_CTRL_IO_0_DI_DLY_SEL_MSK;
	tmp |= (uint32_t)diDelay << SF_CTRL_IO_0_DI_DLY_SEL_POS;
	tmp &= ~SF_CTRL_IO_0_OE_DLY_SEL_MSK;
	tmp |= (uint32_t)oeDelay << SF_CTRL_IO_0_OE_DLY_SEL_POS;
	FLASH_WRITE32(tmp, offset + SF_CTRL_IO_DLY_3_OFFSET);

	tmp = FLASH_READ32(offset + SF_CTRL_IO_DLY_4_OFFSET);
	tmp &= ~SF_CTRL_IO_0_DO_DLY_SEL_MSK;
	tmp |= (uint32_t)doDelay << SF_CTRL_IO_0_DO_DLY_SEL_POS;
	tmp &= ~SF_CTRL_IO_0_DI_DLY_SEL_MSK;
	tmp |= (uint32_t)diDelay << SF_CTRL_IO_0_DI_DLY_SEL_POS;
	tmp &= ~SF_CTRL_IO_0_OE_DLY_SEL_MSK;
	tmp |= (uint32_t)oeDelay << SF_CTRL_IO_0_OE_DLY_SEL_POS;
	FLASH_WRITE32(tmp, offset + SF_CTRL_IO_DLY_4_OFFSET);
}

static int flash_bflb_init(const struct device *dev)
{
	const struct flash_bflb_config *cfg = dev->config;
	struct flash_bflb_data *data = dev->data;
	unsigned int locker;
	int ret;
	uint8_t jedec_id[3];
	volatile uint32_t disp_1, disp_2, tmp;


	k_mutex_init(&data->sahb_mutex);

	if (data->bank == BANK1) {
		ret = flash_bflb_init_bootbank(data);
		if (ret != 0) {
			return ret;
		}
	} else {
		ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
		if (ret != 0) {
			return ret;
		}

		if (data->cfg.cmd.auto_read == BFLB_FLASH_AUTO_READ_DEFAULT
		    && data->cfg.cmd.auto_read_dmycy == BFLB_FLASH_AUTO_READ_DMYCY_DEFAULT)
		flash_bflb_set_default_read_default(data);
		/* Enable bank2 */
		tmp = FLASH_READ32(data->reg + SF_CTRL_2_OFFSET);
		tmp |= SF_CTRL_SF_IF_BK2_EN_MSK;
		tmp |= SF_CTRL_SF_IF_BK2_MODE_MSK;
		FLASH_WRITE32(tmp, data->reg + SF_CTRL_2_OFFSET);

		//SF_Ctrl_Set_IO_Delay(data, 0, 1, 0);

		// tmp = *(volatile uint32_t *)(SF_CTRL_BASE + SF_CTRL_SF_IF_IAHB_12_OFFSET);
		// tmp |= SF_CTRL_SF2_IF_READ_DLY_SRC_MSK;
		// tmp &= ~SF_CTRL_SF2_IF_READ_DLY_EN_MSK;
		// tmp &= ~SF_CTRL_SF2_IF_READ_DLY_N_MSK;
		// tmp |= 0 << SF_CTRL_SF2_IF_READ_DLY_N_POS;
		// tmp &= ~SF_CTRL_SF2_CLK_SF_RX_INV_SEL_MSK;
		// tmp &= ~SF_CTRL_SF2_CLK_SF_RX_INV_SRC_MSK;
		// *(volatile uint32_t *)(SF_CTRL_BASE + SF_CTRL_SF_IF_IAHB_12_OFFSET) = tmp;
  //
		// tmp = FLASH_READ32(data->reg + SF_CTRL_3_OFFSET);
		// tmp |= SF_CTRL_SF_CMDS_2_EN_MSK;
		// tmp &= ~SF_CTRL_SF_CMDS_2_WRAP_MODE_MSK;
		// tmp |= 2U << SF_CTRL_SF_CMDS_2_WRAP_MODE_POS;
		// tmp &= ~SF_CTRL_SF_CMDS_2_WRAP_LEN_MSK;
		// tmp |= 2U << SF_CTRL_SF_CMDS_2_WRAP_LEN_POS;
		// FLASH_WRITE32(tmp, data->reg + SF_CTRL_3_OFFSET);
  //
		// tmp = FLASH_READ32(data->reg + SF_CTRL_SF2_IF_IO_DLY_0_OFFSET);
		// tmp &= ~SF_CTRL_SF2_CLK_OUT_DLY_SEL_MSK;
		// tmp |= 0U << SF_CTRL_SF2_CLK_OUT_DLY_SEL_POS;
		// tmp &= ~SF_CTRL_SF2_CS_DLY_SEL_MSK;
		// tmp |= 0U << SF_CTRL_SF2_CS_DLY_SEL_POS;
		// tmp &= ~SF_CTRL_SF2_DQS_DI_DLY_SEL_MSK;
		// tmp |= 2U << SF_CTRL_SF2_DQS_DI_DLY_SEL_POS;
		// FLASH_WRITE32(tmp, data->reg + SF_CTRL_SF2_IF_IO_DLY_0_OFFSET);

	}

	LOG_DBG("%s: pad %d spi_modes: %d %d auto_read: %x",
		data->bank == BANK2 ? "bank 2" : "bank 1",
		data->pad,
		data->cfg.auto_spi_mode, data->cfg.manual_spi_mode,
		data->cfg.cmd.auto_read);

	locker = irq_lock();

	ret = flash_bflb_save_xip_state(dev);
	if (ret != 0) {
		irq_unlock(locker);
		return ret;
	}

	flash_bflb_reset(data);

	ret = flash_bflb_get_jedec_id_internal(data, jedec_id);
	if (ret != 0) {
		goto exit_nxip_bad;
	}

	ret = flash_bflb_restore_xip_state(data);
	irq_unlock(locker);

	if (*jedec_id == 0) {
		LOG_ERR("Could not get JEDEC ID for %s, unable to initialize",
			data->bank == BANK2 ? "bank 2" : "bank 1");
		return -EIO;
	}

	if (*data->cfg.jedec_id == 0) {
		flash_bflb_xip_memcpy(jedec_id, data->cfg.jedec_id, 3);
	}

	disp_1 = 0;
	disp_2 = 0;
	flash_bflb_xip_memcpy(data->cfg.jedec_id, (uint8_t *)&disp_1, 3);
	flash_bflb_xip_memcpy(jedec_id, (uint8_t *)&disp_2, 3);

	if (disp_1 != disp_2) {
		disp_1 = sys_be32_to_cpu(disp_1) >> 8;
		disp_2 = sys_be32_to_cpu(disp_2) >> 8;
		LOG_WRN("JEDEC ID (%x) does not match device's (%x)", disp_1, disp_2);
	}

	return ret;

exit_nxip_bad:
	flash_bflb_restore_xip_state(data);
	irq_unlock(locker);

	return ret;
}

static DEVICE_API(flash, flash_bflb_api) = {
	.read = flash_bflb_read,
	.write = flash_bflb_write,
	.erase = flash_bflb_erase,
	.get_parameters = flash_bflb_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_bflb_page_layout,
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
#if defined(CONFIG_FLASH_JESD216_API)
	.sfdp_read = flash_bflb_read_sfdp,
	.read_jedec_id = flash_bflb_get_jedec_id,
#endif
};

#define FLASH_BFLB_DEVICE_SET_CMDS(_n)							\
	.cfg.auto_spi_mode = DT_PROP(_n, spi_bus_mode),					\
	.cfg.manual_spi_mode = BUS_NIO,							\
	.cfg.cmd.auto_read = DT_PROP_OR(_n, read_command,				\
					BFLB_FLASH_AUTO_READ_DEFAULT),			\
	.cfg.cmd.auto_read_dmycy = DT_PROP_OR(_n, read_dummy_cycles,			\
					      BFLB_FLASH_AUTO_READ_DMYCY_DEFAULT),	\
	.cfg.cmd.auto_write = DT_PROP_OR(_n, write_command, 0),				\
	.cfg.cmd.auto_write_dmycy = DT_PROP_OR(_n, write_dummy_cycles, 0),		\
	.cfg.cmd.manual_read = SPI_NOR_CMD_READ,					\
	.cfg.cmd.manual_read_dmycy = 0,							\
	.cfg.cmd.read_reg = { SPI_NOR_CMD_RDSR, SPI_NOR_CMD_RDSR2,			\
			      SPI_NOR_CMD_RDSR3, 0 },					\
	.cfg.cmd.write_reg = { SPI_NOR_CMD_WRSR, SPI_NOR_CMD_WRSR2,			\
			      SPI_NOR_CMD_WRSR3, 0 },					\
	.cfg.cmd.contread_on = DT_PROP_OR(_n, continuous_read_command, 0),		\
	.cfg.cmd.contread_off = 0xff,							\
	.cfg.cmd.burstwrap = 0,								\
	.cfg.cmd.write_enable = SPI_NOR_CMD_WREN,					\
	.cfg.cmd.page_program = SPI_NOR_CMD_PP,						\
	.cfg.cmd.quad_page_program = SPI_NOR_CMD_PP_1_1_4,				\
	.cfg.cmd.sector_erase = SPI_NOR_CMD_SE,						\
	.cfg.cmd.block_erase = SPI_NOR_CMD_BE_32K,					\
	.cfg.cmd.enter_32bits_addr = SPI_NOR_CMD_4BA,					\
	.cfg.cmd.exit_32bits_addr = 0xe9,						\
	.cfg.cmd.reset_enable = SPI_NOR_CMD_RESET_EN,					\
	.cfg.cmd.reset = SPI_NOR_CMD_RESET_MEM,						\
	.cfg.cmd.powerdown = SPI_NOR_CMD_DPD,						\
	.cfg.cmd.release_powerdown = SPI_NOR_CMD_RDPD,					\
	.cfg.size = DT_REG_SIZE(_n),							\
	.cfg.page_size = DT_PROP(_n, write_block_size),					\
	.cfg.sector_size = DT_PROP(_n, erase_block_size),				\
	.cfg.block_size = KB(32),							\
	.cfg.jedec_id = DT_PROP_OR(_n, jedec_id, 0),					\
	.cfg.reg = {0},

#define FLASH_BFLB_DEVICE_XIP_END(_a)							\
	(_a == BFLB_XIP_BASE_BANK1 ? BFLB_XIP_END_BANK1 : BFLB_XIP_END_BANK2)

#define FLASH_BFLB_DEVICE_IS_BANK2(_n) (DT_REG_ADDR(_n) == BFLB_XIP_BASE_BANK2)

#define FLASH_BFLB_DEVICE_DEFINE(_n, _controller_n)					\
	BUILD_ASSERT(DT_REG_ADDR(_n) == BFLB_XIP_BASE_BANK1				\
		     || FLASH_BFLB_DEVICE_IS_BANK2(_n),					\
		     "Device address must match one of the mappings");			\
	BUILD_ASSERT(DT_REG_ADDR(_n) == BFLB_XIP_BASE_BANK1				\
		     ? DT_PROP_OR(_n, write_command, 0) == 0 : true,			\
		     "Bank 1 does not support writing.");				\
	BUILD_ASSERT(FLASH_BFLB_DEVICE_IS_BANK2(_n) ?					\
		(DT_NODE_HAS_PROP(_n, pinctrl_0)) : true, "Bank 2 must have pinctrl");	\
	PINCTRL_DT_DEFINE(_n);								\
	static struct flash_bflb_data flash_bflb_data_##_n = {				\
		.reg = DT_REG_ADDR(_controller_n),					\
		FLASH_BFLB_DEVICE_SET_CMDS(_n)						\
		.controller = &flash_bflb_controller_data_##_controller_n,		\
		.nxip_message = NXIP_MSG_NONE,						\
		.bank = (DT_REG_ADDR(_n) == BFLB_XIP_BASE_BANK1 ? BANK1 : BANK2),	\
		.pad = DT_PROP(_n, sf_pad),						\
		.xip_base = DT_REG_ADDR(_n),						\
		.xip_end = FLASH_BFLB_DEVICE_XIP_END(DT_REG_ADDR(_n)),			\
		.layout.pages_count = DT_REG_SIZE(_n) / DT_PROP(_n, erase_block_size),	\
		.layout.pages_size = DT_PROP(_n, erase_block_size),			\
		.parameters.write_block_size = DT_PROP(_n, write_block_size),		\
		.parameters.erase_value = ERASE_VALUE,					\
	};										\
	static const struct flash_bflb_config flash_bflb_config_##_n = {		\
		.pincfg = PINCTRL_DT_DEV_CONFIG_GET(_n),				\
	};										\
	DEVICE_DT_DEFINE(_n, flash_bflb_init, NULL,					\
			      &flash_bflb_data_##_n,					\
			      &flash_bflb_config_##_n, POST_KERNEL,			\
			      CONFIG_FLASH_INIT_PRIORITY,				\
			      &flash_bflb_api);

#define FLASH_BFLB_CONTROLLER_INIT(_n)							\
	static struct flash_bflb_controller_data flash_bflb_controller_data_##_n = {	\
		.override_bank1 = DT_PROP(_n, override_bank1),				\
	};										\
	DT_FOREACH_CHILD_STATUS_OKAY_VARGS(_n, FLASH_BFLB_DEVICE_DEFINE, _n)		\
	BUILD_ASSERT(DT_CHILD_NUM(_n) <= 2, "Only 2 banks available");			\
	BUILD_ASSERT(DT_CHILD_NUM(_n) > 0, "Bank 1 must be configured");


BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1, "There must be only one sf-controller");

DT_FOREACH_STATUS_OKAY(DT_DRV_COMPAT, FLASH_BFLB_CONTROLLER_INIT)
