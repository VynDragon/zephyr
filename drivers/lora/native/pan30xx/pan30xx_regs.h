/*
 * Copyright (c) 2026 MASSDRIVER EI (massdriver.space)
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_LORA_PAN30XX_PAN30XX_REGS_H_
#define ZEPHYR_DRIVERS_LORA_PAN30XX_PAN30XX_REGS_H_

#include <stdint.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/minmax.h>

#include "pan30xx_hal.h"

/* On Panchip modems, configuration is done via registers spread over pages.
 * For most, the page must first be selected by writing it to the appropriate register,
 * then the targeted register can be set.
 * Some registers are accessible from all pages and can be read and written at any point, this
 * includes the 256 bytes FIFO buffer used for both transmission and reception of a single packet.
 */

/* Registers accessible from all pages */

/* HV reset and page selection register */
#define PAN30XX_SYS				0x00

#define PAN30XX_SYS_PAGE_MSK			GENMASK(1, 0)
#define PAN30XX_SYS_PAGE_0			0x0
#define PAN30XX_SYS_PAGE_1			0x1
#define PAN30XX_SYS_PAGE_2			0x2
#define PAN30XX_SYS_PAGE_3			0x3
/* Set -> Unset -> Read register to reset HV registers */
#define PAN30XX_SYS_RESET			BIT(7)

/* Buffer register */
#define PAN30XX_FIFO				0x01

/* Operational Mode register */
#define PAN30XX_MODE				0x02

#define PAN30XX_MODE_MSK			GENMASK(2, 0)
#define PAN30XX_MODE_DEEP_SLEEP			0x0
#define PAN30XX_MODE_SLEEP			0x1
#define PAN30XX_MODE_STANDBY1			0x2
#define PAN30XX_MODE_STANDBY2			0x3
#define PAN30XX_MODE_STANDBY3			0x4
#define PAN30XX_MODE_TX				0x5
#define PAN30XX_MODE_RX				0x6

/* Power setting register */
#define PAN30XX_POW				0x04

/* Must OR this with registers write */
#define PAN30XX_POW_INIT			0x06
/* Unset to reset, wait for at least 100 uS, Set to un-reset all LV registers */
#define PAN30XX_POW_RESET			BIT(4)
#define PAN30XX_POW_LS_3V_EN			BIT(5)

/* Registers accessible from only a page */

/* MAC Configuration (LV) */
#define PAN30XX_MAC_PAGE			0x0

#define PAN30XX_MAC_TX_STATUS			0x54
#define PAN30XX_MAC_TX_STATUS_FIFO_FULL		BIT(2)
#define PAN30XX_MAC_TX_STATUS_FIFO_FULL_SEL	BIT(1)

#define PAN30XX_MAC_IRQ_MASK			0x58
#define PAN30XX_MAC_IRQ_MASK_RX_TIMEOUT_IMM_EN	BIT(7)
#define PAN30XX_MAC_IRQ_MASK_MAPM		BIT(6)
#define PAN30XX_MAC_IRQ_MASK_RESERVED		BIT(5)
#define PAN30XX_MAC_IRQ_MASK_RX_PLHD_DONE	BIT(4)
#define PAN30XX_MAC_IRQ_MASK_RX_DONE		BIT(3)
#define PAN30XX_MAC_IRQ_MASK_CRC_ERR		BIT(2)
#define PAN30XX_MAC_IRQ_MASK_RX_TIMEOUT		BIT(1)
#define PAN30XX_MAC_IRQ_MASK_TX_DONE		BIT(0)

#define PAN30XX_MAC_RX_RESET			0x5b
#define PAN30XX_MAC_RX_RESET_RX_DONE_EN		BIT(2)
#define PAN30XX_MAC_RX_RESET_CR_ERR_EN		BIT(1)
#define PAN30XX_MAC_RX_RESET_HEADER_ERR_EN	BIT(0)

#define PAN30XX_MAC_CAL				0x5d
#define PAN30XX_MAC_CAL_2POINT_EN		BIT(0)

#define PAN30XX_MAC_IRQ_STATUS_CLEAR		0x6c
#define PAN30XX_IRQ_EFUSE_DONE			BIT(7)
#define PAN30XX_IRQ_MAPM			BIT(6)
#define PAN30XX_IRQ_RESERVED			BIT(5)
#define PAN30XX_IRQ_RX_PLHD_DONE		BIT(4)
#define PAN30XX_IRQ_RX_DONE			BIT(3)
#define PAN30XX_IRQ_CRC_ERR			BIT(2)
#define PAN30XX_IRQ_RX_TIMEOUT			BIT(1)
#define PAN30XX_IRQ_TX_DONE			BIT(0)

#define PAN30XX_IRQ_ALL				\
	(PAN30XX_IRQ_EFUSE_DONE | PAN30XX_IRQ_MAPM | PAN30XX_IRQ_RX_PLHD_DONE \
	| PAN30XX_IRQ_RX_DONE | PAN30XX_IRQ_CRC_ERR | PAN30XX_IRQ_RX_TIMEOUT | PAN30XX_IRQ_TX_DONE)

#define PAN30XX_MAC_MAPM_PREAMBLE_CNT		0x6d

#define PAN30XX_MAC_MAPM_ADDRX			0x6e

/* GPIO Configuration (LV) */
#define PAN30XX_GPIO_PAGE			0x0

#define PAN30XX_GPIO_FUNC			0x5e
#define PAN30XX_GPIO_FUNC_GPIO10_IS_GPIO	BIT(7)
#define PAN30XX_GPIO_FUNC_GPIO10_IS_TXEN	0
#define PAN30XX_GPIO_FUNC_GPIO11_IS_GPIO	BIT(6)
#define PAN30XX_GPIO_FUNC_GPIO11_IS_CAD		0

#define PAN30XX_GPIO_PULLDOWN_LSB		0x5f
#define PAN30XX_GPIO_PULLDOWN_LSB_GPIO0		BIT(0)
#define PAN30XX_GPIO_PULLDOWN_LSB_GPIO3		BIT(3)

#define PAN30XX_GPIO_PULLDOWN_MSB		0x60
#define PAN30XX_GPIO_PULLDOWN_MSB_GPIO10	BIT(2)
#define PAN30XX_GPIO_PULLDOWN_MSB_GPIO11	BIT(3)

#define PAN30XX_GPIO_PULLUP_LSB			0x61
#define PAN30XX_GPIO_PULLUP_LSB_GPIO0		BIT(0)
#define PAN30XX_GPIO_PULLUP_LSB_GPIO3		BIT(3)

#define PAN30XX_GPIO_PULLUP_MSB			0x62
#define PAN30XX_GPIO_PULLUP_MSB_GPIO10		BIT(2)
#define PAN30XX_GPIO_PULLUP_MSB_GPIO11		BIT(3)

#define PAN30XX_GPIO_IE_LSB			0x63
#define PAN30XX_GPIO_IE_LSB_GPIO0		BIT(0)
#define PAN30XX_GPIO_IE_LSB_GPIO3		BIT(3)

#define PAN30XX_GPIO_IE_MSB			0x64
#define PAN30XX_GPIO_IE_MSB_GPIO10		BIT(2)
#define PAN30XX_GPIO_IE_MSB_GPIO11		BIT(3)

#define PAN30XX_GPIO_OE_LSB			0x65
#define PAN30XX_GPIO_OE_LSB_GPIO0		BIT(0)
#define PAN30XX_GPIO_OE_LSB_GPIO3		BIT(3)

#define PAN30XX_GPIO_OE_MSB			0x66
#define PAN30XX_GPIO_OE_MSB_GPIO10		BIT(2)
#define PAN30XX_GPIO_OE_MSB_GPIO11		BIT(3)

#define PAN30XX_GPIO_OUT_LSB			0x67
#define PAN30XX_GPIO_OUT_LSB_GPIO0		BIT(0)
#define PAN30XX_GPIO_OUT_LSB_GPIO3		BIT(3)

#define PAN30XX_GPIO_OUT_MSB			0x68
#define PAN30XX_GPIO_OUT_MSB_GPIO10		BIT(2)
#define PAN30XX_GPIO_OUT_MSB_GPIO11		BIT(3)

#define PAN30XX_GPIO_IN_LSB			0x74
#define PAN30XX_GPIO_IN_LSB_GPIO0		BIT(0)
#define PAN30XX_GPIO_IN_LSB_GPIO3		BIT(3)

#define PAN30XX_GPIO_IN_MSB			0x75
#define PAN30XX_GPIO_IN_MSB_GPIO10		BIT(2)
#define PAN30XX_GPIO_IN_MSB_GPIO11		BIT(3)

/* Operation Configuration (HV) */
#define PAN30XX_OP_PAGE				0x3

#define PAN30XX_OP_RXTRX_MODE			0x06
#define PAN30XX_OP_RXTRX_MODE_TX_CONTINUOUS	BIT(2)
#define PAN30XX_OP_RXTRX_MODE_TX_SINGLE		0
#define PAN30XX_OP_RXTRX_MODE_RX_CONTINUOUS	BIT(1)
#define PAN30XX_OP_RXTRX_MODE_RX_SINGLE_TIMEOUT	BIT(0)
#define PAN30XX_OP_RXTRX_MODE_RX_SINGLE		0

/* RX Timeout in microseconds */
#define PAN30XX_OP_RX_TIMEOUT			0x07
#define PAN30XX_OP_RX_TIMEOUT_7_0		0x07
#define PAN30XX_OP_RX_TIMEOUT_15_8		0x08

#define PAN30XX_OP_CR_BW			0x0d
#define PAN30XX_OP_CR_BW_BW_MSK			GENMASK(7, 4)
#define PAN30XX_OP_CR_BW_BW_62_5		0x06
#define PAN30XX_OP_CR_BW_BW_125			0x07
#define PAN30XX_OP_CR_BW_BW_250			0x08
#define PAN30XX_OP_CR_BW_BW_500			0x09
#define PAN30XX_OP_CR_BW_CR_MSK			GENMASK(3, 1)
#define PAN30XX_OP_CR_BW_CR_4_5			0x01
#define PAN30XX_OP_CR_BW_CR_4_6			0x02
#define PAN30XX_OP_CR_BW_CR_4_7			0x03
#define PAN30XX_OP_CR_BW_CR_4_8			0x04

#define PAN30XX_OP_SF				0x0e
/* Same as LoRa API and other modems */
#define PAN30XX_OP_SF_MSK			GENMASK(7, 4)
#define PAN30XX_OP_CRC_EN			BIT(3)

/* Higher 4 bits: 0x1 to 0xf, but for SF5 0x1 to 0x3 and SF6 0x1 to 0x7
 * Lower 4 bits: 0x0 to 0xf, but for SF5 0x0 to 0x3 and SF6 0x0 to 0x7
 */
#define PAN30XX_OP_SYNCWORD			0x0f

/* Intermediate frequency */
#define PAN30XX_OP_IF				0x10
#define PAN30XX_OP_IF_7_0			0x10
#define PAN30XX_OP_IF_11_8			0x11

#define PAN30XX_OP_LDRO				0x12
#define PAN30XX_OP_LDRO_INIT			0x16
#define PAN30XX_OP_LDRO_EN			BIT(3)
#define PAN30XX_OP_LDRO_RESERVED_MODE0		(BIT(4) | BIT(2))
#define PAN30XX_OP_LDRO_RESERVED_MODE1		0

#define PAN30XX_OP_PREAMBLE_LEN			0x13
#define PAN30XX_OP_PREAMBLE_LEN_7_0		0x13
#define PAN30XX_OP_PREAMBLE_LEN_15_8		0x14

#define PAN30XX_OP_INTERFACE			0x1a
#define PAN30XX_OP_INTERFACE_3W_SPI		0x83
#define PAN30XX_OP_INTERFACE_SPI		0x03

#define PAN30XX_OP_DCDC				0x24
#define PAN30XX_OP_DCDC_EN			BIT(3)

#define PAN30XX_OP_XTAL				0x26
/* Active crystal oscillator for TCXO */
#define PAN30XX_OP_XTAL_ACTIVE			BIT(7)
#define PAN30XX_OP_XTAL_FAST_STARTUP_EN		BIT(6)
#define PAN30XX_OP_XTAL_DIG_EN			BIT(5)

/* Undocumented Registers ------------------------------------------------------------------------
 * Registers marked as Read-Modify-Write are demonstrated with only the specific bits toggled,
 * Unmarked registers may be written directly.
 */

/* Read-Modify-Write */
#define PAN30XX_IQ_PAGE				0x1
#define PAN30XX_IQ				0x0e
#define PAN30XX_IQ_MSK				GENMASK(6,5)
#define PAN30XX_IQ_NORMAL			BIT(6)
#define PAN30XX_IQ_INVERTED			BIT(5)

/* Read-Modify-Write
 * Turn Off DCDC before enabling PA, restore after disabling PA
 * Enable TX path before PA
 */
#define PAN30XX_PA_PAGE				0x3
#define PAN30XX_PA				0x4f
#define PAN30XX_PA_EN				BIT(3)

/* Read-Modify-Write */
#define PAN30XX_CAD_MODE_PAGE			0x1
#define PAN30XX_CAD_MODE			0x25
#define PAN30XX_CAD_MODE_MSK			GENMASK(6,3)
#define PAN30XX_CAD_MODE_CHIRPIOT		0
#define PAN30XX_CAD_MODE_LORA			0x48
#define PAN30XX_CAD_MODE_DETECT_THRES		GENMASK(1:0)

/* RF calibration efuses */
#define PAN30XX_RF_EF_PAGE			0x2

/* Read-Modify-Write */
#define PAN30XX_RF_EF_LOCKS			0x3e
#define PAN30XX_RF_EF_LOCKS_LOCK_EN		0x8

/* First, unlock efuses by setting PAN30XX_RF_EF_LOCKS_LOCK_EN to 0
 * Input 3 bytes: Pattern Pattern (TargetAddr << 1)
 * Pattern = Mask? Always 0x5aa5
 * Then wait until PAN30XX_IRQ_EFUSE_DONE is set
 * The output is then read off the same register.
 * After operations and setting the calibration registers,
 * PAN30XX_RF_EF_LOCKS_LOCK_EN is set back to 1
 *
 * Known addresses:
 * 0x1c: Apply calibration only if retrieved value is 0x5a?
 * 0x1e: DCDCIMAX, apply at calibration
 * 0x1f: DCDCREF, apply at calibration
 * 0x20: PABIAS, apply at calibration and keep for re-use when changing transmit power, if it is 0,
 * use the value 8 when setting TX power.
 */
#define PAN30XX_RF_EF_REG_IO			0x3b

/* Unsure of usage*/
#define PAN30XX_RF_CALIBRATION_YES_PAGE		0x2
#define PAN30XX_RF_CALIBRATION_YES		0x3d
#define PAN30XX_RF_CALIBRATION_YES_EN		0xfd

/* Written if PABIAS efuse is not 0, otherwise used in power control */
#define PAN30XX_RF_CALIBRATION_PABIAS_PAGE	0x0
#define PAN30XX_RF_CALIBRATION_PABIAS		0x45

/* Reg = DCDCIMAX_EN | (DCDCIMAX & DCDCIMAX_VAL_MSK) */
#define PAN30XX_RF_CALIBRATION_DCDCIMAX_PAGE	0x3
#define PAN30XX_RF_CALIBRATION_DCDCIMAX_EN	0xc0
#define PAN30XX_RF_CALIBRATION_DCDCIMAX_VAL_MSK	0x1f

#define PAN30XX_RF_CALIBRATION_DCDCREF_PAGE	0x3
#define PAN30XX_RF_CALIBRATION_DCDCREF		0x1d

/* AGC setup:
 * Enable AGC
 * Send 40 (calibration?) values, one set for low frequencies, one set for high frequencies
 * Enable AGC calibration (?)
 */
#define PAN30XX_AGC_PAGE			0x2

/* Read-Modify-Write
 * Quoting:
 * - [Page2][0x06][Bit0] equal to 0 means enable AGC function
 * - [Page2][0x06][Bit1] equal to 1 means disable AGC function
 * Suspecting both apply to bit0. Only demonstrated clearing bit A[0]
 */
#define PAN30XX_AGC_ENABLE_1			0x06
#define PAN30XX_AGC_ENABLE_1_DIS		0x01

#define PAN30XX_AGC_CALIBRATION			0x0a

#define PAN30XX_AGC_ENABLE_2			0x34
#define PAN30XX_AGC_ENABLE_2_EN			0xef

/* Transmit power configuration
 * Fetch values to use from provided power tables
 */

/* table LDO value & 0x1 */
#define PAN30XX_TX_POWER_LDO_1_PAGE		0x3
#define PAN30XX_TX_POWER_LDO_1			0x22
#define PAN30XX_TX_POWER_LDO_1_EN		0x01

#define PAN30XX_TX_POWER_RAMP_PAGE		0x0
#define PAN30XX_TX_POWER_RAMP			0x1e

/* table LDO value >> 4 */
#define PAN30XX_TX_POWER_LDO_2_PAGE		0x0
#define PAN30XX_TX_POWER_LDO_2			0x4b

/* Read-Modify-Write */
#define PAN30XX_TX_POWER_PABIAS_TOP_PAGE	0x0
#define PAN30XX_TX_POWER_PABIAS_TOP		0x46
/* Set if table PABias != 0x70 */
#define PAN30XX_TX_POWER_PABIAS_TOP_EN		0x04

/* Value is (table PABias & 0xf0 )| (Efuse PABias - (table PABias & 0x0f)) */
#define PAN30XX_TX_POWER_PABIAS_PAGE		0x0
#define PAN30XX_TX_POWER_PABIAS			0x45

/* Frequency configuration
 * Fetch values to use from provided frequency tables
 */

#define PAN30XX_FREQUENCY_VCO_PAGE		0x0
#define PAN30XX_FREQUENCY_VCO_1			0x40
#define PAN30XX_FREQUENCY_VCO_2			0x41

#define PAN30XX_FREQUENCY_LO_PAGE		0x0
#define PAN30XX_FREQUENCY_LO			0x3d

#define PAN30XX_FREQUENCY_FREQ_PAGE		0x3
#define PAN30XX_FREQUENCY_FREQ			0x09
#define PAN30XX_FREQUENCY_FREQ_7_0		0x09
#define PAN30XX_FREQUENCY_FREQ_15_8		0x0a
#define PAN30XX_FREQUENCY_FREQ_23_16		0x0b
#define PAN30XX_FREQUENCY_FREQ_31_24		0x0c

#define PAN30XX_FREQUENCY_FAB_PAGE		0x3
#define PAN30XX_FREQUENCY_FAB			0x15
#define PAN30XX_FREQUENCY_FAB_FA		0x15
#define PAN30XX_FREQUENCY_FAB_FB		0x16
#define PAN30XX_FREQUENCY_FAB_FB_7_0		0x16
#define PAN30XX_FREQUENCY_FAB_FB_15_8		0x17

/* Reserved accessible from all pages */
#define PAN30XX_RESERVED_THREE			0x03

/* Values ----------------------------------------------------------------------------------------*/

#define PAN30XX_XTAL_FREQ			MHZ(32UL)

/* Rough estimations as the chips do not calibrate and the vendor provides lookup tables */
#define PAN30XX_MAX_POWER			20
#define PAN30XX_MIN_POWER			(-20)

/* Sync Word Values */
#define PAN30XX_SYNC_WORD_PUBLIC		0x34
#define PAN30XX_SYNC_WORD_PRIVATE		0x12

/* Maximum payload size */
#define PAN30XX_MAX_PAYLOAD_LEN			255

/* FIFO size */
#define PAN30XX_FIFO_SIZE			256

/* Transmit power ramp table from PAN30xx SDK
 * These define 22 Power Levels, ranging from about -19.7 to 19.5 dBm
 */

struct pan30xx_power_ramp {
/* Describes Power Amplifier ramp timing and shape */
	uint8_t Ramp;
/* LDO configuration values */
	uint8_t Ldo;
/* PA Bias */
	uint8_t PABias;
};

#define PAN30XX_POWER_LEVEL_CNT	22

static const struct pan30xx_power_ramp pan30xx_power_ramp_table[PAN30XX_POWER_LEVEL_MAX] =
{
	{0x01, 0x01, 0x00},
	{0x03, 0x01, 0x01},
	{0x03, 0xf0, 0x30},
	{0x05, 0x01, 0x81},
	{0x05, 0xa1, 0x81},
	{0x05, 0xf0, 0x81},
	{0x07, 0x01, 0x81},
	{0x05, 0x01, 0x80},
	{0x05, 0x31, 0x80},
	{0x07, 0x01, 0x80},
	{0x07, 0x21, 0x80},
	{0x0b, 0x11, 0x80},
	{0x0b, 0x21, 0x80},
	{0x0b, 0x41, 0x80},
	{0x0b, 0x61, 0x80},
	{0x0b, 0x91, 0x80},
	{0x0b, 0xb1, 0x80},
	{0x0d, 0xb1, 0x80},
	{0x0f, 0xb1, 0x80},
	{0x11, 0x50, 0x80},
	{0x15, 0x30, 0x20},
	{0x15, 0x50, 0x70},
};

/* Frequency band configurations */

struct pan30xx_frequency {
/* Start frequency in Hz (inclusive) */
	uint32_t StartFreq;
/* Stop  frequency in Hz (exclusive) */
	uint32_t StopFreq;
/* VCO configuration parameter */
	uint8_t  VcoParam;
/* Frequency factor parameter */
	uint8_t  FreqFactor;
/* LO parameter */
	uint8_t  LoParam;
};

static const struct pan30xx_frequency pan30xx_frequency_table[] =
{
/* Keep table ordered by frequencies */
	{138330000,  143330000,  0x2a, 0x0c, 0xc0},
	{143330000,  148330000,  0x3a, 0x0c, 0xc0},
	{148330000,  155000000,  0x4a, 0x0c, 0xc0},
	{155000000,  161000000,  0x5a, 0x0c, 0xc0},
	{161000000,  168000000,  0x6a, 0x0c, 0xc0},
	{168000000,  180000000,  0x7a, 0x0c, 0xc0},
	{207500000,  215000000,  0x2a, 0x08, 0xb0},
	{215000000,  222500000,  0x3a, 0x08, 0xb0},
	{222500000,  232500000,  0x4a, 0x08, 0xb0},
	{232500000,  242500000,  0x5a, 0x08, 0xb0},
	{242500000,  252500000,  0x6a, 0x08, 0xb0},
	{252500000,  270000000,  0x7a, 0x08, 0xb0},
	{282500000,  287000000,  0x2a, 0x06, 0xa0},
	{287000000,  297000000,  0x3a, 0x06, 0xa0},
	{297000000,  310000000,  0x4a, 0x06, 0xa0},
	{310000000,  323000000,  0x5a, 0x06, 0xa0},
	{323000000,  337000000,  0x6a, 0x06, 0xa0},
	{337000000,  360000000,  0x7a, 0x06, 0xa0},
	{408000000,  415000000,  0x1a, 0x06, 0x90},
	{415000000,  430000000,  0x2a, 0x06, 0x90},
	{430000000,  445000000,  0x3a, 0x06, 0x90},
	{445000000,  465000000,  0x4a, 0x06, 0x90},
	{465000000,  485000000,  0x5a, 0x06, 0x90},
	{485000000,  505000000,  0x6a, 0x06, 0x90},
	{505000000,  540000000,  0x7a, 0x06, 0x90},
	{810000000,  830000000,  0x1a, 0x02, 0x80},
	{830000000,  860000000,  0x2a, 0x02, 0x80},
	{860000000,  890000000,  0x3a, 0x02, 0x80},
	{890000000,  930000000,  0x4a, 0x02, 0x80},
	{930000000,  970000000,  0x5a, 0x02, 0x80},
	{970000000,  1010000000, 0x6a, 0x02, 0x80},
	{1010000000, 1080000000, 0x7a, 0x02, 0x80},
};

/* (LoParam & 0x70) >> 4 determines VCO divider to use for frequency calculations */
static const uint8_t pan30xx_vco_dividers[] = { 2, 4, 6, 8, 12, 16, 4, 4 };

#define PAN30XX_AGC_CALIBRATION_SIZE	40

/* AGC Calibration values for LF ranges */
static const uint8_t pan30xx_AGC_LF[PAN30XX_AGC_CALIBRATION_SIZE] =
{
	0x06, 0x00, 0xf8, 0x06, 0x06, 0x00, 0xf8, 0x06,
	0x06, 0x00, 0xf8, 0x06, 0x06, 0x00, 0xf8, 0x06,
	0x14, 0xc0, 0xf9, 0x14, 0x22, 0xd4, 0xf9, 0x22,
	0x30, 0xd8, 0xf9, 0x30, 0x3e, 0xde, 0xf9, 0x3e,
	0x0e, 0xff, 0x80, 0x4f, 0x12, 0x80, 0x38, 0x01
};

/* AGC Calibration values for HF ranges */
static const uint8_t pan30xx_AGC_HF[PAN30XX_AGC_CALIBRATION_SIZE] =
{
	0x09, 0x80, 0xf3, 0x09, 0x09, 0x80, 0xf3, 0x09,
	0x09, 0x80, 0xf3, 0x09, 0x09, 0x80, 0xf3, 0x09,
	0x14, 0x06, 0xf0, 0x14, 0x22, 0xc6, 0xf1, 0x22,
	0x31, 0x73, 0xf0, 0x31, 0x3f, 0xde, 0xf1, 0x3f,
	0x0e, 0xff, 0xe0, 0x32, 0x29, 0x80, 0x38, 0x01
};

/* SDK's initialization sequence */
static const struct pan30xx_register_value pan30xx_init_sequence[] = {
	{0, PAN30XX_RESERVED_THREE, 0x1b, 0xff},
	{0, PAN30XX_POW, 0x76, 0xff},
	{0, 0x06, 0x01, 0xff},
	{0, 0x0b, 0x04, 0xff},
	{0, 0x13, 0x04, 0xff},
	{0, 0x11, 0x20, 0xff},
	{0, 0x12, 0x10, 0xff},
	{0, 0x1f, 0x07, 0xff},
	{0, 0x20, 0x07, 0xff},
	{0, 0x24, 0x03, 0xff},
	{PAN30XX_TX_POWER_PABIAS_TOP_PAGE, PAN30XX_TX_POWER_PABIAS_TOP, 0x03, 0xff},
	{0, 0x25, 0x00, 0xff},
	{0, 0x21, 0x07, 0xff},
	{0, 0x22, 0x07, 0xff},
	{0, 0x15, 0x21, 0xff},
	{0, 0x31, 0xd0, 0xff},
	{0, 0x36, 0x66, 0xff},
	{0, 0x37, 0x6b, 0xff},
	{0, 0x38, 0xcc, 0xff},
	{0, 0x39, 0x09, 0xff},
	{0, 0x3c, 0xb4, 0xff},
	{0, 0x3e, 0x42, 0xff},
	{PAN30XX_FREQUENCY_VCO_PAGE, PAN30XX_FREQUENCY_VCO_1, 0x6a, 0xff},
	{PAN30XX_FREQUENCY_VCO_PAGE, PAN30XX_FREQUENCY_VCO_2, 0x06, 0xff},
	{0, 0x42, 0xaa, 0xff},
	{0, 0x48, 0x77, 0xff},
	{0, 0x49, 0x77, 0xff},
	{0, 0x4a, 0x77, 0xff},
	{PAN30XX_TX_POWER_LDO_2_PAGE, PAN30XX_TX_POWER_LDO_2, 0x05, 0xff},
	{0, 0x4f, 0x04, 0xff},
	{0, 0x50, 0xd2, 0xff},
	{PAN30XX_GPIO_PAGE, PAN30XX_GPIO_FUNC, 0x80, 0xff},
	{1, PAN30XX_RESERVED_THREE, 0x1b, 0xff},
	{1, PAN30XX_POW, 0x76, 0xff},
	{1, 0x0b, 0x08, 0xff},
	{1, 0x0f, 0x0a, 0xff},
	{1, 0x19, 0x00, 0xff},
	{1, 0x2f, 0xd0, 0xff},
	{1, 0x43, 0xda, 0xff},
	{2, PAN30XX_RESERVED_THREE, 0x1b, 0xff},
	{2, PAN30XX_POW, 0x76, 0xff},
	{2, 0x2c, 0xc0, 0xff},
	{2, 0x2d, 0x27, 0xff},
	{2, 0x2e, 0x09, 0xff},
	{2, 0x2f, 0x00, 0xff},
	{2, 0x30, 0x10, 0xff},
	{3, PAN30XX_RESERVED_THREE, 0x1b, 0xff},
	{3, PAN30XX_POW, 0x76, 0xff},
	{PAN30XX_FREQUENCY_FREQ_PAGE, PAN30XX_FREQUENCY_FREQ_15_8, 0x0e, 0xff},
	{PAN30XX_FREQUENCY_FREQ_PAGE, PAN30XX_FREQUENCY_FREQ_23_16, 0xcf, 0xff},
	{PAN30XX_FREQUENCY_FREQ_PAGE, PAN30XX_FREQUENCY_FREQ_31_24, 0x19, 0xff},
	{PAN30XX_OP_PAGE, PAN30XX_OP_CR_BW, 0x98, 0xff},
	{PAN30XX_OP_PAGE, PAN30XX_OP_LDRO, PAN30XX_OP_LDRO_INIT, 0xff},
	{PAN30XX_OP_PAGE, PAN30XX_OP_PREAMBLE_LEN, 0x14, 0xff},
	{PAN30XX_FREQUENCY_FAB_PAGE, PAN30XX_FREQUENCY_FAB_FB_7_0, 0xf4, 0xff},
	{PAN30XX_FREQUENCY_FAB_PAGE, PAN30XX_FREQUENCY_FAB_FB_15_8, 0x01, 0xff},
	{3, 0x1f, 0xd9, 0xff},
};

#endif /* ZEPHYR_DRIVERS_LORA_PAN30XX_PAN30XX_REGS_H_ */
