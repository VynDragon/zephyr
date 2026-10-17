/*
 * Copyright (c) 2026 MASSDRIVER EI (massdriver.space)
 *
 * Reimplementations of blob functions that need to be replaced on BL808 to
 * allow operating the IEEE802.15.4 MAC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bl808_ieee802154, LOG_LEVEL_ERR);

#include <bouffalolab/common/lmac154/lmac154.h>
#include <bouffalolab/common/lmac154/lmac154_fpt.h>

/* IEEE802.15.4 terms:
 * PSDU: PHY Service Data Unit
 * MPDU: MAC Protocol Data Unit
 * FCS: Frame Check Sequence
 */

#define LMAC154_BASE		0x2c000000U
#define LMAC154_TX_FIFO		0x200U
#define LMAC154_RX_FIFO		0x400U
#define LMAC154_TX_STATUS	0x2fcU
#define LMAC154_RX_STATUS	0x4fcU
#define LMAC154_TX_RATE		0x300U
#define LMAC154_RX_RATE		0x500U
#define LMAC154_FRPEND		0x584U

/* Length is [31:24] in status register */
#define LMAC154_STATUS_LEN	GENMASK(31, 24)
#define LMAC154_RX_STATUS_CRC	GENMASK(23, 8)
/* lmac154_data_rate_t */
#define LMAC154_RX_RATE_MSK	GENMASK(1, 0)
/* Opaque values correlated with lmac154_data_rate_t */
#define LMAC154_TX_RATE_UMSK	GENMASK(31, 10)
#define LMAC154_TX_RATE_250K	0xa7U
#define LMAC154_TX_RATE_500K	0x1e1U
#define LMAC154_TX_RATE_1M	0x228U
#define LMAC154_TX_RATE_2M	0x3f6U
#define LMAC154_TX_RATE_IDX	GENMASK(9, 8)

#define LMAC154_FRPEND_MODE	GENMASK(1, 0)

/* Associated PSDU size with rate over 250K */
#define LMAC154_MAX_LEN_LOW	0x7fU
#define LMAC154_MAX_LEN_HIGH	0xffU
#define LMAC154_FCS_LEN		2U
#define LMAC154_MIN_TX_LEN	(1U + LMAC154_FCS_LEN)
#define LMAC154_MIN_RX_LEN	(3U + LMAC154_FCS_LEN)

#define LMAC154_TX_FIFO_SIZE	(LMAC154_TX_STATUS - LMAC154_TX_FIFO)
#define LMAC154_RX_FIFO_SIZE	(LMAC154_RX_STATUS - LMAC154_RX_FIFO)

#define LMAC154_SUCCESS		0
#define LMAC154_FAIL		(-1)

/*
 * TX power register shared with BL616's rf_pri_set_154_tx_power():
 * dBm value lives in bits [31:26].
 */
#define BL808_RF_154_TXPWR_REG		0x20001700U
#define BL808_RF_154_TXPWR_MASK		GENMASK(31, 26)

/* PHY auto-rx for 802.15.4, either 0x2824 (BL616) or 0x1824 (BL702) */
#define LMAC154_AUTORX_LEN_REG		0x20002824U
#define LMAC154_AUTORX_LEN		GENMASK(31, 24)
#define LMAC154_AUTORX_LEN_HIGH_BIT	BIT(13)

/* Pull word-copy function from shim */
uint32_t *arch_memcpy4(uint32_t *dst, const uint32_t *src, uint32_t n);

void wl_rf_set_154_tx_power(int8_t dbm)
{
	uint32_t tmp = sys_read32(BL808_RF_154_TXPWR_REG);

	tmp &= ~BL808_RF_154_TXPWR_MASK;
	tmp |= FIELD_PREP(BL808_RF_154_TXPWR_MASK, (uint32_t)dbm);
	sys_write32(tmp, BL808_RF_154_TXPWR_REG);
}

/* Copy PSDU without FCS into the TX FIFO */
int LMAC154_SET_MPDU(const uint8_t *frame, size_t len)
{
	uint32_t tmp = sys_read32(LMAC154_BASE + LMAC154_TX_RATE);
	size_t max_len = LMAC154_MAX_LEN_LOW;

	/* Data must be copied word-aligned */
	if (((uintptr_t)frame) % sizeof(uint32_t)) {
		LOG_ERR_RATELIMIT("SET_MPDU: Misaligned PSDU");
		return LMAC154_FAIL;
	}

	if ((tmp & LMAC154_TX_RATE_IDX) != 0) {
		max_len = LMAC154_MAX_LEN_HIGH;
	}

	if (len < LMAC154_MIN_TX_LEN || len > (max_len - LMAC154_FCS_LEN)
	    || ROUND_UP(len, sizeof(uint32_t)) > LMAC154_TX_FIFO_SIZE) {
		LOG_ERR_RATELIMIT("SET_MPDU: Invalid PSDU length (%zu)", len);
		return LMAC154_FAIL;
	}

	arch_memcpy4((uint32_t *)(LMAC154_BASE + LMAC154_TX_FIFO), (const uint32_t *)frame,
		     DIV_ROUND_UP(len, sizeof(uint32_t)));

	tmp = sys_read32(LMAC154_BASE + LMAC154_TX_STATUS);
	tmp &= ~LMAC154_STATUS_LEN;
	tmp |= FIELD_PREP(LMAC154_STATUS_LEN, len + LMAC154_FCS_LEN);
	sys_write32(tmp, LMAC154_BASE + LMAC154_TX_STATUS);

	return LMAC154_SUCCESS;
}

/* Copy PSDU with FCS from the RX FIFO */
int LMAC154_GET_MPDU(uint8_t *frame, uint8_t *len)
{
	uint32_t status = sys_read32(LMAC154_BASE + LMAC154_RX_STATUS);
	size_t max_len = LMAC154_MAX_LEN_LOW;
	size_t have_len = FIELD_GET(LMAC154_STATUS_LEN, status);
	size_t fifo_len;

	/* Always report even when failed */
	*len = (uint8_t)have_len;

	LOG_ERR_RATELIMIT("GET_MPDU: %zu", have_len);

	/* Data must be copied word-aligned */
	if (((uintptr_t)frame) % sizeof(uint32_t)) {
		LOG_ERR_RATELIMIT("GET_MPDU: Misaligned PSDU");
		return LMAC154_FAIL;
	}

	if ((sys_read32(LMAC154_AUTORX_LEN_REG) & LMAC154_AUTORX_LEN_HIGH_BIT) != 0) {
		max_len = LMAC154_MAX_LEN_HIGH;
	}

	/* FIFO holds the frame without FCS */
	fifo_len = have_len - LMAC154_FCS_LEN;

	if (have_len < LMAC154_MIN_RX_LEN || have_len > max_len
	    || ROUND_UP(fifo_len, sizeof(uint32_t)) > LMAC154_RX_FIFO_SIZE) {
		LOG_ERR_RATELIMIT("GET_MPDU: Invalid PSDU length (%zu)", have_len);
		return LMAC154_FAIL;
	}

	arch_memcpy4((uint32_t *)frame, (uint32_t *)(LMAC154_BASE + LMAC154_RX_FIFO),
		     DIV_ROUND_UP(fifo_len, sizeof(uint32_t)));

	sys_put_le16((uint16_t)FIELD_GET(LMAC154_RX_STATUS_CRC, status), &frame[fifo_len]);

	return LMAC154_SUCCESS;
}

// void lmac154_triggerTx(uint8_t *data_ptr, uint8_t length, uint8_t csma)
// {
// 	/* Stub, we should not allow using this */
// 	LOG_ERR_RATELIMIT("lmac154_triggerTx is unsupported on BL808");
// 	ARG_UNUSED(data_ptr);
// 	ARG_UNUSED(length);
// 	ARG_UNUSED(csma);
// }

void lmac154_setTxDataRate(lmac154_data_rate_t rate_in)
{
	uint32_t rate = sys_read32(LMAC154_BASE + LMAC154_TX_RATE);

	rate &= LMAC154_TX_RATE_UMSK;

	switch (rate_in) {
	case LMAC154_DATA_RATE_250K:
		rate |= LMAC154_TX_RATE_250K;
		break;
	case LMAC154_DATA_RATE_500K:
		rate |= LMAC154_TX_RATE_500K;
		break;
	case LMAC154_DATA_RATE_1M:
		rate |= LMAC154_TX_RATE_1M;
		break;
	case LMAC154_DATA_RATE_2M:
		rate |= LMAC154_TX_RATE_2M;
		break;
	default:
		return;
	}

	sys_write32(rate, LMAC154_BASE + LMAC154_TX_RATE);
}

uint8_t lmac154_getRxLength(void)
{
	uint32_t status = sys_read32(LMAC154_BASE + LMAC154_RX_STATUS);

	return FIELD_GET(LMAC154_STATUS_LEN, status);
}

void lmac154_readRxCrc(uint8_t crc[2])
{
	uint32_t status = sys_read32(LMAC154_BASE + LMAC154_RX_STATUS);

	sys_put_le16((uint16_t)FIELD_GET(LMAC154_RX_STATUS_CRC, status), crc);
}

lmac154_data_rate_t lmac154_getRxDataRate(void)
{
	uint32_t rate = sys_read32(LMAC154_BASE + LMAC154_RX_RATE);

	return (lmac154_data_rate_t)FIELD_GET(LMAC154_RX_RATE_MSK, rate);
}

// void lmac154_setFramePendingMode(lmac154_fptMode_t mode)
// {
// 	uint32_t frame_pending = sys_read32(LMAC154_BASE + LMAC154_FRPEND);
//
// 	frame_pending |= FIELD_PREP(LMAC154_FRPEND_MODE, mode);
// 	sys_write32(frame_pending, LMAC154_BASE + LMAC154_FRPEND);
// }

// int lmac154_framePendingResult(uint32_t tout, bool * isFramePended)
// {
//
// }
