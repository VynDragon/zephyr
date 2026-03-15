/*
 * Copyright (c) 2024-2026 MASSDRIVER EI (massdriver.space)
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The Bouffalo Lab USB V2 controller features:
 *  - Dedicated CX (Control Exchange) engine for EP0
 *  - 8 IN + 8 OUT data endpoints (separate)
 *  - Shared FIFO pool (F0-F3 in base regs, F4-F7 in ext regs)
 *  - Built-in DMA engine for FIFO data transfer
 *  - PHY controlled via PDS registers
 *  - Grouped interrupt architecture (G0=CX, G1=FIFO, G2=device)
 */

#include "udc_common.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/cache.h>
#include <zephyr/sys/clock.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udc_bflb_udc_2, CONFIG_UDC_DRIVER_LOG_LEVEL);

#include <soc.h>
#include <bflb_soc.h>
#include <glb_reg.h>
#include <pds_reg.h>
#include <bouffalolab/common/usb_v2_reg.h>

/* EP0 + 4 data endpoints, fixed for all Bouffalo Lab USB V2 variants */
#define USB_BFLB_V2_NUM_BIDIR_EPS	5
#define USB_BFLB_V2_NUM_MONODIR_EPS	3
/* Number of data FIFOs (F0-F3) */
#define USB_BFLB_V2_NUM_DATA_FIFOS	4U

#define USB_BFLB_V2_SPEED_LOW		1U
#define USB_BFLB_V2_SPEED_FULL		0U
#define USB_BFLB_V2_SPEED_HIGH		2U

#define USB_BFLB_V2_FX_X_MASK		0x3FU
#define USB_BFLB_V2_FX_X_OFFSET		8U

#define USB_BFLB_V2_XPS_X_OFFSET	4U

#define USB_BFLB_V2_HSFIFOCAP		512U

#define USB_BFLB_V2_EP_DIR_IN		0U
#define USB_BFLB_V2_EP_DIR_OUT		1U
#define USB_BFLB_V2_FIFO_DIR_OUT	0U
#define USB_BFLB_V2_FIFO_DIR_IN		1U
#define USB_BFLB_V2_FIFO_DIR_BID	2U
#define USB_BFLB_V2_FIFO_EP_NONE	15U

#define USB_BFLB_V2_TIMER_AFTER_RESET_HS	(0x44CU)
#define USB_BFLB_V2_TIMER_AFTER_RESET_FS	(0x2710U)
#define USB_BFLB_V2_TIMER_AFTER_RESET_T		K_MSEC(100)

#define USB_BFLB_V2_MAXPS_HS			1024

/* CX_COMEND interrupt bit — not defined in the vendor register header */
#define USB_MCX_COMEND_INT (1U << 3)

#define DT_DRV_COMPAT bflb_udc_2

struct udc_bflb_v2_config {
	uint32_t base;
	void (*irq_enable_func)(const struct device *const dev);
	void (*irq_disable_func)(const struct device *const dev);
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	int32_t speed_idx;
	uint8_t zero_buff[USB_BFLB_V2_MAXPS_HS];
};

struct udc_bflb_v2_data {
	/* Per-endpoint last-known transfer direction (true = IN) */
	bool ep_is_in[USB_BFLB_V2_NUM_BIDIR_EPS];
	/* BID FIFO serialization: true when a VDMA is in-flight (1-indexed) */
	bool fifo_active[USB_BFLB_V2_NUM_DATA_FIFOS + 1];
	/* Last OUT VDMA programmed length per FIFO (1-indexed) */
	uint32_t out_vdma_len[USB_BFLB_V2_NUM_DATA_FIFOS + 1];
	/* Setup packet received, pending processing in work queue */
	bool setup_received;
	/* Timepoint until which speed register reads are deferred */
	k_timepoint_t reset_expiration;
	/* Setup packet temporary storage */
	uint32_t setup_packet[2];
};

/* Work queue event types for deferred USB processing */
enum udc_bflb_v2_ev_type {
	/* Start the next queued transfer on an endpoint */
	UDC_BFLB_V2_EVT_XFER,
	/* VDMA complete for control (CX) FIFO */
	UDC_BFLB_V2_EVT_CTRL_END,
	/* VDMA complete for a data endpoint FIFO */
	UDC_BFLB_V2_EVT_END,
};

struct udc_bflb_v2_ev {
	const struct device *dev;
	uint8_t ep_addr;
	struct k_work_delayable work;
	enum udc_bflb_v2_ev_type event;
};

K_MEM_SLAB_DEFINE(udc_bflb_v2_ev_slab, sizeof(struct udc_bflb_v2_ev),
		  CONFIG_UDC_BFLB_V2_EVENT_COUNT, sizeof(void *));

static void udc_bflb_v2_ev_submit(const struct device *const dev,
				  const uint8_t ep_addr,
				  const enum udc_bflb_v2_ev_type event,
				  k_timeout_t delay);

static enum udc_bus_speed udc_bflb_v2_device_speed(const struct device *const dev)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	struct udc_bflb_v2_data *const priv = udc_get_private(dev);
	uint32_t speed;

	/* Reset or init ongoing, result would be incorrect */
	while (!sys_timepoint_expired(priv->reset_expiration)) {
		k_msleep(1);
	}

	speed = sys_read32(cfg->base + USB_OTG_CSR_OFFSET);
	speed &= USB_SPD_TYP_HOV_POV_MASK;
	speed = speed >> USB_SPD_TYP_HOV_POV_SHIFT;

	if (speed == USB_BFLB_V2_SPEED_FULL) {
		return UDC_BUS_SPEED_FS;
	} else if (speed == USB_BFLB_V2_SPEED_HIGH) {
		return UDC_BUS_SPEED_HS;
	} else {
		return UDC_BUS_UNKNOWN;
	}

	return UDC_BUS_UNKNOWN;
}

static bool udc_bflb_v2_fifo_is_double(const struct device *const dev, const uint8_t fifo)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = sys_read32(cfg->base + USB_DEV_FCFG_OFFSET);
	if ((tmp & (USB_BLKSZ_F0 << ((fifo - 1U) * USB_BFLB_V2_FX_X_OFFSET))) != 0) {
		return true;
	}

	return false;
}

static uint8_t udc_bflb_v2_fifo_to_ep(const struct device *const dev, const uint8_t fifo)
{
#ifdef CONFIG_UDC_BFLB_V2_FIFO_BIDIR
	if (udc_bflb_v2_fifo_is_double(dev, fifo)) {
		if (fifo < 3U) {
			return 1U;
		}
		return 2U;
	}

	return fifo;
#else
	if (udc_bflb_v2_fifo_is_double(dev, fifo)) {
		return 1U;
	}
	if (fifo < 3U) {
		return 1U;
	}
	return 2U;
#endif
}

static void udc_bflb_v2_ctrl_ack(const struct device *const dev)
{
	uint32_t tmp;
	const struct udc_bflb_v2_config *const cfg = dev->config;

	tmp = sys_read32(cfg->base + USB_DEV_CXCFE_OFFSET);
	tmp |= USB_CX_DONE;
	sys_write32(tmp, cfg->base + USB_DEV_CXCFE_OFFSET);
}

static void udc_bflb_v2_ep_ack(const struct device *const dev, const uint8_t ep_idx)
{
	uint32_t tmp;
	const struct udc_bflb_v2_config *const cfg = dev->config;

	tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET
			 + (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
	tmp |= USB_TX0BYTE_IEP1;
	sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET
		    + (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
}

static void udc_bflb_v2_fifo_configure(const struct device *const dev,
				       const uint8_t fifo_idx,
				       struct udc_ep_config *const config,
				       uint8_t block_num,
				       const bool enabled)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	uint32_t tmp;
	uint8_t ep_type = config->attributes & USB_EP_TRANSFER_TYPE_MASK;

	__ASSERT_NO_MSG(fifo_idx > 0 && fifo_idx <= USB_BFLB_V2_NUM_DATA_FIFOS);

	tmp = sys_read32(cfg->base + USB_DEV_FCFG_OFFSET);
	tmp &= ~(USB_BFLB_V2_FX_X_MASK << ((fifo_idx - 1U) * USB_BFLB_V2_FX_X_OFFSET));
	tmp |= (ep_type << ((fifo_idx - 1U) * USB_BFLB_V2_FX_X_OFFSET + USB_BLK_TYP_F0_SHIFT));
	tmp |= ((block_num - 1U) << ((fifo_idx - 1U) * USB_BFLB_V2_FX_X_OFFSET
				     + USB_BLKNO_F0_SHIFT));
	if (config->mps > USB_BFLB_V2_HSFIFOCAP) {
		tmp |= (USB_BLKSZ_F0 << ((fifo_idx - 1U) * USB_BFLB_V2_FX_X_OFFSET));
	}
	if (enabled) {
		tmp |= (USB_EN_F0 << ((fifo_idx - 1U) * USB_BFLB_V2_FX_X_OFFSET));
	} else {
		tmp &= ~(USB_EN_F0 << ((fifo_idx - 1U) * USB_BFLB_V2_FX_X_OFFSET));
	}
	sys_write32(tmp, cfg->base + USB_DEV_FCFG_OFFSET);
}

static void udc_bflb_v2_ep_set_out_mps(const struct device *const dev,
				       const uint8_t ep_idx,
				       const uint16_t ep_mps)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = sys_read32(cfg->base + USB_DEV_OUTMPS1_OFFSET
		+ (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
	tmp |= USB_RSTG_OEP1;
	sys_write32(tmp, cfg->base + USB_DEV_OUTMPS1_OFFSET
		+ (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_OUTMPS1_OFFSET
		+ (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
	tmp &= ~USB_RSTG_OEP1;
	sys_write32(tmp, cfg->base + USB_DEV_OUTMPS1_OFFSET
		+ (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_OUTMPS1_OFFSET
		+ (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
	tmp &= ~USB_MAXPS_OEP1_MASK;
	tmp |= ep_mps;
	sys_write32(tmp, cfg->base + USB_DEV_OUTMPS1_OFFSET
		+ (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
}

static void udc_bflb_v2_ep_set_in_mps(const struct device *const dev,
					const uint8_t ep_idx,
					const uint16_t ep_mps)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET
		+ (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
	tmp |= USB_RSTG_IEP1;
	sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET
		+ (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET
		+ (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
	tmp &= ~USB_RSTG_IEP1;
	sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET
		+ (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET
		+ (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
	tmp &= ~USB_MAXPS_IEP1_MASK;
	tmp |= ep_mps;
	tmp &= ~USB_TX_NUM_HBW_IEP1_MASK;
	sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET
		+ (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
}

static void udc_bflb_v2_fifo_reset_ctrl(const struct device *const dev)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = sys_read32(cfg->base + USB_DEV_CXCFE_OFFSET);
	tmp |= USB_CX_CLR;
	sys_write32(tmp, cfg->base + USB_DEV_CXCFE_OFFSET);
}

static void udc_bflb_v2_vdma_stop(const struct device *const dev, const uint8_t fifo_idx)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	const uint32_t fifo_off = (fifo_idx - 1U) * USB_BFLB_V2_FX_X_OFFSET;
	uint32_t tmp;
	bool stoppable = false;

	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);
	/* It needs to be stopped */
	if ((tmp & USB_VDMA_START_CXF) != 0) {
		tmp &= ~USB_VDMA_START_CXF;
		/* Undocumented Abort bit */
		tmp |= (1U << 3);
		stoppable = true;
	}
	sys_write32(tmp, cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);
}

static void udc_bflb_v2_fifo_reset(const struct device *const dev, const uint8_t fifo_idx)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	uint32_t tmp;

	__ASSERT_NO_MSG(fifo_idx > 0 && fifo_idx <= USB_BFLB_V2_NUM_DATA_FIFOS);

	tmp = sys_read32(cfg->base + USB_DMA_TFN_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_FIBC0_OFFSET + 4U * (fifo_idx - 1U));
	tmp |= USB_FFRST0_HOV;
	sys_write32(tmp, cfg->base + USB_DEV_FIBC0_OFFSET + 4U * (fifo_idx - 1U));
}

/*
 * Map an endpoint to a FIFO in the EPMAP register.
 * ep_idx/fifo_idx are 1-based; ep_dir: 0=IN, 1=OUT.
 * EPMAP0 covers EP1-4, EPMAP1 covers EP5-8.
 */
static void udc_bflb_v2_ep_setfifo(const struct device *const dev,
				   const uint8_t ep_idx,
				   const uint8_t fifo_idx,
				   const uint8_t ep_dir)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	uint32_t tmp;
	const uint32_t ep_dir_bit = (uint32_t)ep_dir * 4U;

	__ASSERT_NO_MSG(fifo_idx > 0 && fifo_idx <= USB_BFLB_V2_NUM_DATA_FIFOS);
	__ASSERT_NO_MSG(ep_idx > 0 && ep_idx < USB_BFLB_V2_NUM_BIDIR_EPS);

	if (ep_idx < 5U) {
		tmp = sys_read32(cfg->base + USB_DEV_EPMAP0_OFFSET);
		tmp &= ~(0xfU << ((uint32_t)(ep_idx - 1U) * 8U + ep_dir_bit));
		tmp |= ((uint32_t)(fifo_idx - 1U) << ((uint32_t)(ep_idx - 1U) * 8U + ep_dir_bit));
		sys_write32(tmp, cfg->base + USB_DEV_EPMAP0_OFFSET);
	} else {
		tmp = sys_read32(cfg->base + USB_DEV_EPMAP1_OFFSET);
		tmp &= ~(0xfU << ((uint32_t)(ep_idx - 5U) * 8U + ep_dir_bit));
		tmp |= ((uint32_t)(fifo_idx - 1U) << ((uint32_t)(ep_idx - 5U) * 8U + ep_dir_bit));
		sys_write32(tmp, cfg->base + USB_DEV_EPMAP1_OFFSET);
	}
}

/*
 * Map a FIFO to an endpoint in the FMAP register.
 * ep_idx/fifo_idx are 1-based; fifo_dir: 0=OUT, 1=IN, 2=bidirectional.
 * Use USB_BFLB_V2_FIFO_EP_NONE as ep_idx to disconnect a FIFO.
 */
static void udc_bflb_v2_fifo_setep(const struct device *const dev,
				      const uint8_t ep_idx,
				      const uint8_t fifo_idx,
				      const uint8_t fifo_dir)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	uint32_t tmp;

	__ASSERT_NO_MSG(fifo_idx > 0 && fifo_idx <= USB_BFLB_V2_NUM_DATA_FIFOS);
	__ASSERT_NO_MSG(ep_idx > 0);
	__ASSERT_NO_MSG(fifo_dir <= USB_BFLB_V2_FIFO_DIR_BID);

	tmp = sys_read32(cfg->base + USB_DEV_FMAP_OFFSET);
	tmp &= ~(USB_BFLB_V2_FX_X_MASK << ((uint32_t)(fifo_idx - 1U) * USB_BFLB_V2_FX_X_OFFSET));
	tmp |= ((uint32_t)ep_idx << ((uint32_t)(fifo_idx - 1U) * USB_BFLB_V2_FX_X_OFFSET));
	tmp |= ((uint32_t)fifo_dir << ((uint32_t)(fifo_idx - 1U) * USB_BFLB_V2_FX_X_OFFSET
		+ USB_DIR_FIFO0_SHIFT));
	sys_write32(tmp, cfg->base + USB_DEV_FMAP_OFFSET);
}

static void udc_bflb_v2_vdma_startread(const struct device *const dev,
				       const uint8_t fifo_idx,
				       uint8_t *const buf,
				       const uint32_t len)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	const uint32_t fifo_off = (fifo_idx - 1U) * USB_BFLB_V2_FX_X_OFFSET;
	uint32_t tmp;

	/* Clear START first to ensure a 0→1 edge when we set it below.
	 * A stale START=1 from a previous VDMA would prevent the hardware
	 * from detecting the new start trigger.
	 */
	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);
	tmp &= ~USB_VDMA_LEN_CXF_MASK;
	tmp &= ~USB_VDMA_IO_CXF;
	tmp &= ~USB_VDMA_TYPE_CXF;
	tmp &= ~USB_VDMA_START_CXF;
	tmp |= (len << USB_VDMA_LEN_CXF_SHIFT);
	sys_write32(tmp, cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);

	sys_write32((uint32_t)buf, cfg->base + USB_VDMA_F0PS2_OFFSET + fifo_off);

	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);
	tmp |= USB_VDMA_START_CXF;
	sys_write32(tmp, cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);
}

static bool udc_bflb_v2_vdma_startwrite(const struct device *const dev,
					const uint8_t fifo_idx,
					const uint8_t *data,
					const uint32_t len)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	const uint32_t fifo_off = (fifo_idx - 1U) * USB_BFLB_V2_FX_X_OFFSET;
	uint32_t tmp;

	sys_cache_data_flush_and_invd_range((uint8_t *)data, len);

	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);
	tmp &= ~USB_VDMA_LEN_CXF_MASK;
	tmp &= ~USB_VDMA_IO_CXF;
	tmp |= USB_VDMA_TYPE_CXF;
	tmp &= ~USB_VDMA_START_CXF;
	tmp |= (len << USB_VDMA_LEN_CXF_SHIFT);
	sys_write32(tmp, cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);

	sys_write32((uint32_t)data, cfg->base + USB_VDMA_F0PS2_OFFSET + fifo_off);

	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);
	tmp |= USB_VDMA_START_CXF;
	sys_write32(tmp, cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);

	/* Check if VDMA completed immediately to avoid a lock-up */
	__asm__ volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_off);
	if ((tmp & USB_VDMA_START_CXF) == 0) {
		return true;
	} else {
		return false;
	}
}

static void udc_bflb_v2_vdma_startread_ctrl(const struct device *const dev,
					    uint8_t *buf,
					    uint32_t len)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	struct udc_bflb_v2_data *const priv = udc_get_private(dev);
	uint32_t tmp;

	tmp = sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET);
	tmp &= ~USB_VDMA_LEN_CXF_MASK;
	tmp &= ~USB_VDMA_IO_CXF;
	tmp &= ~USB_VDMA_TYPE_CXF;
	tmp |= (len << USB_VDMA_LEN_CXF_SHIFT);
	sys_write32(tmp, cfg->base + USB_VDMA_CXFPS1_OFFSET);

	sys_write32((uint32_t)buf, cfg->base + USB_VDMA_CXFPS2_OFFSET);

	priv->ep_is_in[0] = false;

	tmp = sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET);
	tmp |= USB_VDMA_START_CXF;
	sys_write32(tmp, cfg->base + USB_VDMA_CXFPS1_OFFSET);
}

static void udc_bflb_v2_vdma_startwrite_ctrl(const struct device *const dev,
					     const uint8_t *data,
					     const uint32_t len)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	struct udc_bflb_v2_data *const priv = udc_get_private(dev);
	uint32_t tmp;

	sys_cache_data_flush_and_invd_range((uint8_t *)data, len);

	tmp = sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET);
	tmp &= ~USB_VDMA_LEN_CXF_MASK;
	tmp &= ~USB_VDMA_IO_CXF;
	tmp |= USB_VDMA_TYPE_CXF;
	tmp |= (len << USB_VDMA_LEN_CXF_SHIFT);
	sys_write32(tmp, cfg->base + USB_VDMA_CXFPS1_OFFSET);

	sys_write32((uint32_t)data, cfg->base + USB_VDMA_CXFPS2_OFFSET);

	priv->ep_is_in[0] = true;

	tmp = sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET);
	tmp |= USB_VDMA_START_CXF;
	sys_write32(tmp, cfg->base + USB_VDMA_CXFPS1_OFFSET);
}

static uint8_t udc_bflb_v2_ep_to_fifo(struct udc_ep_config *const ep_cfg)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep_cfg->addr);

#ifdef CONFIG_UDC_BFLB_V2_FIFO_BIDIR
	if (ep_cfg->mps > USB_BFLB_V2_HSFIFOCAP) {
		if (ep_idx == 1) {
			return 1U;
		} else {
			return 3U;
		}
	}

	return ep_idx;
#else
	if (ep_cfg->mps > USB_BFLB_V2_HSFIFOCAP) {
		if (USB_EP_DIR_IS_OUT(ep_cfg->addr)) {
			return 1U;
		} else {
			return 3U;
		}
	}
	if (USB_EP_DIR_IS_OUT(ep_cfg->addr)) {
		return ep_idx * 2U - 1U;
	} else {
		return ep_idx * 2U;
	}
#endif
}

static int udc_bflb_v2_set_address(const struct device *const dev, const uint8_t addr)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	uint32_t tmp;

	if ((sys_read32(cfg->base + USB_DEV_ADR_OFFSET) & USB_DEVADR_MASK) != addr) {
		tmp = sys_read32(cfg->base + USB_DEV_ADR_OFFSET);
		tmp &= ~USB_DEVADR_MASK;
		tmp |= addr;
		sys_write32(tmp, cfg->base + USB_DEV_ADR_OFFSET);
	}

	return 0;
}

static void udc_bflb_v2_ctrl_setup_start(const struct device *const dev)
{
	udc_bflb_v2_ev_submit(dev, USB_CONTROL_EP_OUT, UDC_BFLB_V2_EVT_CTRL_END, K_NO_WAIT);
}

static void udc_bflb_v2_ctrl_dout_start(const struct device *const dev, const uint16_t size)
{
	struct net_buf *buf;
	struct udc_ep_config *const ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);

	if (!udc_ctrl_stage_is_data_out(dev)) {
		LOG_ERR("Unexpected control dout token");
		return;
	}

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, size);
	if (buf == NULL) {
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOMEM);
		return;
	}

	udc_buf_put(ep_cfg, buf);
	net_buf_add(buf, size);

	udc_bflb_v2_vdma_startread_ctrl(dev, buf->data, size);
}

static void udc_bflb_v2_ctrl_din_start(const struct device *const dev)
{
	struct net_buf *buf;
	struct udc_ep_config *const ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);

	if (!udc_ctrl_stage_is_data_in(dev)) {
		LOG_ERR("Unexpected control din token");
		return;
	}

	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		udc_submit_event(dev, UDC_EVT_ERROR, -ENODATA);
		return;
	}

	udc_bflb_v2_vdma_startwrite_ctrl(dev, buf->data, buf->len);
}

static int udc_bflb_v2_ctrl_xfer_done(const struct device *const dev)
{
	struct udc_bflb_v2_data *const priv = udc_get_private(dev);
	const struct udc_bflb_v2_config *const cfg = dev->config;
	struct net_buf *buf;
	int err;
	uint32_t tmp;
	struct usb_setup_packet *spkg;

	if (priv->setup_received) {
		buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, 8U);
		if (buf == NULL) {
			udc_submit_event(dev, UDC_EVT_ERROR, -ENOMEM);

			/* Re-enable setup interrupt */
			tmp = sys_read32(cfg->base + USB_DEV_MISG0_OFFSET);
			tmp &= ~USB_MCX_SETUP_INT;
			sys_write32(tmp, cfg->base + USB_DEV_MISG0_OFFSET);

			return -ENOMEM;
		}

		udc_ep_buf_set_setup(buf);
		net_buf_add(buf, 8);

		/* Read setup packet directly from CX FIFO register port without the need for VDMA */
		tmp = sys_read32(cfg->base + USB_DMA_TFN_OFFSET);
		tmp |= USB_ACC_CXF_HOV;
		sys_write32(tmp, cfg->base + USB_DMA_TFN_OFFSET);

		priv->setup_packet[0] = sys_read32(cfg->base + USB_DMA_CPS3_OFFSET);
		priv->setup_packet[1] = sys_read32(cfg->base + USB_DMA_CPS3_OFFSET);

		tmp = sys_read32(cfg->base + USB_DMA_TFN_OFFSET);
		tmp &= ~USB_ACC_CXF_HOV;
		sys_write32(tmp, cfg->base + USB_DMA_TFN_OFFSET);

		memcpy(buf->data, priv->setup_packet, sizeof(priv->setup_packet));

		udc_ctrl_update_stage(dev, buf);
		priv->setup_received = false;

		if (udc_ctrl_stage_is_data_in(dev)) {
			err = udc_ctrl_submit_s_in_status(dev);
		} else if (udc_ctrl_stage_is_data_out(dev)) {
			err = 0;
			udc_bflb_v2_ctrl_dout_start(dev, udc_data_stage_length(buf));
		} else if (udc_ctrl_stage_is_no_data(dev)) {
			spkg = (struct usb_setup_packet *)buf->data;
			/* Stack queue too slow */
			if (spkg->bRequest == USB_SREQ_SET_ADDRESS) {
				udc_bflb_v2_set_address(dev, spkg->wValue);
			}
			err =  udc_ctrl_submit_s_status(dev);
		} else {
			err = -EINVAL;
		}

		/* Re-enable setup interrupt after submit completes */
		tmp = sys_read32(cfg->base + USB_DEV_MISG0_OFFSET);
		tmp &= ~USB_MCX_SETUP_INT;
		sys_write32(tmp, cfg->base + USB_DEV_MISG0_OFFSET);

		return err;
	} else if (udc_ctrl_stage_is_data_out(dev)) {
		buf = udc_buf_get(udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT));
		if (buf != NULL) {
			sys_cache_data_invd_range(buf->data, buf->len);
		}
		udc_ctrl_update_stage(dev, buf);
		return udc_ctrl_submit_s_out_status(dev, buf);
	} else if (udc_ctrl_stage_is_data_in(dev)) {
		buf = udc_buf_get(udc_get_ep_cfg(dev, USB_CONTROL_EP_IN));

		if (buf == NULL) {
			LOG_ERR("No buf for DATA_IN completion");
			return -ENODATA;
		}

		udc_ctrl_update_stage(dev, buf);
		if (udc_ctrl_stage_is_status_out(dev)) {
			net_buf_unref(buf);
			/* Hardware handles status OUT automatically after
			 * CX_DONE. Allocate a status buf to notify the
			 * USB stack that the transfer is complete.
			 */
			buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, 0U);
			if (buf == NULL) {
				return -ENOMEM;
			}
			err = udc_ctrl_submit_status(dev, buf);
			udc_ctrl_update_stage(dev, buf);
			return err;
		}
		net_buf_unref(buf);
	} else if (udc_ctrl_stage_is_no_data(dev) || udc_ctrl_stage_is_status_in(dev)) {
		/*
		 * CX_COMEND after status IN ZLP was sent.
		 * Notify USB stack and transition to SETUP.
		 */
		buf = udc_buf_get(udc_get_ep_cfg(dev, USB_CONTROL_EP_IN));
		if (buf != NULL) {
			udc_ctrl_submit_status(dev, buf);
			udc_ctrl_update_stage(dev, buf);
		}
	}

	return 0;
}

static uint32_t udc_bflb_v2_ep_remain(const struct device *const dev, const uint8_t fifo_idx)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = (sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET
		+ (fifo_idx - 1U) * USB_BFLB_V2_FX_X_OFFSET)
		& USB_VDMA_LEN_CXF_MASK);

	return (tmp >> USB_VDMA_LEN_CXF_SHIFT);
}

static void udc_bflb_v2_ep_dout_start(const struct device *const dev,
				      struct udc_ep_config *const ep_cfg)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	struct udc_bflb_v2_data *const priv = udc_get_private(dev);
	struct net_buf *buf;
	uint8_t ep_idx = USB_EP_GET_IDX(ep_cfg->addr);
	uint8_t fifo;
	uint32_t tmp;

	if (priv->ep_is_in[ep_idx]) {
		LOG_ERR("Unexpected ep 0x%02x dout token", ep_cfg->addr);
	}

	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		LOG_ERR("No buffer for OUT ep 0x%02x", ep_cfg->addr);
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
	} else {
		fifo = udc_bflb_v2_ep_to_fifo(ep_cfg);

		/* Don't start VDMA yet — just arm the G1 OUT/SPK
		 * interrupts so we know when data arrives in the FIFO.
		 * The VDMA will be started from the ISR after the
		 * FIFO byte count is known, avoiding the hardware
		 * blind spot where VDMA doesn't auto-complete for
		 * packets between 16 and MPS-1 bytes.
		 */
		sys_write32((3U << (2U * (fifo - 1U))),
			    cfg->base + USB_DEV_ISG1_OFFSET);
		tmp = sys_read32(cfg->base + USB_DEV_MISG1_OFFSET);
		tmp &= ~(3U << (2U * (fifo - 1U)));
		sys_write32(tmp, cfg->base + USB_DEV_MISG1_OFFSET);
	}
}

static void udc_bflb_v2_ep_din_start(const struct device *const dev,
				     struct udc_ep_config *const ep_cfg)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	struct udc_bflb_v2_data *const priv = udc_get_private(dev);
	const uint8_t ep_idx = USB_EP_GET_IDX(ep_cfg->addr);
	struct net_buf *buf;
	uint32_t tmp;
	uint8_t fifo;
	uint32_t chunk;

	if (!priv->ep_is_in[ep_idx]) {
		LOG_ERR("Unexpected ep 0x%02x din token", ep_cfg->addr);
	}

	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		LOG_ERR("No buffer for IN ep 0x%02x", ep_cfg->addr);
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
	} else {
		fifo = udc_bflb_v2_ep_to_fifo(ep_cfg);
		chunk = MIN(buf->len, udc_mps_ep_size(ep_cfg));

		/* Clean up potential stale irq, start VDMA to load data, then check
		 * for immediate VDMA completion before unmasking G1 IN_INT.
		 */
		sys_write32(1U << (15U + fifo), cfg->base + USB_DEV_ISG1_OFFSET);
		if (udc_bflb_v2_vdma_startwrite(dev, fifo, buf->data, chunk)) {
			udc_bflb_v2_ev_submit(dev, ep_cfg->addr, UDC_BFLB_V2_EVT_END, K_NO_WAIT);
		} else {
			tmp = sys_read32(cfg->base + USB_DEV_MISG1_OFFSET);
			tmp &= ~(1U << (15U + fifo));
			sys_write32(tmp, cfg->base + USB_DEV_MISG1_OFFSET);
		}
	}
}

/*
 * OUT chunk completion: VDMA transferred up to one MPS from FIFO to memory.
 * Always completes the transfer after one chunk — the class layer can
 * re-queue if it wants more data.
 */
static void udc_bflb_v2_out_chunk_done(const struct device *dev, struct udc_ep_config *ep_cfg)
{
	const uint8_t fifo = udc_bflb_v2_ep_to_fifo(ep_cfg);
	struct net_buf *buf = udc_buf_peek(ep_cfg);
	struct udc_bflb_v2_data *const priv = udc_get_private(dev);
	uint32_t remain, chunk, received;

	if (buf == NULL) {
		LOG_ERR("No buf for OUT chunk ep 0x%02x", ep_cfg->addr);
		udc_submit_event(dev, UDC_EVT_ERROR, -ENODATA);
		return;
	}

	remain = udc_bflb_v2_ep_remain(dev, fifo);
	/* Use the actual VDMA programmed length, not the buffer
	 * tailroom — the VDMA may have been started for fewer
	 * bytes than the buffer can hold (FIFO-first approach).
	 */
	chunk = priv->out_vdma_len[fifo];
	if (chunk < remain) {
		received = 0;
	} else {
		received = chunk - remain;
	}

	sys_cache_data_invd_range(buf->data + buf->len, received);

	net_buf_add(buf, received);

	/* Complete the transfer — dequeue and submit to class layer */
	buf = udc_buf_get(ep_cfg);
	udc_ep_set_busy(ep_cfg, false);
	udc_submit_ep_event(dev, buf, 0);
}

/*
 * IN chunk completion: VDMA transferred up to one MPS from memory to FIFO.
 * Always complete after one chunk — the host may read fewer bytes than
 * the class layer queued, and re-arming would stall the VDMA waiting
 * for FIFO space that the host will never free.  The class layer can
 * re-queue if it wants to send more data.
 */
static void udc_bflb_v2_in_chunk_done(const struct device *dev, struct udc_ep_config *ep_cfg)
{
	const uint8_t fifo = udc_bflb_v2_ep_to_fifo(ep_cfg);
	struct net_buf *buf = udc_buf_peek(ep_cfg);
	uint32_t remain, sent;
	uint32_t chunk;

	if (buf == NULL) {
		LOG_ERR("No buf for IN chunk ep 0x%02x", ep_cfg->addr);
		udc_submit_event(dev, UDC_EVT_ERROR, -ENODATA);
		return;
	}

	remain = udc_bflb_v2_ep_remain(dev, fifo);
	chunk = MIN(buf->len, udc_mps_ep_size(ep_cfg));
	sent = chunk - remain;


	net_buf_pull(buf, sent);

	/* Complete the transfer — kick_next will clean up any
	 * residual FIFO data via vdma_stop + fifo_reset.
	 */
	buf = udc_buf_get(ep_cfg);
	udc_ep_set_busy(ep_cfg, false);
	udc_submit_ep_event(dev, buf, 0);
}

static void udc_bflb_v2_work_handler_xfer(const struct device *const dev,
					     struct udc_ep_config *const ep_cfg)
{
	const uint8_t ep_idx = USB_EP_GET_IDX(ep_cfg->addr);
	struct udc_bflb_v2_data *const priv = udc_get_private(dev);
	struct net_buf *buf = udc_buf_peek(ep_cfg);
	uint8_t fifo;
#ifdef CONFIG_UDC_BFLB_V2_FIFO_BIDIR
	const struct udc_bflb_v2_config *const cfg = dev->config;
	uint32_t remain, tmp;
	uint8_t in_addr;
	struct udc_ep_config *in_cfg;
#endif

	if (ep_idx == 0) {
		LOG_ERR("Unexpected control transfer in xfer handler");
		return;
	}

	if (buf == NULL) {
		return;
	}

	if (ep_cfg->stat.halted) {
		return;
	}

	if (udc_ep_is_busy(ep_cfg)) {
		return;
	}

	/* Handle IN ZLP separately — no FIFO/VDMA involvement */
	if (USB_EP_DIR_IS_IN(ep_cfg->addr) && buf->len == 0) {
		udc_bflb_v2_ep_ack(dev, ep_idx);
		buf = udc_buf_get(ep_cfg);
		if (buf != NULL) {
			udc_submit_ep_event(dev, buf, 0);
		}
		return;
	}

	fifo = udc_bflb_v2_ep_to_fifo(ep_cfg);
	if (priv->fifo_active[fifo]) {
#ifdef CONFIG_UDC_BFLB_V2_FIFO_BIDIR
		if (USB_EP_DIR_IS_OUT(ep_cfg->addr) && priv->ep_is_in[ep_idx]) {
			remain = udc_bflb_v2_ep_remain(dev, fifo);
			/* BID FIFO: OUT evicts IN */
			in_addr = USB_EP_DIR_IN | ep_idx;
			in_cfg = udc_get_ep_cfg(dev, in_addr);

			LOG_WRN("FIFO %u: evicting IN ep 0x%02x for OUT", fifo, in_addr);
			udc_bflb_v2_vdma_stop(dev, fifo);
			udc_bflb_v2_fifo_reset(dev, fifo);
			priv->fifo_active[fifo] = false;

			tmp = sys_read32(cfg->base + USB_DEV_MISG1_OFFSET);
			tmp |= (3U << (2U * (fifo - 1U)));
			sys_write32(tmp, cfg->base + USB_DEV_MISG1_OFFSET);

			if (in_cfg != NULL && udc_ep_is_busy(in_cfg)) {
				udc_ep_set_busy(in_cfg, false);
				if (remain > 0) {
					udc_bflb_v2_ev_submit(dev, in_addr, UDC_BFLB_V2_EVT_XFER, K_NO_WAIT);
				}
			}
		} else {
			return;
		}
#else
		return;
#endif
	}
	priv->fifo_active[fifo] = true;

	if (USB_EP_DIR_IS_OUT(ep_cfg->addr)) {
		priv->ep_is_in[ep_idx] = false;
		udc_ep_set_busy(ep_cfg, true);
		udc_bflb_v2_ep_dout_start(dev, ep_cfg);
	} else {
		if (!priv->ep_is_in[ep_idx]) {
			udc_bflb_v2_vdma_stop(dev, fifo);
			udc_bflb_v2_fifo_reset(dev, fifo);
		}
		priv->ep_is_in[ep_idx] = true;
		udc_ep_set_busy(ep_cfg, true);
		udc_bflb_v2_ep_din_start(dev, ep_cfg);
	}
}

/*
 * After a VDMA transfer completes on a BID FIFO, release the FIFO and
 * start the next pending transfer — opposite direction first (the typical
 * loopback OUT→IN→OUT… pattern), then same direction.
 */
static void udc_bflb_v2_fifo_kick_next(const struct device *dev,
				       struct udc_ep_config *completed_cfg)
{
	const uint8_t fifo = udc_bflb_v2_ep_to_fifo(completed_cfg);
	struct udc_bflb_v2_data *const priv = udc_get_private(dev);
#ifdef CONFIG_UDC_BFLB_V2_FIFO_BIDIR
	const uint8_t ep_idx = USB_EP_GET_IDX(completed_cfg->addr);
	uint8_t out_addr;
	uint8_t in_addr;
	struct udc_ep_config *out_cfg;
	struct udc_ep_config *in_cfg;
	struct udc_ep_config *first_cfg;
	struct udc_ep_config *second_cfg;
#endif

	LOG_DBG("Kick next for %u", fifo);

	priv->fifo_active[fifo] = false;


#ifdef CONFIG_UDC_BFLB_V2_FIFO_BIDIR

	out_addr = USB_EP_DIR_OUT | ep_idx;
	in_addr = USB_EP_DIR_IN | ep_idx;
	out_cfg = udc_get_ep_cfg(dev, out_addr);
	in_cfg = udc_get_ep_cfg(dev, in_addr);

	if (USB_EP_DIR_IS_OUT(completed_cfg->addr)) {
		first_cfg = in_cfg;
		second_cfg = out_cfg;
	} else {
		first_cfg = out_cfg;
		second_cfg = in_cfg;
	}

	if (first_cfg != NULL && udc_buf_peek(first_cfg) != NULL &&
		!udc_ep_is_busy(first_cfg)) {
		udc_bflb_v2_ev_submit(dev, first_cfg->addr, UDC_BFLB_V2_EVT_XFER, K_NO_WAIT);
		return;
	}

	if (second_cfg != NULL && udc_buf_peek(second_cfg) != NULL &&
		!udc_ep_is_busy(second_cfg)) {
		udc_bflb_v2_ev_submit(dev, second_cfg->addr, UDC_BFLB_V2_EVT_XFER, K_NO_WAIT);
		return;
	}
#else
	/* Separate per-direction FIFOs: just check if the same
	 * endpoint+direction has another buffer queued.
	 */
	if (udc_buf_peek(completed_cfg) != NULL && !udc_ep_is_busy(completed_cfg)) {
		udc_bflb_v2_work_handler_xfer(dev, completed_cfg);
	}
#endif
}

static void udc_bflb_v2_work_handler(struct k_work *item)
{
	struct k_work_delayable *item_delayable = k_work_delayable_from_work(item);
	const struct udc_bflb_v2_ev *const ev =
		CONTAINER_OF(item_delayable, struct udc_bflb_v2_ev, work);
	struct udc_ep_config *const ep_cfg = udc_get_ep_cfg(ev->dev, ev->ep_addr);
	int err = 0;

	if (unlikely(ep_cfg == NULL)) {
		err = -ENODATA;
		LOG_ERR("Invalid Endpoint Configuration in Work Queue");
	} else {
		switch (ev->event) {
		case UDC_BFLB_V2_EVT_CTRL_END:
			err = udc_bflb_v2_ctrl_xfer_done(ev->dev);
			break;
		case UDC_BFLB_V2_EVT_END:
			if (udc_ep_is_busy(ep_cfg)) {
				if (USB_EP_DIR_IS_OUT(ep_cfg->addr)) {
					udc_bflb_v2_out_chunk_done(ev->dev, ep_cfg);
				} else {
					udc_bflb_v2_in_chunk_done(ev->dev, ep_cfg);
				}
				udc_bflb_v2_fifo_kick_next(ev->dev, ep_cfg);
			}
			break;
		case UDC_BFLB_V2_EVT_XFER:
			udc_bflb_v2_work_handler_xfer(ev->dev, ep_cfg);
			break;

		default:
			break;
		}
	}

	if (unlikely(err != 0)) {
		udc_submit_event(ev->dev, UDC_EVT_ERROR, err);
	}

	k_mem_slab_free(&udc_bflb_v2_ev_slab, (void *)ev);
}

static void udc_bflb_v2_ev_submit(const struct device *const dev,
				  const uint8_t ep_addr,
				  const enum udc_bflb_v2_ev_type event,
				  k_timeout_t delay)
{
	struct udc_bflb_v2_ev *ev;
	int ret;

	ret = k_mem_slab_alloc(&udc_bflb_v2_ev_slab, (void **)&ev, K_NO_WAIT);
	if (ret < 0) {
		udc_submit_event(dev, UDC_EVT_ERROR, ret);
		LOG_ERR("Failed to allocate slab");
		return;
	}

	ev->dev = dev;
	ev->ep_addr = ep_addr;
	ev->event = event;
	k_work_init_delayable(&ev->work, udc_bflb_v2_work_handler);
	ret = k_work_schedule_for_queue(udc_get_work_q(), &ev->work, delay);
	if (ret < 0) {
		udc_submit_event(dev, UDC_EVT_ERROR, ret);
		LOG_ERR("Failed to submit event");
		return;
	}
}

/*
 * UDC API implementation
 *
 */

static int udc_bflb_v2_ep_enqueue(const struct device *const dev,
				  struct udc_ep_config *const config,
				  struct net_buf *buf)
{
	const uint8_t ep_idx = USB_EP_GET_IDX(config->addr);

	if (ep_idx == 0) {
		if (USB_EP_DIR_IS_OUT(config->addr)) {
			udc_buf_put(config, buf);
			udc_bflb_v2_ctrl_dout_start(dev, udc_data_stage_length(buf));
		} else if (buf->len == 0) {
			/* Status IN (ZLP): tell hardware to complete the
			 * control transfer, then notify the USB stack.
			 * By this point the setup buf is already freed
			 * and buf is standalone (not a frag).
			 */
			udc_bflb_v2_ctrl_ack(dev);
			udc_ctrl_update_stage(dev, buf);
			udc_ctrl_submit_status(dev, buf);
		} else {
			udc_buf_put(config, buf);
			udc_bflb_v2_ctrl_din_start(dev);
		}
	} else {
		udc_buf_put(config, buf);

		/*
		 * Always defer data EP starts to the work queue so that
		 * both IN and OUT buffers have a chance to be queued
		 * before the FIFO serialization logic picks which
		 * direction to start (OUT is preferred).
		 */
		udc_bflb_v2_ev_submit(dev, config->addr, UDC_BFLB_V2_EVT_XFER, K_NO_WAIT);
	}

	return 0;
}

static int udc_bflb_v2_ep_dequeue(const struct device *const dev,
				  struct udc_ep_config *const ep_cfg)
{
	const uint8_t ep_idx = USB_EP_GET_IDX(ep_cfg->addr);
	struct udc_bflb_v2_data *const priv = udc_get_private(dev);
	unsigned int lock_key;
	struct net_buf *buf;
	uint8_t fifo;

	lock_key = irq_lock();

	LOG_DBG("dequeue for ep %u", ep_idx);

	/* Stop any in-flight VDMA and clean up FIFO state so that
	 * pending work-queue items (EVT_END, EVT_XFER) that arrive
	 * after this point find the endpoint idle.
	 */
	if (ep_idx > 0) {
		fifo = udc_bflb_v2_ep_to_fifo(ep_cfg);

#ifdef CONFIG_UDC_BFLB_V2_FIFO_BIDIR
		if (!priv->fifo_active[fifo]
		    || (priv->ep_is_in[ep_idx] ?
		    USB_EP_DIR_IS_IN(ep_cfg->addr) : USB_EP_DIR_IS_OUT(ep_cfg->addr))) {
#else
		if (true) {
#endif
			udc_bflb_v2_vdma_stop(dev, fifo);
			udc_bflb_v2_fifo_reset(dev, fifo);
			priv->fifo_active[fifo] = false;
		}
	}

	udc_ep_set_busy(ep_cfg, false);

	buf = udc_buf_get_all(ep_cfg);
	if (buf != NULL) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	irq_unlock(lock_key);

	return 0;
}

static int udc_bflb_v2_ep_enable(const struct device *const dev,
				 struct udc_ep_config *const config)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	const uint8_t ep_idx = USB_EP_GET_IDX(config->addr);
	uint32_t tmp;
	uint8_t fifo;

	if (ep_idx == 0) {
		return 0;
	}

	/* Reset the FIFO before (re-)enabling to clear stale data and
	 * prevent spurious VDMA completion interrupts.
	 */
	fifo = udc_bflb_v2_ep_to_fifo(config);

	udc_bflb_v2_fifo_reset(dev, fifo);
	/* Clear any pending VDMA completion for this FIFO */
	sys_write32(1U << fifo, cfg->base + USB_DEV_ISG3_OFFSET);

	if (USB_EP_DIR_IS_OUT(config->addr)) {
		udc_bflb_v2_ep_set_out_mps(dev, ep_idx, config->mps);
	} else {
		udc_bflb_v2_ep_set_in_mps(dev, ep_idx, config->mps);
	}

#ifdef CONFIG_UDC_BFLB_V2_FIFO_BIDIR
	if (config->mps > USB_BFLB_V2_HSFIFOCAP) {
		if (ep_idx > 2) {
			LOG_DBG("We need to use 2 FIFO per ep if mps > 512 (USB High Speed)");
			return -ENOTSUP;
		}
		if (ep_idx == 1) {
			udc_bflb_v2_ep_setfifo(dev, ep_idx, 1, USB_BFLB_V2_EP_DIR_IN);
			udc_bflb_v2_ep_setfifo(dev, ep_idx, 1, USB_BFLB_V2_EP_DIR_OUT);
			udc_bflb_v2_fifo_setep(dev, ep_idx, 1, USB_BFLB_V2_FIFO_DIR_BID);
			udc_bflb_v2_fifo_setep(dev, ep_idx, 2, USB_BFLB_V2_FIFO_DIR_BID);
			udc_bflb_v2_fifo_configure(dev, 1, config, 1, true);
			udc_bflb_v2_fifo_configure(dev, 2, config, 1, false);
		} else if (ep_idx == 2) {
			udc_bflb_v2_ep_setfifo(dev, ep_idx, 3, USB_BFLB_V2_EP_DIR_IN);
			udc_bflb_v2_ep_setfifo(dev, ep_idx, 3, USB_BFLB_V2_EP_DIR_OUT);
			udc_bflb_v2_fifo_setep(dev, ep_idx, 3, USB_BFLB_V2_FIFO_DIR_BID);
			udc_bflb_v2_fifo_setep(dev, ep_idx, 4, USB_BFLB_V2_FIFO_DIR_BID);
			udc_bflb_v2_fifo_configure(dev, 3, config, 1, true);
			udc_bflb_v2_fifo_configure(dev, 4, config, 1, false);
		}
	} else {
		udc_bflb_v2_ep_setfifo(dev, ep_idx, ep_idx, USB_BFLB_V2_EP_DIR_IN);
		udc_bflb_v2_ep_setfifo(dev, ep_idx, ep_idx, USB_BFLB_V2_EP_DIR_OUT);
		udc_bflb_v2_fifo_setep(dev, ep_idx, ep_idx, USB_BFLB_V2_FIFO_DIR_BID);
		udc_bflb_v2_fifo_configure(dev, ep_idx, config, 1, true);
	}
#else
	if (config->mps > USB_BFLB_V2_HSFIFOCAP) {
		return -ENOTSUP;
	} else {
		/* EP1-2: separate FIFOs per direction.
		* EP1 OUT→F1, IN→F2.  EP2 OUT→F3, IN→F4.
		*/
		fifo = udc_bflb_v2_ep_to_fifo(config);

		if (USB_EP_DIR_IS_OUT(config->addr)) {
			udc_bflb_v2_ep_setfifo(dev, ep_idx, fifo, USB_BFLB_V2_EP_DIR_OUT);
		} else {
			udc_bflb_v2_ep_setfifo(dev, ep_idx, fifo, USB_BFLB_V2_EP_DIR_IN);
		}
		udc_bflb_v2_fifo_setep(dev, ep_idx, fifo, USB_BFLB_V2_FIFO_DIR_BID);
		udc_bflb_v2_fifo_configure(dev, fifo, config, 1, true);
	}

#endif

	tmp = sys_read32(cfg->base + USB_DEV_ADR_OFFSET);
	tmp |= USB_AFT_CONF;
	sys_write32(tmp, cfg->base + USB_DEV_ADR_OFFSET);

	return 0;
}

static int udc_bflb_v2_ep_disable(const struct device *const dev,
				  struct udc_ep_config *const config)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	const uint8_t ep_idx = USB_EP_GET_IDX(config->addr);
	struct udc_bflb_v2_data *const priv = udc_get_private(dev);
	uint8_t fifo;
	uint32_t tmp;

	if (ep_idx == 0) {
		return 0;
	}

	LOG_DBG("disable for ep %u", ep_idx);

	/* Stop any active VDMA, then reset the FIFO(s) */
#ifdef CONFIG_UDC_BFLB_V2_FIFO_BIDIR
	if (config->mps > USB_BFLB_V2_HSFIFOCAP) {
		if (ep_idx == 1) {
			udc_bflb_v2_vdma_stop(dev, 1);
			udc_bflb_v2_vdma_stop(dev, 2);
			udc_bflb_v2_fifo_reset(dev, 1);
			udc_bflb_v2_fifo_reset(dev, 2);
			priv->fifo_active[1] = false;
			priv->fifo_active[2] = false;
		} else if (ep_idx == 2) {
			udc_bflb_v2_vdma_stop(dev, 3);
			udc_bflb_v2_vdma_stop(dev, 4);
			udc_bflb_v2_fifo_reset(dev, 3);
			udc_bflb_v2_fifo_reset(dev, 4);
			priv->fifo_active[3] = false;
			priv->fifo_active[4] = false;
		}
	} else {
		fifo = udc_bflb_v2_ep_to_fifo(config);

		if (!priv->fifo_active[fifo]
		    || (priv->ep_is_in[ep_idx] ?
		    USB_EP_DIR_IS_IN(config->addr) : USB_EP_DIR_IS_OUT(config->addr))) {
			udc_bflb_v2_vdma_stop(dev, fifo);
			udc_bflb_v2_fifo_reset(dev, fifo);
			priv->fifo_active[fifo] = false;
		}
	}
#else
	fifo = udc_bflb_v2_ep_to_fifo(config);

	udc_bflb_v2_vdma_stop(dev, fifo);
	udc_bflb_v2_fifo_reset(dev, fifo);
	priv->fifo_active[fifo] = false;
#endif

	/* Mask G1 IN_INT for any IN FIFOs associated with this endpoint */
	if (USB_EP_DIR_IS_IN(config->addr)) {
		fifo = udc_bflb_v2_ep_to_fifo(config);

		tmp = sys_read32(cfg->base + USB_DEV_MISG1_OFFSET);
		tmp |= (1U << (15U + fifo));
		sys_write32(tmp, cfg->base + USB_DEV_MISG1_OFFSET);
	} else if (USB_EP_DIR_IS_OUT(config->addr)) {
		fifo = udc_bflb_v2_ep_to_fifo(config);

		tmp = sys_read32(cfg->base + USB_DEV_MISG1_OFFSET);
		tmp &= ~(3U << (2U * (fifo - 1U)));
		sys_write32(tmp, cfg->base + USB_DEV_MISG1_OFFSET);
	}

	/* Clear the busy flag so the endpoint can be re-used after
	 * a SET_INTERFACE disable/enable cycle without getting stuck
	 * returning -EBUSY on the next enqueue.
	 */
	udc_ep_set_busy(config, false);

	return 0;
}

static int udc_bflb_v2_ep_set_halt(const struct device *const dev,
				   struct udc_ep_config *const config)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	const uint8_t ep_idx = USB_EP_GET_IDX(config->addr);
	struct udc_bflb_v2_data *const priv = udc_get_private(dev);
	uint32_t tmp;
	uint8_t fifo;

	if (ep_idx == 0) {
		tmp = sys_read32(cfg->base + USB_DEV_CXCFE_OFFSET);
		tmp |= USB_CX_STL;
		sys_write32(tmp, cfg->base + USB_DEV_CXCFE_OFFSET);
	} else {
		fifo = udc_bflb_v2_ep_to_fifo(config);

		/* Stop any in-flight VDMA and reset the FIFO.  While the
		 * endpoint was halted the host received STALL instead of
		 * data, so no completion ever fired and the busy flag was
		 * never cleared.  Clean up that stale state.
		 */
#ifdef CONFIG_UDC_BFLB_V2_FIFO_BIDIR
		if (!priv->fifo_active[fifo]
		    || (priv->ep_is_in[ep_idx] ?
		    USB_EP_DIR_IS_IN(config->addr) : USB_EP_DIR_IS_OUT(config->addr))) {
#else
		if (true) {
#endif
			udc_bflb_v2_vdma_stop(dev, fifo);
			udc_bflb_v2_fifo_reset(dev, fifo);
			priv->fifo_active[fifo] = false;
		}

		udc_ep_set_busy(config, false);

		/* Clear pending G3 VDMA completion for this FIFO */
		sys_write32(1U << fifo, cfg->base + USB_DEV_ISG3_OFFSET);

		if (USB_EP_DIR_IS_OUT(config->addr)) {
			tmp = sys_read32(cfg->base + USB_DEV_OUTMPS1_OFFSET
				+ (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
			tmp |= USB_STL_OEP1;
			sys_write32(tmp, cfg->base + USB_DEV_OUTMPS1_OFFSET
				+ (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
		} else {
			tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET
				+ (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
			tmp |= USB_STL_IEP1;
			sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET
				+ (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
		}
		config->stat.halted = true;
	}

	return 0;
}

static int udc_bflb_v2_ep_clear_halt(const struct device *const dev,
				      struct udc_ep_config *const config)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	const uint8_t ep_idx = USB_EP_GET_IDX(config->addr);
	uint32_t tmp;

	if (ep_idx == 0) {
		tmp = sys_read32(cfg->base + USB_DEV_CXCFE_OFFSET);
		tmp &= ~USB_CX_STL;
		sys_write32(tmp, cfg->base + USB_DEV_CXCFE_OFFSET);
	} else if (config->stat.halted) {

		if (USB_EP_DIR_IS_OUT(config->addr)) {
			tmp = sys_read32(cfg->base + USB_DEV_OUTMPS1_OFFSET +
					 (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
			tmp &= ~USB_STL_OEP1;
			tmp |= USB_RSTG_OEP1;
			sys_write32(tmp, cfg->base + USB_DEV_OUTMPS1_OFFSET +
						 (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
			/* RSTG is not self-clearing — must be lowered */
			tmp &= ~USB_RSTG_OEP1;
			sys_write32(tmp, cfg->base + USB_DEV_OUTMPS1_OFFSET +
						 (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
		} else {
			tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET +
					 (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
			tmp &= ~USB_STL_IEP1;
			tmp |= USB_RSTG_IEP1;
			sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET +
						 (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
			/* RSTG is not self-clearing — must be lowered */
			tmp &= ~USB_RSTG_IEP1;
			sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET +
						 (ep_idx - 1U) * USB_BFLB_V2_XPS_X_OFFSET);
		}

		config->stat.halted = false;

		udc_bflb_v2_ev_submit(dev, config->addr, UDC_BFLB_V2_EVT_XFER, K_NO_WAIT);
	}

	return 0;
}

static int udc_bflb_v2_host_wakeup(const struct device *const dev)
{
	LOG_DBG("Remote wakeup from %p", dev);

	return -ENOTSUP;
}

static int udc_bflb_v2_enable(const struct device *const dev)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	struct udc_bflb_v2_data *const priv = udc_get_private(dev);
	uint32_t tmp;

	tmp = sys_read32(PDS_BASE + PDS_USB_CTL_OFFSET);
	tmp |= PDS_REG_USB_IDDIG_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_CTL_OFFSET);

	/* Disable global interrupt while configuring */
	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp &= ~USB_GLINT_EN_HOV;
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	/* Force signals unplugging*/
	tmp = sys_read32(cfg->base + USB_PHY_TST_OFFSET);
	tmp |= USB_UNPLUG;
	sys_write32(tmp, cfg->base + USB_PHY_TST_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp &= ~USB_CAP_RMWAKUP;
	tmp |= USB_CHIP_EN_HOV;
	if (cfg->speed_idx < UDC_BUS_SPEED_HS) {
		tmp |= USB_FORCE_FS;
	} else {
		tmp &= ~USB_FORCE_FS;
	}
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp |= USB_SFRST_HOV;
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	/* wait for soft reset */
	while ((sys_read32(cfg->base + USB_DEV_CTL_OFFSET) & USB_SFRST_HOV) != 0) {
	}

	tmp = sys_read32(cfg->base + USB_DEV_ADR_OFFSET);
	tmp &= ~USB_AFT_CONF;
	sys_write32(tmp, cfg->base + USB_DEV_ADR_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_SMT_OFFSET);
	tmp &= ~USB_SOFMT_MASK;
	if (cfg->speed_idx == UDC_BUS_SPEED_HS) {
		tmp |= USB_BFLB_V2_TIMER_AFTER_RESET_HS;
	} else {
		tmp |= USB_BFLB_V2_TIMER_AFTER_RESET_FS;
	}
	sys_write32(tmp, cfg->base + USB_DEV_SMT_OFFSET);

	/* Clear all pending interrupts in each group.
	 * 'MISGx': Mask Interrupts Source Group x
	 * 'ISGx' : Interrupts Source Group x (set is clear, read is status)
	 */

	/* Clear irqs group 0: control */
	sys_write32(0xffffffffU, cfg->base + USB_DEV_ISG0_OFFSET);
	/* Clear irqs group 1: FIFO */
	sys_write32(0xffffffffU, cfg->base + USB_DEV_ISG1_OFFSET);
	/* Clear irqs group 2: device */
	sys_write32(0x3ffU, cfg->base + USB_DEV_ISG2_OFFSET);
	/* Clear irqs group 3: DMA */
	sys_write32(0xffffffffU, cfg->base + USB_DEV_ISG3_OFFSET);

	/* G0: unmask SETUP only (control transfer setup token) */
	tmp = sys_read32(cfg->base + USB_DEV_MISG0_OFFSET);
	tmp &= ~(USB_MCX_SETUP_INT);
	tmp |= USB_MCX_COMFAIL_INT
		| USB_MCX_COMABORT_INT
		| USB_MCX_COMEND_INT
		| USB_MCX_IN_INT
		| USB_MCX_OUT_INT;
	sys_write32(tmp, cfg->base + USB_DEV_MISG0_OFFSET);

	/* G1: mask all FIFO interrupts (DMA completion used instead) */
	sys_write32(0xffffffffU, cfg->base + USB_DEV_MISG1_OFFSET);

	/* G2: unmask bits [4:0] = USBRST, SUSP, RESM, ISO_ERR, ISO_ABORT
	 *     + bit 6 = RX0BYTE (OUT ZLP received)
	 */
	sys_write32(0xffffffe0U & ~USB_MRX0BYTE_INT, cfg->base + USB_DEV_MISG2_OFFSET);

	/* G3: unmask bits [4:0] = VDMA completion for CXF + FIFO 0-3 */
	sys_write32(0xffffffe0U, cfg->base + USB_DEV_MISG3_OFFSET);

	/* Enable all interrupt groups at top level */
	tmp = sys_read32(cfg->base + USB_DEV_MIGR_OFFSET);
	tmp &= ~(USB_MINT_G0
		| USB_MINT_G1
		| USB_MINT_G2
		| USB_MINT_G3
		| USB_MINT_G4);
	sys_write32(tmp, cfg->base + USB_DEV_MIGR_OFFSET);

	tmp = sys_read32(cfg->base + USB_GLB_INT_OFFSET);
	tmp |= USB_MHC_INT;
	tmp |= USB_MOTG_INT;
	tmp &= ~USB_MDEV_INT;
	sys_write32(tmp, cfg->base + USB_GLB_INT_OFFSET);

	/* Disconnect all EPs from FIFOs (0xF = no EP assigned) */
	sys_write32(0xffffffffU, cfg->base + USB_DEV_EPMAP0_OFFSET);
	sys_write32(0xffU, cfg->base + USB_DEV_EPMAP1_OFFSET);
	udc_bflb_v2_fifo_setep(dev, USB_BFLB_V2_FIFO_EP_NONE, 1, USB_BFLB_V2_FIFO_DIR_OUT);
	udc_bflb_v2_fifo_setep(dev, USB_BFLB_V2_FIFO_EP_NONE, 2, USB_BFLB_V2_FIFO_DIR_OUT);
	udc_bflb_v2_fifo_setep(dev, USB_BFLB_V2_FIFO_EP_NONE, 3, USB_BFLB_V2_FIFO_DIR_OUT);
	udc_bflb_v2_fifo_setep(dev, USB_BFLB_V2_FIFO_EP_NONE, 4, USB_BFLB_V2_FIFO_DIR_OUT);

	udc_bflb_v2_fifo_reset(dev, 1);
	udc_bflb_v2_fifo_reset(dev, 2);
	udc_bflb_v2_fifo_reset(dev, 3);
	udc_bflb_v2_fifo_reset(dev, 4);
	memset(priv->fifo_active, 0, sizeof(priv->fifo_active));

	/* Enable VDMA (virtual DMA) for all FIFO transfers */
	tmp = sys_read32(cfg->base + USB_VDMA_CTRL_OFFSET);
	tmp |= USB_VDMA_EN;
	sys_write32(tmp, cfg->base + USB_VDMA_CTRL_OFFSET);

	/* Re-connect D+/D- */
	tmp = sys_read32(cfg->base + USB_PHY_TST_OFFSET);
	tmp &= ~USB_UNPLUG;
	sys_write32(tmp, cfg->base + USB_PHY_TST_OFFSET);

	/* Enable global interrupt — controller is now ready */
	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp |= USB_GLINT_EN_HOV;
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	priv->reset_expiration = sys_timepoint_calc(USB_BFLB_V2_TIMER_AFTER_RESET_T);

	return 0;
}

static int udc_bflb_v2_disable(const struct device *const dev)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp &= ~USB_GLINT_EN_HOV;
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	tmp = sys_read32(cfg->base + USB_PHY_TST_OFFSET);
	tmp |= USB_UNPLUG;
	sys_write32(tmp, cfg->base + USB_PHY_TST_OFFSET);

	return 0;
}

/* Enable the USB PLL (48 MHz from WiFi PLL CFG10). */
static void udc_bflb_v2_clock_init(const struct device *const dev)
{
	uint32_t tmp;

	/* Power on USB PLL multi-modulus divider */
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);
	tmp |= GLB_PU_USBPLL_MMDIV_MSK;
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);

	k_usleep(5);

	/* RSTB toggle (1→0→1): reset the USB PLL divider */
	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);
	tmp |= GLB_USBPLL_RSTB_MSK;
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);

	k_usleep(5);

	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);
	tmp &= ~GLB_USBPLL_RSTB_MSK;
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);

	k_usleep(5);

	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);
	tmp |= GLB_USBPLL_RSTB_MSK;
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);
}

static void udc_bflb_v2_phy_init(const struct device *const dev)
{
	uint32_t tmp;

	tmp = sys_read32(PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
	tmp &= ~PDS_REG_USB_PHY_XTLSEL_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	tmp = sys_read32(PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
	tmp |= PDS_REG_PU_USB20_PSW_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	tmp = sys_read32(PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
	tmp |= PDS_REG_USB_PHY_PONRST_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	k_usleep(1);

	tmp = sys_read32(PDS_BASE + PDS_USB_CTL_OFFSET);
	tmp &= ~PDS_REG_USB_SW_RST_N_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_CTL_OFFSET);

	k_usleep(1);

	tmp = sys_read32(PDS_BASE + PDS_USB_CTL_OFFSET);
	tmp |= PDS_REG_USB_EXT_SUSP_N_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_CTL_OFFSET);

	k_msleep(5);

	tmp = sys_read32(PDS_BASE + PDS_USB_CTL_OFFSET);
	tmp |= PDS_REG_USB_SW_RST_N_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_CTL_OFFSET);

	k_msleep(5);
}

static int udc_bflb_v2_init(const struct device *const dev)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	int ret;

	udc_bflb_v2_clock_init(dev);
	udc_bflb_v2_phy_init(dev);

	/* Disconnect all EPs from FIFOs: 4 data FIFOs + 1 control FIFO,
	 * mapped via EPMAP0 (EP1-4) and EPMAP1 (EP5-8). 0xF = no EP.
	 */
	sys_write32(0xffffffffU, cfg->base + USB_DEV_EPMAP0_OFFSET);
	sys_write32(0xffU, cfg->base + USB_DEV_EPMAP1_OFFSET);
	udc_bflb_v2_fifo_setep(dev, USB_BFLB_V2_FIFO_EP_NONE, 1, USB_BFLB_V2_FIFO_DIR_OUT);
	udc_bflb_v2_fifo_setep(dev, USB_BFLB_V2_FIFO_EP_NONE, 2, USB_BFLB_V2_FIFO_DIR_OUT);
	udc_bflb_v2_fifo_setep(dev, USB_BFLB_V2_FIFO_EP_NONE, 3, USB_BFLB_V2_FIFO_DIR_OUT);
	udc_bflb_v2_fifo_setep(dev, USB_BFLB_V2_FIFO_EP_NONE, 4, USB_BFLB_V2_FIFO_DIR_OUT);

	udc_bflb_v2_fifo_reset_ctrl(dev);
	udc_bflb_v2_fifo_reset(dev, 1);
	udc_bflb_v2_fifo_reset(dev, 2);
	udc_bflb_v2_fifo_reset(dev, 3);
	udc_bflb_v2_fifo_reset(dev, 4);

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL, 64, 0);
	if (ret < 0) {
		LOG_ERR("Failed to enable control endpoint");
		return ret;
	}

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL, 64, 0);
	if (ret < 0) {
		LOG_ERR("Failed to enable control endpoint");
		return ret;
	}

	cfg->irq_enable_func(dev);

	return 0;
}

/* Shut down the controller completely */
static int udc_bflb_v2_shutdown(const struct device *const dev)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	uint32_t tmp;

	cfg->irq_disable_func(dev);

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	tmp = sys_read32(PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
	tmp &= ~PDS_REG_USB_PHY_XTLSEL_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	tmp = sys_read32(PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
	tmp &= ~PDS_REG_PU_USB20_PSW_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	tmp = sys_read32(PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
	tmp &= ~PDS_REG_USB_PHY_PONRST_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	tmp = sys_read32(PDS_BASE + PDS_USB_CTL_OFFSET);
	tmp &= ~PDS_REG_USB_EXT_SUSP_N_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_CTL_OFFSET);

	return 0;
}

static int udc_bflb_v2_driver_preinit(const struct device *const dev)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	struct udc_data *const data = dev->data;
	uint16_t mps = 512;
	int err;
	int i;
#ifdef CONFIG_UDC_BFLB_V2_FIFO_BIDIR
	const int cnt = USB_BFLB_V2_NUM_BIDIR_EPS;
#else
	/* Plus one because testusb is apparently stupid */
	const int cnt = USB_BFLB_V2_NUM_MONODIR_EPS;
#endif

	k_mutex_init(&data->mutex);

	data->caps.rwup = true;
	data->caps.mps0 = UDC_MPS0_64;
	if (cfg->speed_idx == UDC_BUS_SPEED_HS) {
		data->caps.hs = true;
		mps = USB_BFLB_V2_MAXPS_HS;
	}

	for (i = 0; i < cnt; i++) {
		cfg->ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			cfg->ep_cfg_out[i].caps.control = 1;
			cfg->ep_cfg_out[i].caps.mps = 64;
		} else {
			cfg->ep_cfg_out[i].caps.bulk = 1;
			cfg->ep_cfg_out[i].caps.interrupt = 1;
			cfg->ep_cfg_out[i].caps.iso = 1;
			cfg->ep_cfg_out[i].caps.mps = mps;
		}

		cfg->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		err = udc_register_ep(dev, &cfg->ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	for (i = 0; i < cnt; i++) {
		cfg->ep_cfg_in[i].caps.in = 1;
		if (i == 0) {
			cfg->ep_cfg_in[i].caps.control = 1;
			cfg->ep_cfg_in[i].caps.mps = 64;
		} else {
			cfg->ep_cfg_in[i].caps.bulk = 1;
			cfg->ep_cfg_in[i].caps.interrupt = 1;
			cfg->ep_cfg_in[i].caps.iso = 1;
			cfg->ep_cfg_in[i].caps.mps = mps;
		}

		cfg->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		err = udc_register_ep(dev, &cfg->ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	return 0;
}

static void udc_bflb_v2_isr(const struct device *const dev)
{
	const struct udc_bflb_v2_config *const cfg = dev->config;
	struct udc_bflb_v2_data *const priv = udc_get_private(dev);
	uint32_t glb_intstatus;
	uint32_t dev_intstatus;
	uint32_t group_intstatus;
	uint32_t tmp;
	uint8_t ep;
	uint8_t ep_idx;
	uint8_t dir;
	struct net_buf *stale;
	uint32_t rxz;
	int i;

	glb_intstatus = sys_read32(cfg->base + USB_GLB_ISR_OFFSET);

	if (glb_intstatus & USB_DEV_INT) {
		dev_intstatus = sys_read32(cfg->base + USB_DEV_IGR_OFFSET);
		if (dev_intstatus & USB_INT_G0) {
			group_intstatus = sys_read32(cfg->base + USB_DEV_ISG0_OFFSET);
			group_intstatus &= ~sys_read32(cfg->base + USB_DEV_MISG0_OFFSET);

			if (group_intstatus & USB_CX_COMABT_INT) {
				udc_submit_event(dev, UDC_EVT_ERROR, -ECANCELED);
				LOG_ERR("Control command abort");
			}

			if (group_intstatus & USB_CX_SETUP_INT) {
				/* Mask setup interrupt until processing
				 * completes. Without this, the host's setup
				 * retries flood the ISR and starve the work
				 * queue that processes them.
				 */
				tmp = sys_read32(cfg->base + USB_DEV_MISG0_OFFSET);
				tmp |= USB_MCX_SETUP_INT;
				sys_write32(tmp, cfg->base + USB_DEV_MISG0_OFFSET);

				/* Setup packet implies bus is active */
				if (udc_is_suspended(dev)) {
					udc_set_suspended(dev, false);
				}

				priv->ep_is_in[0] = false;
				priv->setup_received = true;
				udc_bflb_v2_ctrl_setup_start(dev);
			}

			if (group_intstatus & USB_CX_COMFAIL_INT) {
				udc_submit_event(dev, UDC_EVT_ERROR, -EIO);
				LOG_ERR("Control command fail");
			}

			/* Clear ALL pending G0 (control) interrupt status,
			 * including masked bits like CX_COMEND.  Leaving
			 * masked bits set can jam the CX engine and prevent
			 * it from accepting the next SETUP packet.
			 */
			sys_write32(sys_read32(cfg->base + USB_DEV_ISG0_OFFSET),
				    cfg->base + USB_DEV_ISG0_OFFSET);
		}
		if (dev_intstatus & USB_INT_G1) {
			group_intstatus = sys_read32(cfg->base + USB_DEV_ISG1_OFFSET);
			group_intstatus &= ~sys_read32(cfg->base + USB_DEV_MISG1_OFFSET);

			sys_write32(group_intstatus, cfg->base + USB_DEV_ISG1_OFFSET);

			/*
			 * G1 IN_INT: host read data from an IN FIFO.
			 * This is the true IN completion signal — the
			 * data has actually been sent to the host.
			 * F0_IN_INT=bit16 .. F3_IN_INT=bit19 correspond
			 * to driver fifo_idx 1..4.
			 */
			for (i = 1U; i <= USB_BFLB_V2_NUM_DATA_FIFOS; i++) {
				if (!(group_intstatus & (1U << (15U + i)))) {
					continue;
				}

				/* Re-mask this FIFO's IN_INT */
				tmp = sys_read32(cfg->base + USB_DEV_MISG1_OFFSET);
				tmp |= (1U << (15U + i));
				sys_write32(tmp, cfg->base + USB_DEV_MISG1_OFFSET);

				ep_idx = udc_bflb_v2_fifo_to_ep(dev, i);
				udc_bflb_v2_ev_submit(dev, USB_EP_DIR_IN | ep_idx,
						      UDC_BFLB_V2_EVT_END, K_NO_WAIT);
			}

			/*
			 * G1 OUT/SPK_INT: data has arrived in a FIFO.
			 * Start VDMA for the exact FIFO byte count so
			 * the transfer completes via G3 without hitting
			 * the hardware blind spot (16..MPS-1 bytes).
			 */
			for (i = 1U; i <= USB_BFLB_V2_NUM_DATA_FIFOS; i++) {
				uint32_t fibc;
				struct udc_ep_config *out_cfg;
				struct net_buf *out_buf;

				if (!(group_intstatus & (3U << (2U * (i - 1U))))) {
					continue;
				}

				/* Re-mask OUT and SPK for this FIFO */
				tmp = sys_read32(cfg->base + USB_DEV_MISG1_OFFSET);
				tmp |= (3U << (2U * (i - 1U)));
				sys_write32(tmp, cfg->base + USB_DEV_MISG1_OFFSET);

				/* Read FIFO byte count */
				fibc = sys_read32(cfg->base + USB_DEV_FIBC0_OFFSET
						  + 4U * (i - 1U));
				fibc &= USB_BC_F0_MASK; /* byte count in bits [10:0] */

				ep_idx = udc_bflb_v2_fifo_to_ep(dev, i);
				out_cfg = udc_get_ep_cfg(dev,
							 USB_EP_DIR_OUT | ep_idx);
				if (out_cfg == NULL) {
					continue;
				}
				out_buf = udc_buf_peek(out_cfg);
				if (out_buf == NULL || fibc == 0U) {
					continue;
				}

				/* Start VDMA for exactly the bytes in FIFO */
				priv->out_vdma_len[i] = fibc;
				udc_bflb_v2_vdma_startread(dev, i,
					out_buf->data + out_buf->len,
					fibc);
			}
		}
		if (dev_intstatus & USB_INT_G2) {
			group_intstatus = sys_read32(cfg->base + USB_DEV_ISG2_OFFSET);
			group_intstatus &= ~sys_read32(cfg->base + USB_DEV_MISG2_OFFSET);

			/* suspended */
			if (group_intstatus & USB_SUSP_INT) {
				sys_write32(USB_SUSP_INT, cfg->base + USB_DEV_ISG2_OFFSET);

				udc_set_suspended(dev, true);
				udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
			}

			/* resumed — only act on genuine resume after suspend */
			if ((group_intstatus & USB_RESM_INT) && udc_is_suspended(dev)) {
				sys_write32(USB_RESM_INT, cfg->base + USB_DEV_ISG2_OFFSET);

				udc_set_suspended(dev, false);
				udc_submit_event(dev, UDC_EVT_RESUME, 0);
			} else if (group_intstatus & USB_RESM_INT) {
				/* Clear the spurious RESM status bit */
				sys_write32(USB_RESM_INT, cfg->base + USB_DEV_ISG2_OFFSET);
			}

			if (group_intstatus & USBRST_INT) {
				sys_write32(USBRST_INT, cfg->base + USB_DEV_ISG2_OFFSET);

				/* Clear AFT_CONF - device returns to default
				 * state */
				tmp = sys_read32(cfg->base + USB_DEV_ADR_OFFSET);
				tmp &= ~USB_AFT_CONF;
				sys_write32(tmp, cfg->base + USB_DEV_ADR_OFFSET);

				udc_bflb_v2_fifo_reset_ctrl(dev);
				udc_bflb_v2_fifo_reset(dev, 1);
				udc_bflb_v2_fifo_reset(dev, 2);
				udc_bflb_v2_fifo_reset(dev, 3);
				udc_bflb_v2_fifo_reset(dev, 4);
				memset(priv->fifo_active, 0, sizeof(priv->fifo_active));

				/* Clear any spurious VDMA completions from FIFO
				 * reset */
				sys_write32(0x1FU, cfg->base + USB_DEV_ISG3_OFFSET);

				/* Clear stale setup state and drain any buffers
				 * left from an interrupted control transfer.
				 */
				priv->setup_received = false;
				stale = udc_buf_get(udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT));
				while (stale != NULL) {
					net_buf_unref(stale);
					stale = udc_buf_get(
						udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT));
				}
				stale = udc_buf_get(udc_get_ep_cfg(dev, USB_CONTROL_EP_IN));
				while (stale != NULL) {
					net_buf_unref(stale);
					stale = udc_buf_get(udc_get_ep_cfg(dev, USB_CONTROL_EP_IN));
				}

				/* Bus reset implies bus is active */
				if (udc_is_suspended(dev)) {
					udc_set_suspended(dev, false);
				}

				tmp = sys_read32(cfg->base + USB_DEV_SMT_OFFSET);
				tmp &= ~USB_SOFMT_MASK;
				if (cfg->speed_idx == UDC_BUS_SPEED_HS) {
					tmp |= USB_BFLB_V2_TIMER_AFTER_RESET_HS;
				} else {
					tmp |= USB_BFLB_V2_TIMER_AFTER_RESET_FS;
				}
				sys_write32(tmp, cfg->base + USB_DEV_SMT_OFFSET);

				/* Re-enable setup interrupt (may have been
				 * masked) */
				tmp = sys_read32(cfg->base + USB_DEV_MISG0_OFFSET);
				tmp &= ~USB_MCX_SETUP_INT;
				sys_write32(tmp, cfg->base + USB_DEV_MISG0_OFFSET);

				priv->reset_expiration =
					sys_timepoint_calc(USB_BFLB_V2_TIMER_AFTER_RESET_T);

				udc_submit_event(dev, UDC_EVT_RESET, 0);
			}

			if (group_intstatus & USB_ISO_SEQ_ERR_INT) {
				sys_write32(USB_ISO_SEQ_ERR_INT, cfg->base + USB_DEV_ISG2_OFFSET);

				udc_submit_event(dev, UDC_EVT_ERROR, -EIO);
				LOG_ERR("Isochronous sequence error");
			}

			if (group_intstatus & USB_ISO_SEQ_ABORT_INT) {
				sys_write32(USB_ISO_SEQ_ABORT_INT, cfg->base + USB_DEV_ISG2_OFFSET);

				udc_submit_event(dev, UDC_EVT_ERROR, -ECANCELED);
				LOG_ERR("Isochronous sequence aborted");
			}

			/* RX0BYTE: a ZLP was received on a data OUT endpoint.
			 * This fires even when VDMA is not running, unlike the
			 * G3 short-packet completion.  Use it to complete OUT
			 * transfers whose terminating ZLP arrived during the
			 * VDMA re-arm gap (the MPS-size transfer problem).
			 *
			 * DEV_RXZ is EP-indexed: bit 0 = EP1, bit 1 = EP2, …
			 */
			if (group_intstatus & USB_RX0BYTE_INT) {
				rxz = sys_read32(cfg->base + USB_DEV_RXZ_OFFSET);

				for (ep = 1U; ep < USB_BFLB_V2_NUM_BIDIR_EPS; ep++) {
					if (!(rxz & (1U << (ep - 1U)))) {
						continue;
					}
					/* Clear this EP's ZLP status */
					sys_write32(1U << (ep - 1U),
						    cfg->base + USB_DEV_RXZ_OFFSET);

					udc_bflb_v2_ev_submit(dev, USB_EP_DIR_OUT | ep,
							      UDC_BFLB_V2_EVT_END, K_NO_WAIT);
				}
				sys_write32(USB_RX0BYTE_INT, cfg->base + USB_DEV_ISG2_OFFSET);
			}
		}
		if (dev_intstatus & USB_INT_G3) {
			group_intstatus = sys_read32(cfg->base + USB_DEV_ISG3_OFFSET);
			group_intstatus &= ~sys_read32(cfg->base + USB_DEV_MISG3_OFFSET);
			sys_write32(group_intstatus, cfg->base + USB_DEV_ISG3_OFFSET);

			if (group_intstatus & USB_VDMA_CMPLT_CXF) {
				if (priv->ep_is_in[0]) {
					udc_bflb_v2_ev_submit(dev,
						USB_CONTROL_EP_IN, UDC_BFLB_V2_EVT_CTRL_END,
						K_NO_WAIT);
					udc_bflb_v2_ctrl_ack(dev);
				} else {
					udc_bflb_v2_ev_submit(dev,
						USB_CONTROL_EP_OUT, UDC_BFLB_V2_EVT_CTRL_END,
						K_NO_WAIT);
				}
			}

			for (i = 1U; i <= USB_BFLB_V2_NUM_DATA_FIFOS; i++) {
				if (!(group_intstatus & (1U << i))) {
					continue;
				}
				ep_idx = udc_bflb_v2_fifo_to_ep(dev, i);
#if CONFIG_UDC_BFLB_V2_FIFO_BIDIR
				dir = priv->ep_is_in[ep_idx] ? USB_EP_DIR_IN
								: USB_EP_DIR_OUT;
#else
				dir = (i & 1U) ? USB_EP_DIR_OUT : USB_EP_DIR_IN;
#endif

				/*
				 * For IN, G3 just confirms VDMA finished;
				 * G1 IN_INT was already unmasked
				 * synchronously in ep_din_start after
				 * polling VDMA completion.  Nothing to
				 * do here.
				 * For OUT, G3 means data is in memory.
				 * Stop the VDMA engine before submitting
				 * the completion event so the remain count
				 * register is stable when the work handler
				 * reads it.
				 */
				if (dir == USB_EP_DIR_IN) {
					continue;
				}
				udc_bflb_v2_vdma_stop(dev, i);
				udc_bflb_v2_ev_submit(dev, dir | ep_idx, UDC_BFLB_V2_EVT_END,
						      K_NO_WAIT);
			}
		}
		if (dev_intstatus & USB_INT_G4) {
			/* Nothing we care about in group 4 */
		}
	}
}

static void udc_bflb_v2_lock(const struct device *const dev)
{
	udc_lock_internal(dev, K_FOREVER);
}

static void udc_bflb_v2_unlock(const struct device *const dev)
{
	udc_unlock_internal(dev);
}

static const struct udc_api udc_bflb_v2_api = {
	.lock = udc_bflb_v2_lock,
	.unlock = udc_bflb_v2_unlock,
	.device_speed = udc_bflb_v2_device_speed,
	.init = udc_bflb_v2_init,
	.enable = udc_bflb_v2_enable,
	.disable = udc_bflb_v2_disable,
	.shutdown = udc_bflb_v2_shutdown,
	.set_address = udc_bflb_v2_set_address,
	.host_wakeup = udc_bflb_v2_host_wakeup,
	.ep_enable = udc_bflb_v2_ep_enable,
	.ep_disable = udc_bflb_v2_ep_disable,
	.ep_set_halt = udc_bflb_v2_ep_set_halt,
	.ep_clear_halt = udc_bflb_v2_ep_clear_halt,
	.ep_enqueue = udc_bflb_v2_ep_enqueue,
	.ep_dequeue = udc_bflb_v2_ep_dequeue,
};

#define UDC_BFLB_UDC_2_DEVICE_DEFINE(n)						\
	static void udc_irq_enable_func##n(const struct device *const dev)	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n),					\
			    DT_INST_IRQ(n, priority),				\
			    udc_bflb_v2_isr,					\
			    DEVICE_DT_INST_GET(n), 0);				\
										\
		irq_enable(DT_INST_IRQN(n));					\
	}									\
										\
	static void udc_irq_disable_func##n(const struct device *const dev)	\
	{									\
		irq_disable(DT_INST_IRQN(n));					\
	}									\
										\
	static struct udc_ep_config ep_cfg_out[USB_BFLB_V2_NUM_BIDIR_EPS];	\
	static struct udc_ep_config ep_cfg_in[USB_BFLB_V2_NUM_BIDIR_EPS];	\
										\
	static const struct udc_bflb_v2_config udc_bflb_v2_config_##n = {	\
		.base = DT_INST_REG_ADDR(n),					\
		.ep_cfg_in = ep_cfg_out,					\
		.ep_cfg_out = ep_cfg_in,					\
		.speed_idx = DT_ENUM_IDX(DT_DRV_INST(n), maximum_speed),	\
		.irq_enable_func = udc_irq_enable_func##n,			\
		.irq_disable_func = udc_irq_disable_func##n,			\
		.zero_buff = {0},						\
	};									\
										\
	static struct udc_bflb_v2_data udc_priv_##n = {				\
		.setup_received = false,					\
	};									\
										\
	static struct udc_data udc_data_##n = {					\
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),		\
		.priv = &udc_priv_##n,						\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, udc_bflb_v2_driver_preinit, NULL,		\
			      &udc_data_##n, &udc_bflb_v2_config_##n,	\
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			      &udc_bflb_v2_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_BFLB_UDC_2_DEVICE_DEFINE)
