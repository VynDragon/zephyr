/*
 * Copyright (c) 2024 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "udc_common.h"

#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udc_bflb_bl61x, CONFIG_UDC_DRIVER_LOG_LEVEL);

#include <soc.h>
#include <bouffalolab/common/usb_v2_reg.h>

struct udc_bflb_bl61x_config {
	uint32_t base;
	size_t num_of_eps;
	void (*irq_enable_func)(const struct device *dev);
	void (*irq_disable_func)(const struct device *dev);
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	int speed_idx;
};


struct udc_bflb_bl61x_data {
	struct k_work work;
	struct k_fifo fifo;
	bool ep_in[5];
};

enum udc_bflb_bl61x_ev_type {
	/* Trigger next transfer */
	UBFBL61X_EVT_XFER,
	/* packet dma complete  for ctrl fifo*/
	UBFBL61X_EVT_CTRL_END,
	/* packet dma complete for specific endpoint */
	UBFBL61X_EVT_END,
};

struct udc_bflb_bl61x_ev {
	void *reserved;
	const struct device *dev;
	uint8_t ep_addr;
	enum udc_bflb_bl61x_ev_type event;
};

K_MEM_SLAB_DEFINE(udc_bflb_bl61x_ev_slab, sizeof(struct udc_bflb_bl61x_ev),
		  CONFIG_UDC_BFLB_BL61X_EVENT_COUNT, sizeof(void *));


static void udc_bflb_bl61x_ev_submit(const struct device *dev,
				 const uint8_t ep_addr,
				 const enum udc_bflb_bl61x_ev_type event)
{
	struct udc_bflb_bl61x_data *priv = udc_get_private(dev);
	struct udc_bflb_bl61x_ev *ev;
	int ret;

	ret = k_mem_slab_alloc(&udc_bflb_bl61x_ev_slab, (void **)&ev, K_NO_WAIT);
	if (ret) {
		udc_submit_event(dev, UDC_EVT_ERROR, ret);
		LOG_ERR("Failed to allocate slab");
		return;
	}

	ev->dev = dev;
	ev->ep_addr = ep_addr;
	ev->event = event;
	k_fifo_put(&priv->fifo, ev);
	k_work_submit_to_queue(udc_get_work_q(), &priv->work);
}

static uint8_t udc_bflb_bl61x_epcaps_to_eptype(const struct udc_ep_caps * const caps)
{
	uint8_t ret = 0;

	if (caps->control) {
	}
	if (caps->interrupt) {
		ret |= 3;
	}
	if (caps->bulk) {
		ret |= 2;
	}
	if (caps->iso) {
		ret |= 1;
	}
	return ret;
}

static void udc_bflb_bl61x_fifo_configure(const struct device *dev,
					uint8_t fifo_idx,
					uint8_t ep_type,
					uint16_t block_size,
					uint8_t block_num,
					bool enabled)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	uint32_t tmp = 0;

	if (fifo_idx > 3) {
		return;
	}

	tmp = sys_read32(cfg->base + USB_DEV_FCFG_OFFSET);
	tmp &= ~(0x3f << (fifo_idx * 8));
	tmp |= (ep_type << (fifo_idx * 8 + 0));
	tmp |= ((block_num - 1) << (fifo_idx * 8 + 2));
	if (block_size > 512) {
		tmp |= (1 << (fifo_idx * 8 + 4));
	}
	if (enabled) {
		tmp |= (1 << (fifo_idx * 8 + 5));
	} else {
		tmp &= ~(1 << (fifo_idx * 8 + 5));
	}
	sys_write32(tmp, cfg->base + USB_DEV_FCFG_OFFSET);
}
static void udc_bflb_bl61x_ep_set_out_mps(const struct device *dev,
					uint8_t ep_idx,
					uint16_t ep_mps)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	uint32_t tmp = 0;

	tmp = sys_read32(cfg->base + USB_DEV_OUTMPS1_OFFSET
		+ (ep_idx - 1) * 4);
	tmp &= ~USB_MAXPS_OEP1_MASK;
	tmp |= ep_mps;
	sys_write32(tmp, cfg->base + USB_DEV_OUTMPS1_OFFSET
		+ (ep_idx - 1) * 4);
}

static void udc_bflb_bl61x_ep_set_in_mps(const struct device *dev,
					uint8_t ep_idx,
					uint16_t ep_mps)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	uint32_t tmp = 0;

	tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET
		+ (ep_idx - 1) * 4);
	tmp &= ~USB_MAXPS_IEP1_MASK;
	tmp |= ep_mps;
	tmp &= ~USB_TX_NUM_HBW_IEP1_MASK;
	sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET
		+ (ep_idx - 1) * 4);
}

static void udc_bflb_bl61x_fifo_reset_ctrl(const struct device *dev)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	uint32_t tmp = 0;

	tmp = sys_read32(cfg->base + USB_DEV_CXCFE_OFFSET);
        tmp |= USB_CX_CLR;
        sys_write32(tmp, cfg->base + USB_DEV_CXCFE_OFFSET);
}

/* fifo_idx : 0-3
*/
static void udc_bflb_bl61x_fifo_reset(const struct device *dev, uint8_t fifo_idx)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	uint32_t tmp = 0;

	tmp = sys_read32(cfg->base + USB_DEV_FIBC0_OFFSET + 4 * fifo_idx);
        tmp |= USB_FFRST0_HOV;
        sys_write32(tmp, cfg->base + USB_DEV_FIBC0_OFFSET + 4 * fifo_idx);
}

/* fifo_idx : 0-3, ep_idx: 0-4
 * ep_direction: 0 in 1 out
 */
static void udc_bflb_bl61x_ep_setfifo(const struct device *dev,
					uint8_t ep_idx,
					uint8_t fifo_idx,
					uint8_t ep_dir)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	uint32_t tmp = 0;

	if (ep_idx > 4 || ep_dir > 1 || fifo_idx > 3) {
		return;
	}
	ep_dir = ep_dir * 4;

	if (ep_idx < 4) {
		tmp = sys_read32(cfg->base + USB_DEV_EPMAP0_OFFSET);
		tmp &= ~(0xf << (ep_idx * 8 + ep_dir));
		tmp |= (fifo_idx << (ep_idx * 8 + ep_dir));
		sys_write32(tmp, cfg->base + USB_DEV_EPMAP0_OFFSET);
	} else {
		tmp = sys_read32(cfg->base + USB_DEV_EPMAP1_OFFSET);
		tmp &= ~(0xf << ep_dir);
		tmp |= (fifo_idx << ep_dir);
		sys_write32(tmp, cfg->base + USB_DEV_EPMAP1_OFFSET);
	}
}

/* fifo_idx : 0-3, ep_idx: 0-4
 * ep_direction: 0 in 1 out
 * fifo_direction: 0 out 1 in 2 bidirectional
 */
static void udc_bflb_bl61x_fifo_setep(const struct device *dev,
					uint8_t ep_idx,
					uint8_t fifo_idx,
					uint8_t fifo_dir)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	uint32_t tmp = 0;

	if (ep_idx > 4 || fifo_idx > 3 || fifo_dir > 2) {
		return;
	}

	tmp = sys_read32(cfg->base + USB_DEV_FMAP_OFFSET);
	tmp &= ~(0x3f << (fifo_idx * 8));
	tmp |= (ep_idx << (fifo_idx * 8));
	tmp |= (fifo_dir << (fifo_idx * 8 + 4));
	sys_write32(tmp, cfg->base + USB_DEV_FMAP_OFFSET);
}

static uint8_t __attribute__((aligned(256))) vdma_buffer0[1024] = {0};
static uint8_t __attribute__((aligned(256))) vdma_buffer1[1024] = {0};
static uint8_t __attribute__((aligned(256))) vdma_buffer2[1024] = {0};
static uint8_t __attribute__((aligned(256))) vdma_buffer3[1024] = {0};

static uint8_t *vdma_buffer[4] = { vdma_buffer0, vdma_buffer1, vdma_buffer2, vdma_buffer3 };


/* bl61x cannot use cpu read/write for USB */
static void udc_bflb_bl61x_vdma_startread(const struct device *dev,
					uint8_t fifo_idx,
					uint32_t len)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	uint32_t tmp = 0;

	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_idx * 8);
	tmp &= ~USB_VDMA_LEN_CXF_MASK;
	tmp &= ~USB_VDMA_IO_CXF;
	tmp &= ~USB_VDMA_TYPE_CXF;
	tmp |= (len << USB_VDMA_LEN_CXF_SHIFT);
	sys_write32(tmp, cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_idx * 8);

	sys_write32((uint32_t)vdma_buffer[fifo_idx], cfg->base + USB_VDMA_F0PS2_OFFSET + fifo_idx * 8);

	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_idx * 8);
	tmp |= USB_VDMA_START_CXF;
	sys_write32(tmp, cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_idx * 8);
}

static void udc_bflb_bl61x_vdma_startwrite(const struct device *dev,
					uint8_t fifo_idx, uint8_t *data,
					uint32_t len)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	uint32_t tmp = 0;

	memcpy(vdma_buffer[fifo_idx], data, len);

	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_idx * 8);
	tmp &= ~USB_VDMA_LEN_CXF_MASK;
	tmp &= ~USB_VDMA_IO_CXF;
	tmp |= USB_VDMA_TYPE_CXF;
	tmp |= (len << USB_VDMA_LEN_CXF_SHIFT);
	sys_write32(tmp, cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_idx * 8);

	sys_write32((uint32_t)vdma_buffer[fifo_idx], cfg->base + USB_VDMA_F0PS2_OFFSET + fifo_idx * 8);

	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_idx * 8);
	tmp |= USB_VDMA_START_CXF;
	sys_write32(tmp, cfg->base + USB_VDMA_F0PS1_OFFSET + fifo_idx * 8);
}

static uint8_t __attribute__((aligned(256))) vdma_ctrl_buffer[64] = {0};

static void udc_bflb_bl61x_vdma_startread_ctrl(const struct device *dev, uint32_t len)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	uint32_t tmp = 0;

	tmp = sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET);
	tmp &= ~USB_VDMA_LEN_CXF_MASK;
	tmp &= ~USB_VDMA_IO_CXF;
	tmp &= ~USB_VDMA_TYPE_CXF;
	tmp |= (len << USB_VDMA_LEN_CXF_SHIFT);
	sys_write32(tmp, cfg->base + USB_VDMA_CXFPS1_OFFSET);

	sys_write32((uint32_t)vdma_ctrl_buffer, cfg->base + USB_VDMA_CXFPS2_OFFSET);

	tmp = sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET);
	tmp |= USB_VDMA_START_CXF;
	sys_write32(tmp, cfg->base + USB_VDMA_CXFPS1_OFFSET);
}

static void udc_bflb_bl61x_vdma_startwrite_ctrl(const struct device *dev,
						uint8_t *data, uint32_t len)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	uint32_t tmp = 0;

	memcpy(vdma_ctrl_buffer, data, len);

	tmp = sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET);
	tmp &= ~USB_VDMA_LEN_CXF_MASK;
	tmp &= ~USB_VDMA_IO_CXF;
	tmp |= USB_VDMA_TYPE_CXF;
	tmp |= (len << USB_VDMA_LEN_CXF_SHIFT);
	sys_write32(tmp, cfg->base + USB_VDMA_CXFPS1_OFFSET);

	sys_write32((uint32_t)vdma_ctrl_buffer, cfg->base + USB_VDMA_CXFPS2_OFFSET);

	tmp = sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET);
	tmp |= USB_VDMA_START_CXF;
	sys_write32(tmp, cfg->base + USB_VDMA_CXFPS1_OFFSET);
}

static uint8_t udc_bflb_bl61x_ep_get_fifo(struct udc_ep_config * const ep_cfg)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep_cfg->addr);

	if (ep_cfg->mps > 512) {
		if (ep_idx == 1) {
			return 0;
		} else {
			return 2;
		}
	}
	return ep_idx - 1;
}

static uint8_t udc_bflb_bl61x_fifo_get_ep(const struct device *dev, uint8_t fifo)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;


	if (udc_bflb_bl61x_device_speed(dev) == UDC_BUS_SPEED_FS) {
		return fifo + 1 ;
	}

	if (fifo < 2) {
		return 1;
	}
	return 2;
}

static void udc_bflb_bl61x_xfer_start(const struct device *dev,
				  struct udc_ep_config *const config,
				  struct net_buf *const buf)
{
	uint8_t *data_ptr;
	size_t len;
	uint32_t tmp = 0;
	uint8_t ep_idx = USB_EP_GET_IDX(config->addr);

	if (USB_EP_DIR_IS_OUT(config->addr)) {
		len = MIN(net_buf_tailroom(buf), config->mps);
		data_ptr = net_buf_tail(buf);
		if (ep_idx == 0) {
			udc_bflb_bl61x_vdma_startread_ctrl(dev, len);
		} else {
			udc_bflb_bl61x_vdma_startread(dev,
				udc_bflb_bl61x_ep_get_fifo(config), len);
		}
	} else {
		len = MIN(buf->len, config->mps);
		data_ptr = buf->data;
		if (ep_idx == 0) {
			udc_bflb_bl61x_vdma_startwrite_ctrl(dev, data_ptr, len);
		} else {
			udc_bflb_bl61x_vdma_startwrite(dev,
				udc_bflb_bl61x_ep_get_fifo(config),
				       data_ptr, len);
		}
	}
}

static int udc_bflb_bl61x_xfer_next(const struct device *dev,
			      struct udc_ep_config *const cfg)
{
	struct net_buf *buf;

	buf = udc_buf_peek(dev, cfg->addr);
	if (buf == NULL) {
		return -ENODATA;
	}

	udc_bflb_bl61x_xfer_start(dev, cfg, buf);
	return 0;
}

static int udc_bflb_bl61x_ctrl_feed_dout(const struct device *dev,
				   const size_t length)
{
	struct net_buf *buf;
	struct udc_ep_config *ep_cfg;

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, length);
	if (buf == NULL) {
		return -ENOMEM;
	}

	ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);

	if (unlikely(ep_cfg == NULL)) {
		return -ENODATA;
	}

	udc_bflb_bl61x_xfer_start(dev, ep_cfg, buf);

	return 0;
}


static void udc_bflb_bl61x_ctrl_setup_packet(const struct device *dev)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	struct udc_bflb_bl61x_data *priv = udc_get_private(dev);
	struct net_buf *buf;
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	uint8_t *data_ptr;

	/* clear ctrl fifo transfer complete isr*/
	sys_write32(USB_VDMA_CMPLT_CXF, cfg->base + USB_DEV_ISG3_OFFSET);

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, 8U);
	if (buf == NULL) {
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOMEM);
	}

	udc_ep_buf_set_setup(buf);
	udc_buf_put(ep_cfg, buf);

	udc_bflb_bl61x_vdma_startread_ctrl(dev, 8U);

	while (sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET) & USB_VDMA_START_CXF) {
	}

	memcpy(buf->data, vdma_ctrl_buffer, 8U);

	sys_write32(USB_VDMA_START_CXF, cfg->base + USB_DEV_ISG3_OFFSET);
	udc_ctrl_update_stage(dev, buf);
	net_buf_unref(buf);
}

static void udc_bflb_bl61x_ctrl_dout_end(const struct device *dev)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	struct net_buf *buf;
	uint8_t *data_ptr;
	uint32_t len;

	buf = udc_buf_peek(dev, USB_CONTROL_EP_OUT);
	udc_ctrl_update_stage(dev, buf);
	udc_ctrl_submit_s_out_status(dev, buf);
}


static inline int work_handler_setup(const struct device *dev)
{
	struct net_buf *buf;
	int err;

	buf = udc_buf_get(dev, USB_CONTROL_EP_OUT);
	if (buf == NULL) {
		return -ENODATA;
	}

	/* Update to next stage of control transfer */
	udc_ctrl_update_stage(dev, buf);

	if (udc_ctrl_stage_is_data_out(dev)) {
		/*  Allocate and feed buffer for data OUT stage */
		LOG_DBG("s:%p|feed for -out-", buf);
		err = udc_bflb_bl61x_ctrl_feed_dout(dev,
			udc_data_stage_length(buf));
		if (err == -ENOMEM) {
			err = udc_submit_ep_event(dev, buf, err);
		}
	} else if (udc_ctrl_stage_is_data_in(dev)) {
		/*
		 * Here we have to feed both descriptor tables so that
		 * no setup packets are lost in case of successive
		 * status OUT stage and next setup.
		 */
		LOG_DBG("s:%p|feed for -in-status >setup", buf);
		err = udc_bflb_bl61x_ctrl_feed_dout(dev, 8U);
		if (err == 0) {
			err = udc_bflb_bl61x_ctrl_feed_dout(dev, 8U);
		}

		/* Finally alloc buffer for IN and submit to upper layer */
		if (err == 0) {
			err = udc_ctrl_submit_s_in_status(dev);
		}
	} else {
		LOG_DBG("s:%p|feed >setup", buf);
		/*
		 * For all other cases we feed with a buffer
		 * large enough for setup packet.
		 */
		err = udc_bflb_bl61x_ctrl_feed_dout(dev, 8U);
		if (err == 0) {
			err = udc_ctrl_submit_s_status(dev);
		}
	}

	return err;
}

static inline int work_handler_out(const struct device *dev,
				   const uint8_t ep_addr)
{
	struct net_buf *buf;
	int err = 0;

	buf = udc_buf_get(dev, ep_addr);
	if (buf == NULL) {
		return -ENODATA;
	}

	if (ep_addr == USB_CONTROL_EP_OUT) {
		if (udc_ctrl_stage_is_status_out(dev)) {
			/* s-in-status finished, next bd is already fed */
			LOG_DBG("dout:%p|no feed", buf);
			/* Status stage finished, notify upper layer */
			udc_ctrl_submit_status(dev, buf);
		} else {
			/*
			 * For all other cases we feed with a buffer
			 * large enough for setup packet.
			 */
			LOG_DBG("dout:%p|feed >setup", buf);
			err = udc_bflb_bl61x_ctrl_feed_dout(dev, 8U);
		}

		/* Update to next stage of control transfer */
		udc_ctrl_update_stage(dev, buf);

		if (udc_ctrl_stage_is_status_in(dev)) {
			err = udc_ctrl_submit_s_out_status(dev, buf);
		}
	} else {
		err = udc_submit_ep_event(dev, buf, 0);
	}

	return err;
}

static inline int work_handler_in(const struct device *dev,
				  const uint8_t ep_addr)
{
	struct net_buf *buf;

	buf = udc_buf_get(dev, ep_addr);
	if (buf == NULL) {
		return -ENODATA;
	}

	if (ep_addr == USB_CONTROL_EP_IN) {
		if (udc_ctrl_stage_is_status_in(dev) ||
		    udc_ctrl_stage_is_no_data(dev)) {
			/* Status stage finished, notify upper layer */
			udc_ctrl_submit_status(dev, buf);
		}

		/* Update to next stage of control transfer */
		udc_ctrl_update_stage(dev, buf);

		if (udc_ctrl_stage_is_status_out(dev)) {
			/*
			 * IN transfer finished, release buffer,
			 * control OUT buffer should be already fed.
			 */
			net_buf_unref(buf);
		}

		return 0;
	}

	return udc_submit_ep_event(dev, buf, 0);
}

static void udc_bflb_bl61x_work_handler(struct k_work *item)
{
	struct udc_bflb_bl61x_ev *ev;
	struct udc_bflb_bl61x_data *priv;
	struct udc_ep_config *ep_cfg;
	struct net_buf *buf;
	uint8_t *data_ptr;
	uint8_t ep_idx = 0;
	int err = 0;

	priv = CONTAINER_OF(item, struct udc_bflb_bl61x_data, work);

	while ((ev = k_fifo_get(&priv->fifo, K_NO_WAIT)) != NULL) {
		err = 0;
		LOG_DBG("dev %p, ep 0x%02x, event %u",
			ev->dev, ev->ep_addr, ev->event);

		ep_cfg = udc_get_ep_cfg(ev->dev, ev->ep_addr);

		if (unlikely(ep_cfg == NULL)) {
			err = -ENODATA;
			LOG_ERR("Unexpected Invalid Endpoint Configuration in Work Queue");
		} else {
			switch (ev->event) {
			case UBFBL61X_EVT_SETUP_END:
				//err = udc_bflb_bl61x_ctrl_setup_end(ev->dev);
				break;
			case UBFBL61X_EVT_DOUT_END:
				/*err = work_handler_out(ev->dev, ev->ep_addr);
				udc_ep_set_busy(ev->dev, ev->ep_addr, false);*/
				break;
			case UBFBL61X_EVT_DIN_END:
				/*err = work_handler_in(ev->dev, ev->ep_addr);
				udc_ep_set_busy(ev->dev, ev->ep_addr, false);*/
				break;
			case UBFBL61X_EVT_XFER:
			default:
				break;
			}
		}

		if (unlikely(err)) {
			udc_submit_event(ev->dev, UDC_EVT_ERROR, err);
		}

		/* Peek next transfer */
		/*if (ev->ep_addr != USB_CONTROL_EP_OUT && !udc_ep_is_busy(ev->dev, ev->ep_addr)) {
			if (udc_bflb_bl61x_xfer_next(ev->dev, ep_cfg) == 0) {
				udc_ep_set_busy(ev->dev, ev->ep_addr, true);
			}
		}*/

		k_mem_slab_free(&udc_bflb_bl61x_ev_slab, (void *)ev);
	}
}

static int udc_bflb_bl61x_ep_enqueue(const struct device *dev,
				   struct udc_ep_config *const cfg,
				   struct net_buf *buf)
{
	LOG_DBG("%p enqueue %p", dev, buf);

	udc_buf_put(cfg, buf);

	if (cfg->stat.halted) {
		LOG_DBG("ep 0x%02x halted", cfg->addr);
		return 0;
	}

	udc_bflb_bl61x_ev_submit(dev, cfg->addr, UBFBL61X_EVT_XFER);
	return 0;
}

static int udc_bflb_bl61x_ep_dequeue(const struct device *dev,
				   struct udc_ep_config *const cfg)
{
	unsigned int lock_key;
	struct net_buf *buf;

	lock_key = irq_lock();

	buf = udc_buf_get_all(dev, cfg->addr);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	irq_unlock(lock_key);

	return 0;
}


static int udc_bflb_bl61x_ep_enable(const struct device *dev,
				  struct udc_ep_config *const config)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	struct udc_bflb_bl61x_data *priv = udc_get_private(dev);
	uint32_t tmp = 0;
	uint8_t ep_idx = USB_EP_GET_IDX(config->addr);

	LOG_DBG("Enable ep 0x%02x", config->addr);

	if (USB_EP_DIR_IS_OUT(config->addr)) {
		udc_bflb_bl61x_ep_set_out_mps(dev, ep_idx, config->mps);
	} else {
		udc_bflb_bl61x_ep_set_in_mps(dev, ep_idx, config->mps);
	}

	if (config->mps > 512) {
		if (ep_idx > 2) {
			LOG_DBG("We need to use 2 FIFO per ep if mps > 512");
			return -ENOTSUP;
		}
		if (ep_idx == 1) {
			udc_bflb_bl61x_ep_setfifo(dev, ep_idx, 0, 0);
			udc_bflb_bl61x_ep_setfifo(dev, ep_idx, 0, 1);
			udc_bflb_bl61x_fifo_setep(dev, ep_idx, 0, 2);
			udc_bflb_bl61x_fifo_setep(dev, ep_idx, 1, 2);
			udc_bflb_bl61x_fifo_configure(dev, 0,
				udc_bflb_bl61x_epcaps_to_eptype(&config->caps),
				config->mps, 1, true);
			udc_bflb_bl61x_fifo_configure(dev, 1,
				udc_bflb_bl61x_epcaps_to_eptype(&config->caps),
				config->mps, 1, false);
		} else {
			udc_bflb_bl61x_ep_setfifo(dev, ep_idx, 2, 0);
			udc_bflb_bl61x_ep_setfifo(dev, ep_idx, 2, 1);
			udc_bflb_bl61x_fifo_setep(dev, ep_idx, 2, 2);
			udc_bflb_bl61x_fifo_setep(dev, ep_idx, 3, 2);
			udc_bflb_bl61x_fifo_configure(dev, 2,
				udc_bflb_bl61x_epcaps_to_eptype(&config->caps),
				config->mps, 1, true);
			udc_bflb_bl61x_fifo_configure(dev, 3,
				udc_bflb_bl61x_epcaps_to_eptype(&config->caps),
				config->mps, 1, false);
		}
	} else {
		udc_bflb_bl61x_ep_setfifo(dev, ep_idx, ep_idx - 1, 0);
		udc_bflb_bl61x_ep_setfifo(dev, ep_idx, ep_idx - 1, 1);
		udc_bflb_bl61x_fifo_setep(dev, ep_idx, ep_idx - 1, 2);
		udc_bflb_bl61x_fifo_configure(dev, ep_idx - 1,
			udc_bflb_bl61x_epcaps_to_eptype(&config->caps),
			config->mps, 1, true);
	}

	tmp = sys_read32(cfg->base + USB_DEV_ADR_OFFSET);
	tmp |= USB_AFT_CONF;
	sys_write32(tmp, cfg->base + USB_DEV_ADR_OFFSET);

	return 0;
}

/* cant disable */
static int udc_bflb_bl61x_ep_disable(const struct device *dev,
				   struct udc_ep_config *const cfg)
{
	LOG_DBG("Disable ep 0x%02x", cfg->addr);

	return 0;
}


static int udc_bflb_bl61x_ep_set_halt(const struct device *dev,
				    struct udc_ep_config *const config)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	uint32_t tmp = 0;
	uint8_t ep_idx = USB_EP_GET_IDX(config->addr);

	LOG_DBG("Set halt ep 0x%02x", config->addr);

	if (ep_idx == 0) {
		tmp = sys_read32(cfg->base + USB_DEV_CXCFE_OFFSET);
		tmp |= USB_CX_STL;
		sys_write32(tmp, cfg->base + USB_DEV_CXCFE_OFFSET);
	} else {
		if (USB_EP_DIR_IS_OUT(config->addr)) {
			tmp = sys_read32(cfg->base + USB_DEV_OUTMPS1_OFFSET
				+ (ep_idx - 1) * 4);
			tmp |= USB_STL_OEP1;
			sys_write32(tmp, cfg->base + USB_DEV_OUTMPS1_OFFSET
				+ (ep_idx - 1) * 4);
		} else {
			tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET
				+ (ep_idx - 1) * 4);
			tmp |= USB_STL_IEP1;
			sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET
				+ (ep_idx - 1) * 4);
		}
	}

	config->stat.halted = true;

	return 0;
}

static int udc_bflb_bl61x_ep_clear_halt(const struct device *dev,
				      struct udc_ep_config *const config)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	uint32_t tmp = 0;
	uint8_t ep_idx = USB_EP_GET_IDX(config->addr);

	LOG_DBG("Clear halt ep 0x%02x", config->addr);

	if (ep_idx == 0) {
	} else {
		if (USB_EP_DIR_IS_OUT(config->addr)) {
			tmp = sys_read32(cfg->base + USB_DEV_OUTMPS1_OFFSET
				+ (ep_idx - 1) * 4);
			tmp &= ~USB_STL_OEP1;
			sys_write32(tmp, cfg->base + USB_DEV_OUTMPS1_OFFSET
				+ (ep_idx - 1) * 4);
		} else {
			tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET
				+ (ep_idx - 1) * 4);
			tmp &= ~USB_STL_IEP1;
			sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET
				+ (ep_idx - 1) * 4);
		}
		udc_bflb_bl61x_ev_submit(dev, config->addr, UBFBL61X_EVT_XFER);
	}

	config->stat.halted = false;

	return 0;
}

static int udc_bflb_bl61x_set_address(const struct device *dev, const uint8_t addr)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	uint32_t tmp = 0;

	LOG_DBG("Set new address %u for %p", addr, dev);

	tmp = sys_read32(cfg->base + USB_DEV_ADR_OFFSET);
	tmp &= ~USB_DEVADR_MASK;
	tmp |= addr;
	sys_write32(tmp, cfg->base + USB_DEV_ADR_OFFSET);

	return 0;
}

static int udc_bflb_bl61x_host_wakeup(const struct device *dev)
{
	LOG_DBG("Remote wakeup from %p", dev);

	return -ENOTSUP;
}

static enum udc_bus_speed udc_bflb_bl61x_device_speed(const struct device *dev)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	uint32_t speed = 3;

	speed = sys_read32(cfg->base + USB_OTG_CSR_OFFSET);
	speed &= USB_SPD_TYP_HOV_POV_MASK;
	speed = speed >> USB_SPD_TYP_HOV_POV_SHIFT;

	if (speed == 0) {
		return UDC_BUS_SPEED_FS;
	} else if (speed == 1) {
		return UDC_BUS_UNKNOWN;
	} else if (speed == 2) {
		return UDC_BUS_SPEED_HS;
	}

	return UDC_BUS_UNKNOWN;
}

static int udc_bflb_bl61x_enable(const struct device *dev)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	uint32_t tmp = 0;

	LOG_DBG("Enable device %p", dev);

	/* 'disable global irq' */
	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp &= ~USB_GLINT_EN_HOV;
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	/* force unplug signal? */
	tmp = sys_read32(cfg->base + USB_PHY_TST_OFFSET);
	tmp |= USB_UNPLUG;
	sys_write32(tmp, cfg->base + USB_PHY_TST_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp &= ~USB_CAP_RMWAKUP;
	tmp |= USB_CHIP_EN_HOV;
	if (cfg->speed_idx < 2) {
		tmp |= USB_FORCE_FS;
	} else {
		tmp &= ~USB_FORCE_FS;
	}
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp |= USB_SFRST_HOV;
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	/* wait for soft reset */
	while ((sys_read32(cfg->base + USB_DEV_CTL_OFFSET)
		& USB_SFRST_HOV) != 0) {
	}

	tmp = sys_read32(cfg->base + USB_DEV_ADR_OFFSET);
	tmp &= ~USB_AFT_CONF;
	sys_write32(tmp, cfg->base + USB_DEV_ADR_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_SMT_OFFSET);
	tmp &= ~USB_SOFMT_MASK;
	if (cfg->speed_idx == 2) {
		tmp |= 0x44C;
	} else {
		tmp |= 0x2710;
	}
	sys_write32(tmp, cfg->base + USB_DEV_SMT_OFFSET);

	/* 'MISGx': Mask Interrupts Source Group x
	 * 'ISGx' : Interrupts Source Group x (set is clear, read is status)
	 */

	/* enable IRQs in group 0 for setup */
	tmp = sys_read32(cfg->base + USB_DEV_MISG0_OFFSET);
	tmp &= ~(USB_MCX_SETUP_INT
		| USB_MCX_IN_INT
		| USB_MCX_OUT_INT);
	tmp |= USB_MCX_COMFAIL_INT
		| USB_MCX_COMABORT_INT
		/* this is COMEND */
		| (1 << 3);
	sys_write32(tmp, cfg->base + USB_DEV_MISG0_OFFSET);

	/* enable IRQs in group 1 (fifo interrupts) */
	sys_write32(0x0, cfg->base + USB_DEV_MISG1_OFFSET);

	/* enable some group 2 interrupts (usb rst tx rx sus res) */
	sys_write32(0xFFFFFF98, cfg->base + USB_DEV_MISG2_OFFSET);

	/* enable some group 3 interrupts (DMA completion interrupts?) */
	sys_write32(0xFFFFFFE0, cfg->base + USB_DEV_MISG3_OFFSET);

	/* enable group irqs */
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

	/* clear irqs group 2 */
	sys_write32(0x3ff, cfg->base + USB_DEV_ISG2_OFFSET);
	/* clear irqs group 3 */
	sys_write32(0xffffffff, cfg->base + USB_DEV_ISG3_OFFSET);

	/* resetting fifo and ep mapping went there */

	/* enable 'vdma' (virtual dma? video dma?) */
	tmp = sys_read32(cfg->base + USB_VDMA_CTRL_OFFSET);
	tmp |= USB_VDMA_EN;
	sys_write32(tmp, cfg->base + USB_VDMA_CTRL_OFFSET);

	/* force plug signal? */
	tmp = sys_read32(cfg->base + USB_PHY_TST_OFFSET);
	tmp &= ~USB_UNPLUG;
	sys_write32(tmp, cfg->base + USB_PHY_TST_OFFSET);

	/* 'enable global irq' */
	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp |= USB_GLINT_EN_HOV;
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	return 0;
}

static int udc_bflb_bl61x_disable(const struct device *dev)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	uint32_t tmp = 0;

	LOG_DBG("Enable device %p", dev);

	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp &= ~USB_GLINT_EN_HOV;
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	tmp = sys_read32(cfg->base + USB_PHY_TST_OFFSET);
	tmp |= USB_UNPLUG;
	sys_write32(tmp, cfg->base + USB_PHY_TST_OFFSET);

	return 0;
}

static void udc_bflb_bl61x_clock_init(const struct device *dev)
{
	uint32_t tmp = 0;

	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);
	tmp |= GLB_PU_USBPLL_MMDIV_MSK;
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);

	k_usleep(5);

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

static void udc_bflb_bl61x_phy_init(const struct device *dev)
{
	uint32_t tmp = 0;

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

	/* enable reset */
	tmp = sys_read32(PDS_BASE + PDS_USB_CTL_OFFSET);
	tmp &= ~PDS_REG_USB_SW_RST_N_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_CTL_OFFSET);

	k_usleep(1);

	/* unsuspend */
	tmp = sys_read32(PDS_BASE + PDS_USB_CTL_OFFSET);
	tmp |= PDS_REG_USB_EXT_SUSP_N_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_CTL_OFFSET);

	k_msleep(5);

	/* disable reset */
	tmp = sys_read32(PDS_BASE + PDS_USB_CTL_OFFSET);
	tmp |= PDS_REG_USB_SW_RST_N_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_CTL_OFFSET);

	k_msleep(5);
}

static int udc_bflb_bl61x_init(const struct device *dev)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;

	udc_bflb_bl61x_clock_init(dev);
	udc_bflb_bl61x_phy_init(dev);

	/* set endpoints and fifo mappings to disabled
	 * we have 5 total fifos (4 regular, one control)
	 * we have 5 bidir ep
	 * disabled is 0xf value
	 * see usb_v2_reg.h for format
	 */

	sys_write32(0xFFFFFFFF, cfg->base + USB_DEV_EPMAP0_OFFSET);
	sys_write32(0xFF, cfg->base + USB_DEV_EPMAP0_OFFSET);
	udc_bflb_bl61x_fifo_setep(dev, 15, 0, 0);
	udc_bflb_bl61x_fifo_setep(dev, 15, 1, 0);
	udc_bflb_bl61x_fifo_setep(dev, 15, 2, 0);
	udc_bflb_bl61x_fifo_setep(dev, 15, 3, 0);

	udc_bflb_bl61x_fifo_reset_ctrl(dev);
	udc_bflb_bl61x_fifo_reset(dev, 0);
	udc_bflb_bl61x_fifo_reset(dev, 1);
	udc_bflb_bl61x_fifo_reset(dev, 2);
	udc_bflb_bl61x_fifo_reset(dev, 3);

	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT,
				   USB_EP_TYPE_CONTROL, 64, 0)) {
		LOG_ERR("Failed to enable control endpoint");
		return -EIO;
	}

	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_IN,
				   USB_EP_TYPE_CONTROL, 64, 0)) {
		LOG_ERR("Failed to enable control endpoint");
		return -EIO;
	}

	cfg->irq_enable_func(dev);

	LOG_INF("Initialized");

	return 0;
}

/* Shut down the controller completely */
static int udc_bflb_bl61x_shutdown(const struct device *dev)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	uint32_t tmp = 0;

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

static int udc_bflb_bl61x_driver_preinit(const struct device *dev)
{
	const struct udc_bflb_bl61x_config *config = dev->config;
	struct udc_data *data = dev->data;
	struct udc_bflb_bl61x_data *priv = data->priv;
	uint16_t mps = 512;
	int err;

	k_mutex_init(&data->mutex);
	k_fifo_init(&priv->fifo);

	data->caps.rwup = true;
	data->caps.mps0 = UDC_MPS0_64;
	if (config->speed_idx == 2) {
		data->caps.hs = true;
		mps = 1024;
	}

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			config->ep_cfg_out[i].caps.control = 1;
			config->ep_cfg_out[i].caps.mps = 64;
		} else {
			config->ep_cfg_out[i].caps.bulk = 1;
			config->ep_cfg_out[i].caps.interrupt = 1;
			config->ep_cfg_out[i].caps.iso = 1;
			config->ep_cfg_out[i].caps.mps = mps;
		}

		config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		err = udc_register_ep(dev, &config->ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_in[i].caps.in = 1;
		if (i == 0) {
			config->ep_cfg_in[i].caps.control = 1;
			config->ep_cfg_in[i].caps.mps = 64;
		} else {
			config->ep_cfg_in[i].caps.bulk = 1;
			config->ep_cfg_in[i].caps.interrupt = 1;
			config->ep_cfg_in[i].caps.iso = 1;
			config->ep_cfg_in[i].caps.mps = mps;
		}

		config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		err = udc_register_ep(dev, &config->ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	LOG_INF("Device %p (max. speed %d)", dev, config->speed_idx);

	return 0;
}

static int udc_bflb_bl61x_lock(const struct device *dev)
{
	return udc_lock_internal(dev, K_FOREVER);
}

static int udc_bflb_bl61x_unlock(const struct device *dev)
{
	return udc_unlock_internal(dev);
}

static void udc_bflb_bl61x_isr(const struct device *dev)
{
	const struct udc_bflb_bl61x_config *cfg = dev->config;
	struct udc_bflb_bl61x_data *priv = udc_get_private(dev);
	uint32_t tmp = 0;
	uint32_t glb_intstatus;
	uint32_t dev_intstatus;
	uint32_t group_intstatus;
	uint8_t ep_idx;

	glb_intstatus = sys_read32(cfg->base + USB_GLB_ISR_OFFSET);

	if (glb_intstatus & USB_DEV_INT) {
		dev_intstatus = sys_read32(cfg->base + USB_DEV_IGR_OFFSET);
		if (dev_intstatus & USB_INT_G0) {
			group_intstatus = sys_read32(cfg->base + USB_DEV_ISG0_OFFSET);
			group_intstatus &=  ~sys_read32(cfg->base + USB_DEV_MISG0_OFFSET);

			if (group_intstatus & USB_CX_COMABT_INT) {
				//udc_submit_event(dev, UDC_EVT_ERROR, -1);
				LOG_ERR("Control command abort");
			}
			if (group_intstatus & USB_CX_SETUP_INT) {
				priv->ep_in[0] = false;
				udc_bflb_bl61x_ev_submit(dev, USB_CONTROL_EP_OUT, UBFBL61X_EVT_CTRL_SETUP_START);
			}
			if (group_intstatus & USB_CX_IN_INT) {
				priv->ep_in[0] = true;
				udc_bflb_bl61x_ev_submit(dev, USB_CONTROL_EP_OUT, UBFBL61X_EVT_CTRL_DIN_START);
			}
			if (group_intstatus & USB_CX_OUT_INT) {
				priv->ep_in[0] = false;
				udc_bflb_bl61x_ev_submit(dev, USB_CONTROL_EP_OUT, UBFBL61X_EVT_CTRL_DOUT_START);
			}
			if (group_intstatus & USB_CX_COMFAIL_INT) {
				//udc_submit_event(dev, UDC_EVT_ERROR, -1);
				LOG_ERR("Control command Fail");
			}
			/* clear isr */
			sys_write32(group_intstatus, cfg->base + USB_DEV_ISG0_OFFSET);
		}
		if (dev_intstatus & USB_INT_G1) {
			group_intstatus = sys_read32(cfg->base + USB_DEV_ISG1_OFFSET);
			group_intstatus &=  ~sys_read32(cfg->base + USB_DEV_MISG1_OFFSET);


			for (int fifo = 0; fifo < cfg->num_of_eps - 1; fifo++) {
				if (group_intstatus & (1 << 16 << (fifo))) {
					priv->ep_in[udc_bflb_bl61x_fifo_get_ep(fifo)] = true;
					udc_bflb_bl61x_ev_submit(dev, USB_EP_DIR_IN | udc_bflb_bl61x_fifo_get_ep(fifo), UBFBL61X_EVT_DIN_START);
				}

				if ((group_intstatus & (1 << ((fifo) * 2))) ||
				(group_intstatus & (1 << 1 << ((fifo) * 2)) {
					priv->ep_in[udc_bflb_bl61x_fifo_get_ep(fifo)] = false;
					udc_bflb_bl61x_ev_submit(dev, USB_EP_DIR_OUT | udc_bflb_bl61x_fifo_get_ep(fifo), UBFBL61X_EVT_DOUT_START);
				}
			}

			sys_write32(group_intstatus, cfg->base + USB_DEV_ISG1_OFFSET);
		}
		if (dev_intstatus & USB_INT_G2) {
			group_intstatus = sys_read32(cfg->base + USB_DEV_ISG2_OFFSET);
			group_intstatus &=  ~sys_read32(cfg->base + USB_DEV_MISG2_OFFSET);

			/* suspended */
			if (group_intstatus & USB_SUSP_INT) {
				sys_write32(USB_SUSP_INT, cfg->base + USB_DEV_ISG2_OFFSET);

				udc_bflb_bl61x_fifo_reset_ctrl(dev);
				udc_bflb_bl61x_fifo_reset(dev, 0);
				udc_bflb_bl61x_fifo_reset(dev, 1);
				udc_bflb_bl61x_fifo_reset(dev, 2);
				udc_bflb_bl61x_fifo_reset(dev, 3);

				udc_set_suspended(dev, true);
				udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
			}
			/* resumed */
			if (group_intstatus & USB_RESM_INT) {
				sys_write32(USB_RESM_INT, cfg->base + USB_DEV_ISG2_OFFSET);
				udc_set_suspended(dev, false);
				udc_submit_event(dev, UDC_EVT_RESUME, 0);
			}

			if (group_intstatus & USBRST_INT) {
				sys_write32(USBRST_INT, cfg->base + USB_DEV_ISG2_OFFSET);

				udc_bflb_bl61x_fifo_reset_ctrl(dev);
				udc_bflb_bl61x_fifo_reset(dev, 0);
				udc_bflb_bl61x_fifo_reset(dev, 1);
				udc_bflb_bl61x_fifo_reset(dev, 2);
				udc_bflb_bl61x_fifo_reset(dev, 3);

				udc_submit_event(dev, UDC_EVT_RESET, 0);
			}
		}
		if (dev_intstatus & USB_INT_G3) {
			group_intstatus = sys_read32(cfg->base + USB_DEV_ISG3_OFFSET);
			group_intstatus &=  ~sys_read32(cfg->base + USB_DEV_MISG3_OFFSET);
			/* clear isr */
			sys_write32(group_intstatus, cfg->base + USB_DEV_ISG3_OFFSET);

			if (group_intstatus & USB_VDMA_CMPLT_CXF) {
				if (priv->ep_in[0]) {
					udc_bflb_bl61x_ev_submit(dev, USB_CONTROL_EP_IN, UBFBL61X_EVT_CTRL_END);
				} else {
					udc_bflb_bl61x_ev_submit(dev, USB_CONTROL_EP_OUT, UBFBL61X_EVT_CTRL_END);
				}
			}

			for (uint8_t i = 0; i < cfg->num_of_eps - 1; i++) {
				if (group_intstatus & (1 << (i + 1))) {
					if (priv->ep_in[1 + i]) {
						udc_bflb_bl61x_ev_submit(dev, USB_EP_DIR_IN | udc_bflb_bl61x_fifo_get_ep(i), UBFBL61X_EVT_END);
					} else {
						udc_bflb_bl61x_ev_submit(dev, USB_EP_DIR_OUT | udc_bflb_bl61x_fifo_get_ep(i), UBFBL61X_EVT_END);
					}
				}
			}
		}
		if (dev_intstatus & USB_INT_G4) {
		}
	}
}

static const struct udc_api udc_bflb_bl61x_api = {
	.lock = udc_bflb_bl61x_lock,
	.unlock = udc_bflb_bl61x_unlock,
	.device_speed = udc_bflb_bl61x_device_speed,
	.init = udc_bflb_bl61x_init,
	.enable = udc_bflb_bl61x_enable,
	.disable = udc_bflb_bl61x_disable,
	.shutdown = udc_bflb_bl61x_shutdown,
	.set_address = udc_bflb_bl61x_set_address,
	.host_wakeup = udc_bflb_bl61x_host_wakeup,
	.ep_enable = udc_bflb_bl61x_ep_enable,
	.ep_disable = udc_bflb_bl61x_ep_disable,
	.ep_set_halt = udc_bflb_bl61x_ep_set_halt,
	.ep_clear_halt = udc_bflb_bl61x_ep_clear_halt,
	.ep_enqueue = udc_bflb_bl61x_ep_enqueue,
	.ep_dequeue = udc_bflb_bl61x_ep_dequeue,
};

#define DT_DRV_COMPAT bflb_udc_bl61x


#define UDC_BFLB_BL61X_DEVICE_DEFINE(n)						\
	static void udc_irq_enable_func##n(const struct device *dev)		\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n),					\
			    DT_INST_IRQ(n, priority),				\
			    udc_bflb_bl61x_isr,					\
			    DEVICE_DT_INST_GET(n), 0);				\
										\
		irq_enable(DT_INST_IRQN(n));					\
	}									\
										\
	static void udc_irq_disable_func##n(const struct device *dev)		\
	{									\
		irq_disable(DT_INST_IRQN(n));					\
	}									\
										\
	static struct udc_ep_config						\
		ep_cfg_out[DT_INST_PROP(n, num_bidir_endpoints)];		\
	static struct udc_ep_config						\
		ep_cfg_in[DT_INST_PROP(n, num_bidir_endpoints)];		\
										\
	static const struct udc_bflb_bl61x_config udc_bflb_bl61x_config_##n = {	\
		.base = DT_INST_REG_ADDR(n),					\
		.num_of_eps = DT_INST_PROP(n, num_bidir_endpoints),		\
		.ep_cfg_in = ep_cfg_out,					\
		.ep_cfg_out = ep_cfg_in,					\
		.speed_idx = DT_ENUM_IDX(DT_DRV_INST(n), maximum_speed),	\
		.irq_enable_func = udc_irq_enable_func##n,			\
		.irq_disable_func = udc_irq_disable_func##n,			\
	};									\
										\
	static struct udc_bflb_bl61x_data udc_priv_##n = {			\
		.work = Z_WORK_INITIALIZER(udc_bflb_bl61x_work_handler),	\
	};									\
										\
	static struct udc_data udc_data_##n = {					\
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),		\
		.priv = &udc_priv_##n,						\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, udc_bflb_bl61x_driver_preinit, NULL,		\
			      &udc_data_##n, &udc_bflb_bl61x_config_##n,	\
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			      &udc_bflb_bl61x_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_BFLB_BL61X_DEVICE_DEFINE)
