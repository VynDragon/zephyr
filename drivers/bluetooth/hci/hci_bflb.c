/*
 * Copyright (c) 2024 MASSDRIVER EI
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/hci.h>

#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/drivers/bluetooth.h>

#include <hci_onchip.h>
#include <ble_lib_api.h>

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_hci_driver_bflb);

#define DT_DRV_COMPAT bflb_bt_hci

#define HCI_BT_BFLB_TIMEOUT K_MSEC(2000)

struct bt_bflb_data {
	bt_hci_recv_t recv;
};


static int bt_bflb_send(const struct device *dev, struct net_buf *buf)
{
	int err = 0;
	uint8_t pkt_indicator;

	LOG_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf), buf->len);

	return err;
}

static int bt_bflb_ble_init(void)
{
	int ret;
	ble_controller_init(5);
	blecontroller_main();

	return 0;
}

static int bt_bflb_ble_deinit(void)
{

	ble_controller_deinit();

	return 0;
}

static int bt_bflb_open(const struct device *dev, bt_hci_recv_t recv)
{
	struct bt_bflb_data *hci = dev->data;
	int err;

	err = bt_bflb_ble_init();
	if (err) {
		return err;
	}

	hci->recv = recv;

	LOG_DBG("BFLB BT started");

	return 0;
}

static int bt_bflb_close(const struct device *dev)
{
	struct bt_bflb_data *hci = dev->data;
	int err;

	err = bt_bflb_ble_deinit();
	if (err) {
		return err;
	}

	hci->recv = NULL;

	LOG_DBG("BFLB BT stopped");

	return 0;
}

static const struct bt_hci_driver_api drv = {
	.open           = bt_bflb_open,
	.send           = bt_bflb_send,
	.close          = bt_bflb_close,
};

#define BT_BFLB_DEVICE_INIT(inst) \
	static struct bt_bflb_data bt_bflb_data_##inst = { \
	}; \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &bt_bflb_data_##inst, NULL, \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &drv)

/* Only one instance supported */
BT_BFLB_DEVICE_INIT(0)
