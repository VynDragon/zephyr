/*
 * Copyright (c) 2024 MASSDRIVER EI
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_wifi

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bflb_wifi, CONFIG_WIFI_LOG_LEVEL);

#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/conn_mgr/connectivity_wifi_mgmt.h>
#include <zephyr/device.h>
#include <zephyr/drivers/eeprom.h>
#include <soc.h>

#include <bitset.h>
#include <wifi_mgmr.h>
#include <wifi_mgmr_api.h>
#include <bl_wifi.h>
#include <wifi_mgmr_ext.h>

static struct net_if *bflb_wifi_iface;
static struct bflb_wifi_data bflb_wifi_data;

struct bflb_wifi_data {
#if defined(CONFIG_NET_STATISTICS_WIFI)
	struct net_stats_wifi stats;
#endif
	WIFI_MGMR_CONNECTION_STATUS_T state;
	uint8_t mac_addr[6];
};


static wifi_conf_t g_conf =
{
  .country_code = "FR",
};

void bflb_wifi_event_handler(int event, int value)
{
	return;
}

static int bflb_wifi_disconnect(const struct device *dev)
{
	return 0;
}

static int bflb_wifi_connect(const struct device *dev,
			    struct wifi_connect_req_params *params)
{
	return 0;
}

static int bflb_wifi_scan(const struct device *dev,
			   struct wifi_scan_params *params,
			   scan_result_cb_t cb)
{
	return 0;
};

static int bflb_wifi_ap_enable(const struct device *dev,
			 struct wifi_connect_req_params *params)
{
	return 0;
};

static int bflb_wifi_ap_disable(const struct device *dev)
{
	return 0;
};

static int bflb_wifi_status(const struct device *dev, struct wifi_iface_status *status)
{
	return 0;
}

static int bflb_wifi_send(const struct device *dev, struct net_pkt *pkt)
{
	return 0;
}

static void bflb_wifi_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct bflb_wifi_data *data = dev->data;
	struct ethernet_context *eth_ctx = net_if_l2_data(iface);
	int ret = 0;
	const struct device *efuse = DEVICE_DT_GET(DT_NODELABEL(efuse));
	uint32_t mac_low = 0, mac_high = 0;

	if (eeprom_read(efuse, 0x14, &mac_low, 4) != 0) {
		LOG_ERR("failed to read part of MAC address");
	}
	if (eeprom_read(efuse, 0x18, &mac_high, 4) != 0) {
		LOG_ERR("failed to read part of MAC address");
	}

	eth_ctx->eth_if_type = L2_ETH_IF_TYPE_WIFI;
	bflb_wifi_iface = iface;
	data->state = WIFI_MGMR_CONNECTION_STATUS_IDLE;

	data->mac_addr[0] = mac_low & 0xFF;
	data->mac_addr[1] = (mac_low & 0xFF00) >> 8;
	data->mac_addr[2] = (mac_low & 0xFF0000) >> 16;
	data->mac_addr[3] = (mac_low & 0xFF000000) >> 24;

	data->mac_addr[4] = mac_high & 0xFF;
	data->mac_addr[5] = (mac_high & 0xFF00) >> 8;
	/* Assign link local address. */
	net_if_set_link_addr(iface, data->mac_addr, 6, NET_LINK_ETHERNET);

	ethernet_init(iface);
	net_if_carrier_off(iface);

	bl_pm_init();
	wifi_main_init();
	ipc_emb_notify();
	wifi_mgmr_drv_init(&g_conf);
	wifi_mgmr_tsk_init();
}

#if defined(CONFIG_NET_STATISTICS_WIFI)
static int bflb_wifi_stats(const struct device *dev, struct net_stats_wifi *stats)
{
	return 0;
}
#endif

static int bflb_wifi_dev_init(const struct device *dev)
{
	return 0;
}

static const struct wifi_mgmt_ops bflb_wifi_mgmt = {
	.scan		   = bflb_wifi_scan,
	.connect	   = bflb_wifi_connect,
	.disconnect	   = bflb_wifi_disconnect,
	.ap_enable	   = bflb_wifi_ap_enable,
	.ap_disable	   = bflb_wifi_ap_disable,
	.iface_status	   = bflb_wifi_status,
#if defined(CONFIG_NET_STATISTICS_WIFI)
	.get_stats	   = bflb_wifi_stats,
#endif
};

static const struct net_wifi_mgmt_offload bflb_api = {
	.wifi_iface.iface_api.init		= bflb_wifi_init,
	.wifi_iface.send			= bflb_wifi_send,
	.wifi_mgmt_api				= &bflb_wifi_mgmt,
};

NET_DEVICE_DT_INST_DEFINE(0,
		bflb_wifi_dev_init, NULL,
		&bflb_wifi_data, NULL, CONFIG_WIFI_INIT_PRIORITY,
		&bflb_api, ETHERNET_L2,
		NET_L2_GET_CTX_TYPE(ETHERNET_L2), NET_ETH_MTU);

CONNECTIVITY_WIFI_MGMT_BIND(Z_DEVICE_DT_DEV_ID(DT_DRV_INST(0)));
