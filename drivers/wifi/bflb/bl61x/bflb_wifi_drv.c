/*
 * Copyright (c) 2024 MASSDRIVER EI
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_bl61x_wifi

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bflb_bl61x_wifi, CONFIG_WIFI_LOG_LEVEL);

#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/conn_mgr/connectivity_wifi_mgmt.h>
#include <zephyr/device.h>
#include <zephyr/drivers/eeprom.h>
#include <soc.h>

#include <wifi_mgmr.h>
#include <wifi_mgmr_ext.h>
#include <wl_api.h>


#include "bflb_wifi_rf.h"

static struct wl_cfg_t bl61x_rfcfg = {
	.status = WL_API_STATUS_OK,
	.mode = WL_API_MODE_WLAN,
	.en_param_load = 1,
	.en_full_cal = 1,
	.param = bl61x_rfparams,
	.log_level = WL_LOG_LEVEL_DEBUG,
#ifdef CONFIG_SOC_BL616C50Q2I
	.device_info = WL_DEVICE_QFN40,
#elif CONFIG_SOC_BL616S50Q2I
	.device_info = WL_DEVICE_QFN40,
#elif CONFIG_SOC_BL618M05Q2I
	.device_info = WL_DEVICE_QFN56,
#endif
};

static rtos_task_handle	fw_task;
static struct net_if *bflb_wifi_iface;
static struct bflb_wifi_data bflb_wifi_data;

struct bflb_wifi_data {
#if defined(CONFIG_NET_STATISTICS_WIFI)
	struct net_stats_wifi stats;
#endif
	uint8_t mac_addr[6];
	scan_result_cb_t	scan_cb;
};


wifi_mgmr_t wifiMgmr =
{
};

wifi_conf_t cnf = { .country_code = "FR", .channel_nums = 14 };

static uint32_t system_get_xtal(void)
{
	uint32_t tmpVal;
	tmpVal = sys_read32(HBN_BASE + HBN_RSV3_OFFSET);
	tmpVal &= 0xF;

	switch (tmpVal) {
	case 0:
		return 0;
	case 1:
		return 24 * 1000 * 1000;
	case 2:
		return 32 * 1000 * 1000;
	case 3:
		return 38.4 * 1000 * 1000;
	case 4:
		return 40 * 1000 * 1000;
	case 5:
		return 26 * 1000 * 1000;
	case 6:
		return 32 * 1000 * 1000;
	default:
		return 0;
	}
}

void bflb_wifi_event_handler(int event, int value)
{
	printf("received event: %d value: %d\n", event, value);

	if (value == CODE_WIFI_ON_SCAN_DONE) {
		//wifi_mgmr_sta_scanlist();
	}
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
	struct bflb_wifi_data	*data = dev->data;

	/*wifi_mgmr_scan_params_t config = {0};
	wifi_mgmr_sta_scan(&config);
	data->scan_cb = cb;*/
	wifi_mgmr_scan_item_t scan_items[WIFI_MGMR_SCAN_ITEMS_MAX];
	wifi_mgmr_scan_beacon_save(wifiMgmr.scan_items);

	for (int i = 0; i < 50; i++) {
		printf("SSID: %s, rssi: %d\n", wifiMgmr.scan_items[i].ssid, wifiMgmr.scan_items[i].rssi);
	}
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

static void bflb_wifi_init_rf_get_capcode(uint8_t* capcode_in, uint8_t* capcode_out)
{
	uint32_t capcode = 0;

	capcode = sys_read32(AON_BASE + AON_XTAL_CFG_OFFSET);
	capcode &= AON_XTAL_CAPCODE_IN_AON_MSK;
	capcode = capcode >> AON_XTAL_CAPCODE_IN_AON_POS;
	*capcode_in = *capcode_out = capcode;
}

static void bflb_wifi_init_rf_set_capcode(uint8_t capcode_in, uint8_t capcode_out)
{
	uint32_t capcode = 0;

	capcode = sys_read32(AON_BASE + AON_XTAL_CFG_OFFSET);
	capcode &= AON_XTAL_CAPCODE_IN_AON_UMSK;
	capcode &= AON_XTAL_CAPCODE_OUT_AON_UMSK;
	capcode |= capcode_in << AON_XTAL_CAPCODE_IN_AON_POS;
	capcode |= capcode_out << AON_XTAL_CAPCODE_IN_AON_POS;
	sys_write32(capcode, AON_BASE + AON_XTAL_CFG_OFFSET);
}

int8_t bflb_wifi_init_rf_rfparam_load(struct wl_param_t *param)
{
	param->tcal = bl61x_tcal;
	param->pwrtarget = bl61x_pwrtarget;
	param->pwrcal = bl61x_pwrcal;
	param->tcap = bl61x_tcap;
	return 0;
}

static int bflb_wifi_init_rf_blob(void)
{
	int ret = 0;
	struct wl_cfg_t* wl_cfg_inst;

	wl_cfg_inst = wl_cfg_get((uint8_t*)WL_API_RMEM_ADDR);

	wl_cfg_inst->capcode_get = bflb_wifi_init_rf_get_capcode;
	wl_cfg_inst->capcode_set = bflb_wifi_init_rf_set_capcode;
	wl_cfg_inst->param_load = bflb_wifi_init_rf_rfparam_load;
	wl_cfg_inst->log_printf = printf;
	wl_cfg_inst->param.xtalfreq_hz = system_get_xtal();

	ret = wl_init();
	return ret;
}

static void bflb_wifi_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct bflb_wifi_data *data = dev->data;
	struct ethernet_context *eth_ctx = net_if_l2_data(iface);
	int ret = 0;
	const struct device *efuse = DEVICE_DT_GET(DT_NODELABEL(efuse));
	uint32_t mac_low = 0, mac_high = 0;
	uint32_t tmpVal = 0;

	if (eeprom_read(efuse, 0x14, &mac_low, 4) != 0) {
		LOG_ERR("failed to read part of MAC address");
	}
	if (eeprom_read(efuse, 0x18, &mac_high, 4) != 0) {
		LOG_ERR("failed to read part of MAC address");
	}

	eth_ctx->eth_if_type = L2_ETH_IF_TYPE_WIFI;
	bflb_wifi_iface = iface;

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

	if (bflb_wifi_init_rf_blob() != 0)
	{
		LOG_ERR("Failed to init rf");
	}

	/* reset wifi peripheral */
	tmpVal = sys_read32(GLB_BASE + GLB_SWRST_CFG0_OFFSET);
	tmpVal &= GLB_SWRST_S20_UMSK;
	sys_write32(tmpVal, GLB_BASE + GLB_SWRST_CFG0_OFFSET);
	__asm__ volatile (".rept 4 ; nop ; .endr");
	tmpVal |= GLB_SWRST_S20_MSK;
	sys_write32(tmpVal, GLB_BASE + GLB_SWRST_CFG0_OFFSET);
	__asm__ volatile (".rept 4 ; nop ; .endr");
	tmpVal &= GLB_SWRST_S20_UMSK;
	sys_write32(tmpVal, GLB_BASE + GLB_SWRST_CFG0_OFFSET);

	/* setup wifi blob irq */
	extern void interrupt0_handler(void);
	IRQ_CONNECT(70, 10, interrupt0_handler, 0, 0);
	irq_enable(70);

	/* start wifi thread */
	//rtos_task_create(wifi_main, "fw", 0, 1024, NULL, 0, &fw_task);
	//rtos_task_create(wifi_mgmr_init, "mgmr", 0, 1024, &cnf, 0, &fw_task);
	wifi_mgmr_init(&cnf);
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
