/*
 * Copyright (c) 2024 MASSDRIVER EI
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <wl_api.h>

/* data from BFLB SDK bsp/board/bl616dk/config/bl_factory_params_IoTKitA_auto.dts
 * and organized according to rftlv adapter
 */

static const struct wl_param_tcal_t bl61x_tcal = {
	.en_tcal = 0,
	.linear_or_follow = 1,
	.Tchannels = {2412, 2427, 2442, 2457, 2472},
	.Tchannel_os = {180, 168, 163, 160, 157},
	.Tchannel_os_low = {199, 186, 170, 165, 160},
	.Troom_os = 255,
};

static const struct wl_param_pwrtarget_t bl61x_pwrtarget = {
	.pwr_11b = {20, 20, 20, 20},
	.pwr_11g = {18, 18, 18, 18, 18, 18, 16, 16},
	.pwr_11n_ht20  = {18, 18, 18, 18, 18, 16, 15, 15},
	.pwr_11n_ht40  = {18, 18, 18, 18, 18, 16, 15, 14},
	.pwr_11ac_vht20 = {18, 18, 18, 18, 18, 16, 15, 15, 15, 14},
	.pwr_11ac_vht40 = {18, 18, 18, 18, 18, 16, 15, 14, 14, 13},
	.pwr_11ac_vht80 = {18, 18, 18, 18, 18, 15, 14, 13, 13, 12},
	.pwr_11ax_he20  = {18, 18, 18, 18, 18, 16, 15, 15, 15, 14, 13, 13},
	.pwr_11ax_he40  = {18, 18, 18, 18, 18, 16, 15, 14, 14, 13, 12, 12},
	.pwr_11ax_he80  = {18, 18, 18, 18, 18, 15, 14, 13, 13, 12, 11, 11},
	.pwr_11ax_he160 = {18, 18, 18, 18, 18, 15, 14, 13, 12, 11, 10, 10},
	.pwr_ble = 13,
	.pwr_bt = {10, 8, 8},
	.pwr_zigbee = 13
};

static const struct wl_param_pwrcal_t bl61x_pwrcal = {
	.channel_pwrcomp_wlan = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16},
	.channel_lp_pwrcomp_wlan = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16},
	.Temperature_MP = 0,
	.channel_pwrcomp_bz = {16, 16, 16, 16, 16},
};

static const struct wl_param_pwrlimit_t bl61x_pwrlimit[NUM_WLAN_CHANNELS];

static const struct wl_efuse_t bl61x_ef;

static const struct wl_param_tcap_t bl61x_tcap = {
	.en_tcap = 0,
	.tcap_tsen = {253, 252, 20, 39, 39, 40, 41, 42, 43, 44},
	.tcap_cap = {28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38},
};

static const struct wl_param_t bl61x_rfparams = {
	.tcal = bl61x_tcal,
	.pwrtarget = bl61x_pwrtarget,
	.pwrcal = bl61x_pwrcal,
	.tcap = bl61x_tcap
};

