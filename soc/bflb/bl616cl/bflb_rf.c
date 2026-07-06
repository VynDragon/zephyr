/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>

#include <bflb_soc.h>
#include <zephyr/drivers/otp.h>
#include <wl_api.h>
#include <aon_reg.h>

LOG_MODULE_REGISTER(bflb_rf, LOG_LEVEL_DBG);

#define XTAL_FREQ DT_PROP(DT_NODELABEL(clk_crystal), clock_frequency)

#define AON_XTAL_CFG_ADDR	(AON_BASE + AON_XTAL_CORE0_OFFSET)
#define CAPCODE_IN_POS		AON_XTAL_CAPCODE_IN_AON_POS
#define CAPCODE_OUT_POS		AON_XTAL_CAPCODE_OUT_AON_POS
#define CAPCODE_MSK		0xffU

#define BFLB_EFUSE_DEFAULT_TRIM	0x80U

/* Retention memory for PHY/RF driver calibration data */
static uint8_t wl_rmem_buf[CONFIG_BFLB_BL616CL_WL_RMEM_SIZE] __aligned(4);

static void capcode_get(uint8_t *capcode_in, uint8_t *capcode_out)
{
	uint32_t val = sys_read32(AON_XTAL_CFG_ADDR);

	*capcode_in = (val >> CAPCODE_IN_POS) & CAPCODE_MSK;
	*capcode_out = (val >> CAPCODE_OUT_POS) & CAPCODE_MSK;
}

static void capcode_set(uint8_t capcode_in, uint8_t capcode_out)
{
	uint32_t val = sys_read32(AON_XTAL_CFG_ADDR);

	val &= ~((CAPCODE_MSK << CAPCODE_IN_POS) | (CAPCODE_MSK << CAPCODE_OUT_POS));
	val |= ((uint32_t)(capcode_in & CAPCODE_MSK) << CAPCODE_IN_POS) |
	       ((uint32_t)(capcode_out & CAPCODE_MSK) << CAPCODE_OUT_POS);
	sys_write32(val, AON_XTAL_CFG_ADDR);
}

/*
 * bflb_rf_init — Initialize PHY/RF hardware via wl_init().
 *
 * Safe to call from both BLE and WiFi init paths: the static guard
 * ensures wl_init() runs exactly once.
 *
 * Mode is WL_API_MODE_ALL when WiFi is enabled (covers both WLAN + BLE),
 * otherwise WL_API_MODE_BZ for BLE-only.
 */
int bflb_rf_init(void)
{
	static bool initialized;
	struct wl_cfg_t *cfg;
	int ret;

	if (initialized) {
		return 0;
	}

	__ASSERT(wl_rmem_size_get() <= sizeof(wl_rmem_buf),
		 "WL rmem buffer too small: need %u, have %u", wl_rmem_size_get(),
		 (unsigned int)sizeof(wl_rmem_buf));

	cfg = wl_cfg_get(wl_rmem_buf);
	if (cfg == NULL) {
		LOG_ERR("wl_cfg_get failed");
		return -ENOMEM;
	}

	cfg->mode = WL_API_MODE_BZ;

	cfg->en_param_load = 0;
	cfg->device_info = 0;
	cfg->en_full_cal = 1;
	cfg->capcode_get = capcode_get;
	cfg->capcode_set = capcode_set;
	cfg->param_load = NULL;
	cfg->log_level = WL_LOG_LEVEL_NONE;
	cfg->log_printf = NULL;
	cfg->param.xtalfreq_hz = XTAL_FREQ;

	/* Default eFuse trim values (SDK falls back to 0x80 when empty) */
	cfg->param.ef.icx_code = 0x80;
	cfg->param.ef.iptat_code = 0x80;
	cfg->param.ef.Temperature_MP = 35;

	ret = wl_init();
	if (ret != 0) {
		LOG_ERR("wl_init failed: %d", ret);
		return -EIO;
	}

	initialized = true;
	LOG_INF("RF calibration complete");
	return 0;
}

/*
 * arch_delay_ms/us — Busy-wait delay functions called by PHY/RF and
 * BLE controller blobs.
 */
void arch_delay_ms(uint32_t ms)
{
	k_busy_wait(ms * 1000U);
}

void arch_delay_us(uint32_t us)
{
	k_busy_wait(us);
}
