/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware port for the Bouffalo Lab BLE controller binary blob on BL61x.
 * Overrides the weak implementations in the precompiled library.
 */

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/otp.h>
#include <stdint.h>
#include <string.h>

#include <bflb_soc.h>
#include <glb_reg.h>
#include <pds_reg.h>

/* BLE IRQ number: IRQ_NUM_BASE(16) + 56 = 72 */
#define BLE_IRQN 72

/* DM IRQ number: IRQ_NUM_BASE(16) + 45 = 61 */
#define DM_IRQN 61

/* Bits per software reset register bank (CFG0/CFG1/CFG2) */
#define SWRST_BITS_PER_REG 32U

/*
 * btblecontroller_ble_irq_init — Register and enable the BLE interrupt
 */
void btblecontroller_ble_irq_init(void *handler)
{
	irq_connect_dynamic(BLE_IRQN, 0, (void (*)(const void *))handler, NULL, 0);
	irq_enable(BLE_IRQN);
}

void btblecontroller_dm_irq_init(void *handler)
{
	irq_connect_dynamic(DM_IRQN, 0, (void (*)(const void *))handler, NULL, 0);
	irq_enable(DM_IRQN);
}

void btblecontroller_ble_irq_enable(uint8_t enable)
{
	if (enable) {
		irq_enable(BLE_IRQN);
	} else {
		irq_disable(BLE_IRQN);
	}
}

void btblecontroller_dm_irq_enable(uint8_t enable)
{
	if (enable) {
		irq_enable(DM_IRQN);
	} else {
		irq_disable(DM_IRQN);
	}
}

/*
 * btblecontroller_efuse_read_mac — Read BLE MAC address from eFuse
 */
int btblecontroller_efuse_read_mac(uint8_t mac[6])
{
	uint8_t id[8] = {0};
	const size_t mac_len = 6U;

	if (hwinfo_get_device_id(id, mac_len) != (ssize_t)mac_len) {
		memset(mac, 0, mac_len);
		return -1;
	}
	memcpy(mac, id, mac_len);
	return 0;
}

void btblecontroller_sys_reset(void)
{
	sys_reboot(0);
}

uint64_t bflb_mtimer_get_time_us(void)
{
	return k_cyc_to_us_floor64(k_cycle_get_64());
}

uint64_t btblecontroller_mtimer_get_time_us(void)
{
	return k_cyc_to_us_floor64(k_cycle_get_64());
}

void btblecontroller_puts(const char *str)
{
	ARG_UNUSED(str);
}

int btblecontroller_printf(const char *fmt, ...)
{
	ARG_UNUSED(fmt);
	return 0;
}

int mfg_media_read_macaddr_with_lock(uint8_t mac[6], uint8_t reload)
{
	ARG_UNUSED(reload);
	return btblecontroller_efuse_read_mac(mac);
}
