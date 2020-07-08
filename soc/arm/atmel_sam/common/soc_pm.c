/*
 * Copyright (c) 2020 Gerson Fernando Budke <nandojve@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Atmel SAM4L MCU family Power Management (PM) module
 * HAL driver.
 */

#include <soc.h>
#include <sys/__assert.h>
#include <sys/util.h>

static uint32_t bridge_peripheral_instances[] = {
	1,	/* CPU MASK Instances */
	10,	/* HSB MASK Instances */
	24,	/* PBA MASK Instances */
	7,	/* PBB MASK Instances */
	5,	/* PBC MASK Instances */
	6,	/* PBD MASK Instances */
};

void soc_pmc_peripheral_enable(uint32_t id)
{
	uint32_t bus_id = id >> 5;
	uint32_t per_idx = id & 0x1F;
	uint32_t mask;

	if (bus_id >= 6) {
		return;
	}

	if (per_idx >= bridge_peripheral_instances[bus_id]) {
		return;
	}

	mask		= *(&PM->CPUMASK + bus_id);
	mask		|= (1U << per_idx);
	PM->UNLOCK	= PM_UNLOCK_KEY(0xAAu) |
			  PM_UNLOCK_ADDR(((uint32_t)&PM->CPUMASK -
					  (uint32_t)PM) +
					  (4 * bus_id));
	*(&PM->CPUMASK + bus_id) = mask;
}

void soc_pmc_peripheral_disable(uint32_t id)
{
	uint32_t bus_id = id >> 5;
	uint32_t per_idx = id & 0x1F;
	uint32_t mask;

	if (bus_id >= 6) {
		return;
	}

	if (per_idx >= bridge_peripheral_instances[bus_id]) {
		return;
	}

	mask		= *(&PM->CPUMASK + bus_id);
	mask		&= ~(1U << per_idx);
	PM->UNLOCK	= PM_UNLOCK_KEY(0xAAu) |
			  PM_UNLOCK_ADDR(((uint32_t)&PM->CPUMASK -
					  (uint32_t)PM) +
					  (4 * bus_id));
	*(&PM->CPUMASK + bus_id) = mask;
}

uint32_t soc_pmc_peripheral_is_enabled(uint32_t id)
{
	uint32_t bus_id = id >> 5;
	uint32_t per_idx = id & 0x1F;
	uint32_t mask;

	if (bus_id >= 6) {
		return 0;
	}

	if (per_idx >= bridge_peripheral_instances[bus_id]) {
		return 0;
	}

	mask = *(&PM->CPUMASK + bus_id);

	return ((mask & (1U << per_idx)) > 0);
}
void soc_pm_enable_pba_divmask(uint32_t mask)
{
	uint32_t temp_mask;

	temp_mask	= PM->PBADIVMASK;
	temp_mask	|= mask;

	PM->UNLOCK	= PM_UNLOCK_KEY(0xAAu) |
			  PM_UNLOCK_ADDR((uint32_t)&PM->PBADIVMASK -
					 (uint32_t)PM);
	PM->PBADIVMASK = temp_mask;
}
