/*
 * Copyright (c) 2021 Gerson Fernando Budke <nandojve@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * Bouffalo Lab SoC specific helpers for pinctrl driver
 */

#ifndef ZEPHYR_SOC_RISCV_BFLB_COMMON_PINCTRL_SOC_H_
#define ZEPHYR_SOC_RISCV_BFLB_COMMON_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>
#include <zephyr/dt-bindings/pinctrl/bflb-pinctrl.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */

typedef struct pinctrl_soc_pin {
	uint8_t		fun;
	uint8_t		pin;
	uint32_t	cfg;
} pinctrl_soc_pin_t;


/**
 * @brief Utility macro to initialize flags field in #pinctrl_soc_pin_t.
 *
 * @param node_id Node identifier.
 */
#define Z_PINCTRL_BFLB_CFG_INIT(node_id)						\
	(										\
	 (BFLB_CFGFLAG_PULL_UP * DT_PROP_OR(node_id, bias_pull_up, 0)) |		\
	 (BFLB_CFGFLAG_PULL_DOWN * DT_PROP_OR(node_id, bias_pull_down, 0)) |		\
	 (BFLB_CFGFLAG_SMT * DT_PROP_OR(node_id, input_schmitt_enable, 0)) |		\
	 ((DT_PROP_OR(node_id, drive_strength, 0) & 0x3) << BFLB_CFGFLAG_DRIVE_SHIFT) |	\
	 (BFLB_CFGFLAG_GPIOMODE_INPUT * DT_PROP_OR(node_id, input-enable, 0)) |		\
	 (BFLB_CFGFLAG_GPIOMODE_OUTPUT * DT_PROP_OR(node_id, output-enable, 0))		\
	)

/**
 * @brief Utility macro to initialize each pin.
 *
 * @param node_id Node identifier.
 * @param prop Property name.
 * @param idx Property entry index.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)				\
	{ .fun   = (DT_PROP_BY_IDX(node_id, prop, idx) & 0x1F),			\
	  .pin   = DT_PROP_BY_IDX(node_id, prop, idx) >> 5,			\
	  .cfg   = Z_PINCTRL_BFLB_CFG_INIT(node_id),				\
	},

/**
 * @brief Utility macro to initialize state pins contained in a given property.
 *
 * @param node_id Node identifier.
 * @param prop Property name describing state pins.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)			      	\
	{DT_FOREACH_CHILD_VARGS(DT_PROP_BY_IDX(node_id, prop, 0),	      	\
				DT_FOREACH_PROP_ELEM, pinmuxes,	       		\
				Z_PINCTRL_STATE_PIN_INIT)}

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_RISCV_BFLB_COMMON_PINCTRL_SOC_H_ */
