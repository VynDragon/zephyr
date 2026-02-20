/*
 * Copyright (c) 2026 William Markezana
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>

#include <bflb_soc.h>
#include <glb_reg.h>
#include <hbn_reg.h>
#include <zephyr/dt-bindings/pinctrl/bflb-common-pinctrl.h>

#if defined(CONFIG_SOC_SERIES_BL70XL)
#include <zephyr/dt-bindings/pinctrl/bl70xl-pinctrl.h>
#else
#error "Unsupported Platform"
#endif

/* GPIO function select values (from vendor SDK bl702l_gpio.h) */
#define GLB_GPIO_FUNC_ANALOG       10
#define GLB_GPIO_FUNC_SWGPIO       11
#define GLB_GPIO_FUNC_KEY_SCAN_IN  21
#define GLB_GPIO_FUNC_KEY_SCAN_DRV 22

/* BL70XL has 4 UART signal routing slots (vs 8 on BL702) */
#define BL70XL_UART_SIG_SLOTS   4
#define BL70XL_UART_SIG_DISABLE 0x0F

/* XTAL32K pins */
#define BL70XL_XTAL32K_PIN_XI 30
#define BL70XL_XTAL32K_PIN_XO 31

/* Per-slot mask for UART signal selection (each slot is GLB_UART_SIG_0_SEL_LEN bits) */
#define UART_SIG_SLOT_MSK (GLB_UART_SIG_0_SEL_MSK >> GLB_UART_SIG_0_SEL_POS)

void pinctrl_bflb_configure_uart(uint8_t pin, uint8_t uart_func)
{
	uint32_t regval;
	uint8_t sig;
	uint8_t sig_pos;

	regval = sys_read32(GLB_BASE + GLB_UART_SIG_SEL_0_OFFSET);

	sig = pin % BL70XL_UART_SIG_SLOTS;
	sig_pos = sig * GLB_UART_SIG_0_SEL_LEN;

	regval &= (~(UART_SIG_SLOT_MSK << sig_pos));
	regval |= (uart_func << sig_pos);

	for (uint8_t i = 0; i < BL70XL_UART_SIG_SLOTS; i++) {
		/* reset other sigs which are the same with uart_func */
		sig_pos = i * GLB_UART_SIG_0_SEL_LEN;
		if (((regval & (UART_SIG_SLOT_MSK << sig_pos)) == (uart_func << sig_pos)) &&
		    (i != sig) && (uart_func != BL70XL_UART_SIG_DISABLE)) {
			regval &= (~(UART_SIG_SLOT_MSK << sig_pos));
			regval |= (BL70XL_UART_SIG_DISABLE << sig_pos);
		}
	}

	sys_write32(regval, GLB_BASE + GLB_UART_SIG_SEL_0_OFFSET);
}

void pinctrl_bflb_init_pin(pinctrl_soc_pin_t pin)
{
	uint8_t drive;
	uint8_t function;
	uint16_t mode;
	uint32_t regval;
	uint32_t tmp;
	uint8_t real_pin;
	uint8_t is_odd;
	uint32_t cfg = 0;
	uint32_t cfg_address;
	uint8_t pull_up;
	uint8_t pull_down;

	real_pin = BFLB_PINMUX_GET_PIN(pin);
	function = BFLB_PINMUX_GET_FUN(pin);
	mode = BFLB_PINMUX_GET_MODE(pin);
	drive = BFLB_PINMUX_GET_DRIVER_STRENGTH(pin);

	/* disable muxed to be xtal32k */
	if (real_pin == BL70XL_XTAL32K_PIN_XI || real_pin == BL70XL_XTAL32K_PIN_XO) {
		tmp = sys_read32(HBN_BASE + HBN_PAD_CTRL_0_OFFSET);
		tmp &= ~(1 << (real_pin - 5));
		sys_write32(tmp, HBN_BASE + HBN_PAD_CTRL_0_OFFSET);
	}

	/* Disable output anyway */
	regval = sys_read32(GLB_BASE + GLB_GPIO_CFGCTL34_OFFSET + ((real_pin >> 5) << 2));
	regval &= ~(1 << (real_pin & 0x1f));
	sys_write32(regval, GLB_BASE + GLB_GPIO_CFGCTL34_OFFSET + ((real_pin >> 5) << 2));

	is_odd = real_pin & 1;

	cfg_address = GLB_BASE + GLB_GPIO_CFGCTL0_OFFSET + (real_pin / 2 * 4);
	cfg = sys_read32(cfg_address);
	cfg &= ~(0xffff << (16 * is_odd));

	regval = sys_read32(GLB_BASE + GLB_GPIO_CFGCTL34_OFFSET + ((real_pin >> 5) << 2));

	if (mode == BFLB_PINMUX_MODE_analog) {
		regval &= ~(1 << (real_pin & 0x1f));
		function = GLB_GPIO_FUNC_ANALOG;
	} else if (mode == BFLB_PINMUX_MODE_periph) {
		cfg |= (1 << (is_odd * 16 + GLB_REG_GPIO_0_IE_POS));
		regval &= ~(1 << (real_pin & 0x1f));
		/* BL702L: key scan drive function needs IE disabled */
		if (function == GLB_GPIO_FUNC_KEY_SCAN_DRV) {
			cfg &= ~(1 << (is_odd * 16 + GLB_REG_GPIO_0_IE_POS));
		}
	} else {
		function = GLB_GPIO_FUNC_SWGPIO;

		if (mode == BFLB_PINMUX_MODE_input) {
			cfg |= (1 << (is_odd * 16 + GLB_REG_GPIO_0_IE_POS));
		}

		if (mode == BFLB_PINMUX_MODE_output) {
			regval |= (1 << (real_pin & 0x1f));
		}
	}

	sys_write32(regval, GLB_BASE + GLB_GPIO_CFGCTL34_OFFSET + ((real_pin >> 5) << 2));

	pull_up = BFLB_PINMUX_GET_PULL_UP(pin);
	pull_down = BFLB_PINMUX_GET_PULL_DOWN(pin);

	if (pull_up != 0) {
		cfg |= (1 << (is_odd * 16 + GLB_REG_GPIO_0_PU_POS));
	} else if (pull_down != 0) {
		cfg |= (1 << (is_odd * 16 + GLB_REG_GPIO_0_PD_POS));
	}

	if (BFLB_PINMUX_GET_SMT(pin) != 0) {
		cfg |= (1 << (is_odd * 16 + GLB_REG_GPIO_0_SMT_POS));
	}

	cfg |= (drive << (is_odd * 16 + GLB_REG_GPIO_0_DRV_POS));
	cfg |= (function << (is_odd * 16 + GLB_REG_GPIO_0_FUNC_SEL_POS));

	/* BL702L: configure output mode to set/clr mode (not for key scan functions) */
	if ((function != GLB_GPIO_FUNC_KEY_SCAN_DRV) && (function != GLB_GPIO_FUNC_KEY_SCAN_IN)) {
		cfg |= (1 << (is_odd * 16 + GLB_REG_GPIO_0_MODE_POS));
	}

	sys_write32(cfg, cfg_address);
}
