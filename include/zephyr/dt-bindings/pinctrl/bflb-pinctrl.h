/*
 * Copyright (c) 2021 Gerson Fernando Budke <nandojve@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_BFLB_PINCTRL_H_
#define ZEPHYR_BFLB_PINCTRL_H_
/**
 * @brief FUN configuration bitfield
 *
 * The Zephyr version of Bouffalo Lab function field encodes
 * function from a function name to a internal config bit array
 */

#define BFLB_FUN_clk_out		0x00
#define BFLB_FUN_bt_coexist		0x01
#define BFLB_FUN_flash_psram		0x02
#define BFLB_FUN_qspi			0x02
#define BFLB_FUN_i2s			0x03
#define BFLB_FUN_spi			0x04
#define BFLB_FUN_i2c			0x06
#define BFLB_FUN_uart			0x07
#define BFLB_FUN_pwm			0x08
#define BFLB_FUN_cam			0x09
#define BFLB_FUN_analog			0x0a
#define BFLB_FUN_gpio			0x0b
#define BFLB_FUN_rf_test		0x0c
#define BFLB_FUN_scan			0x0d
#define BFLB_FUN_jtag			0x0e
#define BFLB_FUN_debug			0x0f
#define BFLB_FUN_external_pa		0x10
#define BFLB_FUN_usb_transceiver	0x11
#define BFLB_FUN_usb_controller		0x12
#define BFLB_FUN_ether_mac		0x13
#define BFLB_FUN_emca			0x13
#define BFLB_FUN_qdec			0x14
#define BFLB_FUN_key_scan_in		0x15
#define BFLB_FUN_key_scan_drive		0x16
#define BFLB_FUN_key_scan_col		0x16
#define BFLB_FUN_cam_misc		0x17


#define BFLB_CFGFLAG_GPIOMODE_INPUT		0x1
#define BFLB_CFGFLAG_GPIOMODE_OUTPUT		0x2
#define BFLB_CFGFLAG_GPIOMODE_ANALOG		0x4
#define BFLB_CFGFLAG_GPIOMODE_ALTERNATE		0x8

#define BFLB_CFGFLAG_PULL_UP			0x10
#define BFLB_CFGFLAG_PULL_DOWN			0x20
#define BFLB_CFGFLAG_SMT			0x40

#define BFLB_CFGFLAG_DRIVE_MASK			0x300
#define BFLB_CFGFLAG_DRIVE_SHIFT		16U

/**
 * @brief helper macro to encode an IO port pin in a numerical format
 *
 * - fun Function value. It should be lower case value defined by a
 *       BFLB_FUN_function.
 * - pin Pin number.
 *
 * ex.: How to define uart0 rx/tx pins, which is define as BFLB_FUN_uart0
 *
 *	bflb,pins = <BFLB_PIN(7, uart)>,
 *		    <BFLB_PIN(16, uart)>;
 */
#define BFLB_PIN(pin, fun) (BFLB_FUN_##fun | (pin << 5))

#endif	/* ZEPHYR_BFLB_PINCTRL_H_ */
