/*
 * Copyright (c) 2021 Gerson Fernando Budke <nandojve@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_HAL_BFLB_HBN_H_
#define ZEPHYR_HAL_BFLB_HBN_H_

#ifdef CONFIG_SOC_SERIES_BL6
	#include <bl602_hbn.h>
#elif CONFIG_SOC_SERIES_BL7
	#include <bl702_hbn.h>
#endif

#endif	/* ZEPHYR_HAL_BFLB_HBN_H_ */
