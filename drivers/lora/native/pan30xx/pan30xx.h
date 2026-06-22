/*
 * Copyright (c) 2026 MASSDRIVER EI (massdriver.space)
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_LORA_PAN30XX_PAN30XX_INTERNAL_H_
#define ZEPHYR_DRIVERS_LORA_PAN30XX_PAN30XX_INTERNAL_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/sys/atomic.h>

#include "pan30xx_hal.h"
#include "pan30xx_regs.h"

enum pan30xx_state {
	PAN30XX_STATE_DEEP_SLEEP = 0,
	PAN30XX_STATE_SLEEP,
	PAN30XX_STATE_IDLE,
	PAN30XX_STATE_TX,
	PAN30XX_STATE_RX,
};

struct pan30xx_tx_result {
	int status;
};

struct pan30xx_rx_result {
	int16_t rssi;
	int8_t snr;
	uint8_t len;
	int status;
};

struct pan30xx_data {
	struct pan30xx_hal_data hal;

	/* Current state (atomic for lock-free state transitions) */
	atomic_t state;
	struct k_mutex lock;

	/* Current configuration */
	struct lora_modem_config config;
	bool config_valid;

	/* TX completion via message queue */
	struct k_msgq tx_msgq;
	struct pan30xx_tx_result tx_result;

	/* RX completion via message queue */
	struct k_msgq rx_msgq;
	struct pan30xx_rx_result rx_result;

	/* RX data buffer (shared between IRQ handler and recv) */
	uint8_t rx_buf[PAN30XX_MAX_PAYLOAD_LEN];

	/* Async RX callback */
	lora_recv_cb rx_cb;
	void *rx_cb_user_data;

	/* Async TX signal */
	struct k_poll_signal *tx_async_signal;

	/* Deferred work for interrupt handling */
	struct k_work irq_work;
	const struct device *dev;
};

#endif /* ZEPHYR_DRIVERS_LORA_PAN30XX_PAN30XX_INTERNAL_H_ */
