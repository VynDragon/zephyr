/*
 * Copyright (c) 2026 MASSDRIVER EI (massdriver.space)
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/sys/byteorder.h>

#include "pan30xx.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pan30xx, CONFIG_LORA_LOG_LEVEL);

#define PAN30XX_REST_STATE \
	(IS_ENABLED(CONFIG_LORA_PAN30XX_NATIVE_SLEEP) \
	 ? PAN30XX_STATE_SLEEP : PAN30XX_STATE_IDLE)

static int bandwidth_to_reg(enum lora_signal_bandwidth bw, uint8_t *reg)
{
	switch (bw) {
	case BW_62_KHZ:
		*reg = PAN30XX_OP_CR_BW_BW_62_5;
		return 0;
	case BW_125_KHZ:
		*reg = PAN30XX_OP_CR_BW_BW_125;
		return 0;
	case BW_250_KHZ:
		*reg = PAN30XX_OP_CR_BW_BW_250;
		return 0;
	case BW_500_KHZ:
		*reg = PAN30XX_OP_CR_BW_BW_500;
		return 0;
	default:
		return -EINVAL;
	}
}

static int bandwidth_to_hz(enum lora_signal_bandwidth bw, uint32_t *bw_hz)
{
	switch (bw) {
	case BW_62_KHZ:
		*bw_hz = 62500;
		return 0;
	case BW_125_KHZ:
		*bw_hz = 125000;
		return 0;
	case BW_250_KHZ:
		*bw_hz = 250000;
		return 0;
	case BW_500_KHZ:
		*bw_hz = 500000;
		return 0;
	default:
		return -EINVAL;
	}
}

static bool should_enable_ldro(enum lora_datarate sf, enum lora_signal_bandwidth bw,
			       const struct pan30xx_hal_config *config)
{
	uint32_t bw_hz;
	int ret;

	if (config->force_ldro) {
		return true;
	}

	ret = bandwidth_to_hz(bw, &bw_hz);
	__ASSERT_NO_MSG(ret == 0);
	/* Symbol time = 2^SF / BW (in seconds) */
	/* 16.38 ms = 16380 us */
	/* 2^SF / BW > 0.01638 => 2^SF * 1000000 / BW > 16380 */
	uint32_t symbol_time_us = ((1 << sf) * 1000000UL) / bw_hz;

	return symbol_time_us > 16380;
}

static int pan30xx_validate_config(const struct device *dev, const struct lora_modem_config *config)
{
	const struct pan30xx_hal_config *hal_config = dev->config;
	uint8_t bw_reg;

	if (hal_config->variant == PAN30XX_PAN3060) {
		if (config->bandwidth == BW_62_KHZ) {
			LOG_ERR("Unsupported bandwidth: %d kHz", config->bandwidth);
			return -EINVAL;
		}
		if (config->datarate > SF_9) {
			LOG_ERR("Unsupported Spreading Factor: %d", config->datarate);
			return -EINVAL;
		}
	}

	if (bandwidth_to_reg(config->bandwidth, &bw_reg) < 0) {
		LOG_ERR("Unsupported bandwidth: %d kHz", config->bandwidth);
		return -EINVAL;
	}

	return 0;
}

static int pan30xx_set_standby(const struct device *dev, uint8_t mode)
{
	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_SET_STANDBY, &mode, 1);
}

static int pan30xx_set_regulator_mode(const struct device *dev, uint8_t mode)
{
	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_SET_REGULATOR_MODE, &mode, 1);
}

static int pan30xx_set_packet_type(const struct device *dev, uint8_t type)
{
	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_SET_PACKET_TYPE, &type, 1);
}

/* IRQ1: DIO9, IRQ2: DIO11 */
static int pan30xx_set_dio_irq_params(const struct device *dev,
				     uint32_t irq1_mask, uint32_t irq2_mask)
{
	uint8_t buf[PAN30XX_IRQ_SIZE*2];

	sys_put_be32(irq1_mask, &buf[0]);
	sys_put_be32(irq2_mask, &buf[PAN30XX_IRQ_SIZE]);

	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_SET_DIO_IRQ_PARAMS, buf, PAN30XX_IRQ_SIZE*2);
}

static int pan30xx_clear_irq_status(const struct device *dev, uint32_t mask)
{
	uint8_t buf[PAN30XX_IRQ_SIZE];

	sys_put_be32(mask, buf);
	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_CLEAR_IRQ, buf, PAN30XX_IRQ_SIZE);
}

static int pan30xx_clear_errors(const struct device *dev)
{
	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_CLEAR_ERRORS, NULL, 0);
}

static int pan30xx_get_errors(const struct device *dev, uint16_t *errors)
{
	uint8_t buf[2];
	int ret;

	ret = pan30xx_hal_read_cmd(dev, PAN30XX_CMD_GET_ERRORS, buf, 2);
	if (ret != 0) {
		return ret;
	}

	*errors = (uint16_t)buf[0] << 8 | (uint16_t)buf[1];

	return ret;
}

static int pan30xx_configure_tcxo(const struct device *dev, uint8_t voltage, uint32_t timeout_ms)
{
	uint32_t timeout = PAN30XX_MS_TO_TIMEOUT(timeout_ms);
	uint8_t buf[4];

	buf[0] = voltage;
	sys_put_be24(timeout, &buf[1]);

	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_SET_TCXO_MODE, buf, 4);
}

static int pan30xx_calibrate(const struct device *dev, uint8_t mask)
{
	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_CALIBRATE, &mask, 1);
}

static int pan30xx_calibrate_image(const struct device *dev, uint32_t freq)
{
	uint8_t buf[2];
	uint32_t freq_mhz = freq / MHZ(1);

	/* Pick a range + and - roughly 10 MHz since we recalibrate at each frequency changes */
	buf[0] = (freq_mhz - 10) / 4;
	buf[1] = DIV_ROUND_UP((freq_mhz + 10), 4);

	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_CALIBRATE_IMAGE, buf, 2);
}

static int pan30xx_set_rf_frequency(const struct device *dev, uint32_t freq)
{
	uint8_t buf[4];

	sys_put_be32(freq, buf);

	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_SET_RF_FREQUENCY, buf, 4);
}

static int pan30xx_set_modulation_params(const struct device *dev,
					uint8_t sf, uint8_t bw, uint8_t cr,
					bool ldro)
{
	uint8_t buf[4] = { sf, bw, cr, ldro };

	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_SET_MODULATION_PARAMS, buf, 4);
}

static int pan30xx_set_packet_params(const struct device *dev,
				    uint16_t preamble_len, uint8_t header_type,
				    uint8_t payload_len, uint8_t crc_mode,
				    uint8_t invert_iq)
{
	uint8_t buf[6];

	sys_put_be16(preamble_len, &buf[0]);
	buf[2] = header_type;
	buf[3] = payload_len;
	buf[4] = crc_mode;
	buf[5] = invert_iq;

	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_SET_PACKET_PARAMS, buf, 6);
}

static int pan30xx_set_sync_word(const struct device *dev, const struct lora_modem_config *config)
{
	uint8_t sync_word;

	if (config->sync_word != 0) {
		sync_word = config->sync_word;
	} else {
		sync_word = config->public_network ? PAN30XX_LORA_SYNC_WORD_PUBLIC
						   : PAN30XX_LORA_SYNC_WORD_PRIVATE;
	}

	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_SET_LORA_SYNCWORD, &sync_word, 1);
}

static int pan30xx_set_tx(const struct device *dev, uint32_t timeout_ms)
{
	uint32_t timeout = PAN30XX_MS_TO_TIMEOUT(timeout_ms);
	uint8_t buf[3];

	sys_put_be24(timeout, buf);
	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_SET_TX, buf, 3);
}

static int pan30xx_set_stop_timer_on_preamble(const struct device *dev,
					    bool enable)
{
	uint8_t val = enable;

	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_STOP_TIMEOUT_ON_PREAMBLE,
				    &val, 1);
}

static int pan30xx_set_rx_duty_cycle(const struct device *dev,
				    uint32_t rx_period, uint32_t sleep_period)
{
	uint8_t buf[7];
	int ret;

	sys_put_be24(rx_period, &buf[0]);
	sys_put_be24(sleep_period, &buf[3]);
	/* Mode = RX mode, not CAD mode */
	buf[6] = 0;

	ret = pan30xx_set_stop_timer_on_preamble(dev, true);
	if (ret < 0) {
		return ret;
	}

	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_SET_RX_DUTY_CYCLE, buf, 7);
}

static int pan30xx_set_rx(const struct device *dev, uint32_t timeout_ms)
{
	uint32_t timeout;

	if (timeout_ms == 0) {
		timeout = PAN30XX_RX_TIMEOUT_CONTINUOUS;
	} else {
		timeout = PAN30XX_MS_TO_TIMEOUT(timeout_ms);
	}

	uint8_t buf[3];

	sys_put_be24(timeout, buf);
	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_SET_RX, buf, 3);
}

static int pan30xx_get_rx_buffer_status(const struct device *dev,
				       uint8_t *payload_len, uint8_t *offset)
{
	uint8_t buf[2];
	int ret;

	ret = pan30xx_hal_read_cmd(dev, PAN30XX_CMD_GET_RX_BUFFER_STATUS, buf, 2);
	if (ret == 0) {
		*payload_len = buf[0];
		*offset = buf[1];
	}

	return ret;
}

static int pan30xx_get_packet_status(const struct device *dev,
				    int16_t *rssi, int8_t *snr)
{
	uint8_t buf[3];
	int ret;

	ret = pan30xx_hal_read_cmd(dev, PAN30XX_CMD_GET_PACKET_STATUS, buf, 3);
	if (ret == 0) {
		/* RSSI is -value/2 dBm */
		*rssi = -((int16_t)buf[0] >> 1);
		/* SNR is value/4 dB (signed) */
		*snr = ((int8_t)buf[1]) >> 2;
	}

	return ret;
}

static int pan30xx_set_lf_clk(const struct device *dev, enum pan30xx_lf_clock clk)
{
	uint8_t buf = clk | (1U << 2);

	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_CONFIG_LF_CLOCK, &buf, 1);
}

static int pan30xx_configure_rf_switches(const struct device *dev)
{
	const struct pan30xx_hal_config *config = dev->config;
	uint8_t buf[8] = { config->rfsw.enable, config->rfsw.standby, config->rfsw.rx,
		config->rfsw.tx, config->rfsw.tx_hp, config->rfsw.tx_hf, config->rfsw.gnss,
		config->rfsw.wifi };

	if (config->rfsw.enable == 0) {
		return 0;
	}

	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_SET_DIO_AS_RF_SWITCH, buf, 8);
}

static int pan30xx_set_rfsw_pulls_sleep(const struct device *dev, bool yes)
{
	uint8_t buf = yes ? 0x1 : 0x0;

	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_DRIVE_DIO_SLEEP_MODE, &buf, 1);
}

static int pan30xx_chip_init(const struct device *dev)
{
	const struct pan30xx_hal_config *config = dev->config;
	uint8_t version_buf[4];
	uint16_t errors;
	int ret;

	/* Hardware reset */
	ret = pan30xx_hal_reset(dev);
	if (ret < 0) {
		LOG_ERR("Reset failed: %d", ret);
		return ret;
	}

	/* Set standby mode */
	ret = pan30xx_set_standby(dev, PAN30XX_STANDBY_RC);
	if (ret < 0) {
		LOG_ERR("Set standby failed: %d", ret);
		return ret;
	}

	ret = pan30xx_clear_errors(dev);
	if (ret != 0) {
		LOG_ERR("Failed clearing errors: %d", ret);
		return ret;
	}

	ret = pan30xx_hal_read_cmd(dev, PAN30XX_CMD_GET_VERSION, version_buf, 4);
	if (ret < 0) {
		LOG_ERR("Fetch version failed: %d", ret);
		return ret;
	}

	if (config->variant != version_buf[1]) {
		LOG_ERR("HW use case (%x) doesn't match variant configured (%x)",
			version_buf[1], config->variant);
		return -ENODEV;
	}

	/* Pre-0x303 version LR1110 */
	if ((version_buf[2] < 0x03 || (version_buf[2] == 0x3 && version_buf[3] < 0x03))
	    && config->variant == PAN30XX_LR1110) {
		LOG_ERR("Please upgrade chip firmware: changing sync word is unsupported");
		return -ENODEV;
	}

	/* Set regulator mode */
	ret = pan30xx_set_regulator_mode(dev, config->use_dcdc ?
					PAN30XX_REGULATOR_DCDC : PAN30XX_REGULATOR_LDO);
	if (ret < 0) {
		LOG_ERR("Set regulator failed: %d", ret);
		return ret;
	}

	ret = pan30xx_configure_rf_switches(dev);
	if (ret < 0) {
		LOG_ERR("Configuring RF switches failed: %d", ret);
		return ret;
	}

	ret = pan30xx_set_lf_clk(dev, config->lf_clk);
	if (ret < 0) {
		LOG_ERR("Configuring LF clock failed: %d", ret);
		return ret;
	}

	ret = pan30xx_get_errors(dev, &errors);
	if (ret != 0) {
		LOG_ERR("Failed fetching errors: %d", ret);
		return ret;
	}

	if (errors != 0) {
		LOG_ERR("There were errors initializing: %x", errors);
		return -EIO;
	}

	/* Configure TCXO if enabled */
	if (config->tcxo_voltage < PAN30XX_NO_TCXO) {
		ret = pan30xx_configure_tcxo(dev, config->tcxo_voltage,
					    config->tcxo_startup_delay_ms);
		if (ret < 0) {
			LOG_ERR("Set TCXO failed: %d", ret);
			return ret;
		}

		/* Run full calibration after TCXO setup */

		ret = pan30xx_clear_errors(dev);
		if (ret != 0) {
			LOG_ERR("Failed clearing errors: %d", ret);
			return ret;
		}

		ret = pan30xx_calibrate(dev, PAN30XX_CALIBRATE_ALL);
		if (ret < 0) {
			LOG_ERR("Calibration failed: %d", ret);
			return ret;
		}

		ret = pan30xx_get_errors(dev, &errors);
		if (ret != 0) {
			LOG_ERR("Failed fetching errors: %d", ret);
			return ret;
		}

		if (errors != 0) {
			LOG_ERR("There were errors calibrating: %x", errors);
			return -EIO;
		}
	}

	/* Save power in sleep mode */
	ret = pan30xx_set_rfsw_pulls_sleep(dev, true);
	if (ret < 0) {
		LOG_ERR("Failed to set sleep pulls: %d", ret);
		return ret;
	}

	/* Set packet type to LoRa */
	ret = pan30xx_set_packet_type(dev, PAN30XX_PACKET_TYPE_LORA);
	if (ret < 0) {
		LOG_ERR("Set packet type failed: %d", ret);
		return ret;
	}

	/* Configure IRQs on IRQ1: TX done, RX done, timeout */
	uint16_t irq_mask = PAN30XX_IRQ_TX_DONE | PAN30XX_IRQ_RX_DONE |
			    PAN30XX_IRQ_RX_TX_TIMEOUT | PAN30XX_IRQ_CRC_ERR;
	ret = pan30xx_set_dio_irq_params(dev, irq_mask, 0);
	if (ret < 0) {
		LOG_ERR("Set IRQ params failed: %d", ret);
		return ret;
	}

	/* Clear any pending IRQs */
	ret = pan30xx_clear_irq_status(dev, PAN30XX_IRQ_ALL);
	if (ret < 0) {
		LOG_ERR("Clear IRQ failed: %d", ret);
		return ret;
	}

#if CONFIG_LORA_LOG_LEVEL >= LOG_LEVEL_INF
	char *lr11x_str;

	switch (config->variant) {
	case PAN30XX_LR1110:
		lr11x_str = "LR1110";
		break;
	case PAN30XX_LR1120:
		lr11x_str = "LR1120";
		break;
	case PAN30XX_LR1121:
		lr11x_str = "LR1121";
		break;
	default:
		lr11x_str = "Unknown LR11xx";
	};
	LOG_INF("%s initialized with HW version %x and FW version %x%02x", lr11x_str,
		version_buf[0], version_buf[2], version_buf[3]);
#endif
	return 0;
}

static void pan30xx_irq_callback(const struct device *dev)
{
	struct pan30xx_data *data = dev->data;

	k_work_submit(&data->irq_work);
}

static int pan30xx_set_sleep(const struct device *dev)
{
	struct pan30xx_data *data = dev->data;
	uint8_t cfg[5] = { PAN30XX_SLEEP_WARM_START, 0x00, 0x00, 0x00, 0x00};
	int ret;

	if (!IS_ENABLED(CONFIG_LORA_PAN30XX_NATIVE_SLEEP)) {
		atomic_set(&data->state, PAN30XX_STATE_IDLE);
		return 0;
	}

	if (atomic_get(&data->state) == PAN30XX_STATE_SLEEP) {
		return 0;
	}

	pan30xx_hal_set_irq_callback(dev, NULL);

	/* No timeout, 32k is disabled so prevent waiting on XTAL */
	ret = pan30xx_set_lf_clk(dev, PAN30XX_LF_CLOCK_RC);
	if (ret != 0) {
		LOG_ERR("Failed to set LF clock for sleep");
		return ret;
	}

	ret = pan30xx_hal_write_cmd(dev, PAN30XX_CMD_SET_SLEEP, cfg, sizeof(cfg));
	if (ret == 0) {
		atomic_set(&data->state, PAN30XX_STATE_SLEEP);
	}

	return ret;
}

static int pan30xx_ensure_ready(const struct device *dev)
{
	const struct pan30xx_hal_config *config = dev->config;
	int ret;

	/* Re-enable interrupt */
	ret = pan30xx_hal_set_irq_callback(dev, pan30xx_irq_callback);
	if (ret < 0) {
		return ret;
	}

	if (!IS_ENABLED(CONFIG_LORA_PAN30XX_NATIVE_SLEEP)) {
		return 0;
	}

	ret = pan30xx_hal_wakeup(dev);
	if (ret != 0) {
		return ret;
	}

	ret = pan30xx_set_lf_clk(dev, config->lf_clk);
	if (ret != 0) {
		LOG_ERR("Failed to restore LF clock");
		return ret;
	}

	return ret;
}

static void pan30xx_handle_irq_tx_done(const struct device *dev)
{
	struct pan30xx_data *data = dev->data;
	struct pan30xx_tx_result result = { .status = 0 };

	LOG_DBG("TX done");
	pan30xx_set_sleep(dev);

	if (data->tx_async_signal != NULL) {
		k_poll_signal_raise(data->tx_async_signal, 0);
		data->tx_async_signal = NULL;
	}
	k_msgq_put(&data->tx_msgq, &result, K_NO_WAIT);
}

static void pan30xx_rx_restart(const struct device *dev)
{
	struct pan30xx_data *data = dev->data;

	if (atomic_get(&data->state) == PAN30XX_STATE_RX_DUTY_CYCLE) {
		int ret;

		ret = pan30xx_set_rx_duty_cycle(dev,
					       data->duty_cycle.rx_period,
					       data->duty_cycle.sleep_period);
		if (ret < 0) {
			data->rx_cb = NULL;
			data->rx_cb_user_data = NULL;
			atomic_set(&data->state, PAN30XX_REST_STATE);
			pan30xx_set_sleep(dev);
		}
	} else {
		pan30xx_set_rx(dev, 0);
	}
}

static void pan30xx_handle_irq_rx_done(const struct device *dev, uint32_t irq_status)
{
	struct pan30xx_data *data = dev->data;
	struct pan30xx_rx_result result = { 0 };
	uint8_t payload_len = 0, offset = 0;
	int ret;

	/* Get received packet info */
	ret = pan30xx_get_rx_buffer_status(dev, &payload_len, &offset);
	if (ret < 0) {
		LOG_ERR("Failed to get RX buffer status");
		result.status = ret;
		goto out;
	}

	pan30xx_get_packet_status(dev, &result.rssi, &result.snr);

	if (irq_status & PAN30XX_IRQ_CRC_ERR) {
		LOG_WRN("CRC error");
		result.status = -EIO;
		goto out;
	}

	result.len = MIN(payload_len, sizeof(data->rx_buf));
	ret = pan30xx_hal_read_buffer(dev, offset, data->rx_buf, result.len);
	if (ret < 0) {
		LOG_ERR("Failed to read RX buffer");
		result.status = ret;
		goto out;
	}

	result.status = result.len;
	LOG_DBG("RX done: %d bytes, RSSI=%d, SNR=%d",
		result.len, result.rssi, result.snr);

out:
	if (data->rx_cb == NULL) {
		pan30xx_set_sleep(dev);
		k_msgq_put(&data->rx_msgq, &result, K_NO_WAIT);
		return;
	}

	/* Async mode: deliver valid packets, drop errors silently */
	if (result.status > 0) {
		data->rx_cb(dev, data->rx_buf, result.len,
			    result.rssi, result.snr,
			    data->rx_cb_user_data);
	}

	/* Restart reception unless the callback stopped it */
	if (data->rx_cb != NULL) {
		pan30xx_rx_restart(dev);
	}
}

static void pan30xx_handle_irq_timeout(const struct device *dev)
{
	struct pan30xx_data *data = dev->data;

	LOG_DBG("Timeout");

	if (atomic_get(&data->state) == PAN30XX_STATE_RX_DUTY_CYCLE) {
		/*
		 * Preamble was detected but full packet reception failed.
		 * The radio fell back to STDBY_RC — restart the duty cycle.
		 */
		pan30xx_rx_restart(dev);
		return;
	}

	pan30xx_set_sleep(dev);

	if (data->tx_async_signal != NULL) {
		struct pan30xx_tx_result result = { .status = -ETIMEDOUT };

		k_poll_signal_raise(data->tx_async_signal, -ETIMEDOUT);
		data->tx_async_signal = NULL;
		k_msgq_put(&data->tx_msgq, &result, K_NO_WAIT);
	} else if (data->rx_cb == NULL) {
		/* Sync RX timeout */
		struct pan30xx_rx_result result = { .status = -EAGAIN };

		k_msgq_put(&data->rx_msgq, &result, K_NO_WAIT);
	}
}

static void pan30xx_irq_work_handler(struct k_work *work)
{
	struct pan30xx_data *data = CONTAINER_OF(work, struct pan30xx_data, irq_work);
	const struct device *dev = data->dev;
	uint32_t irq_status = 0;
	int ret;

	ret = pan30xx_hal_read_status(dev, NULL, &irq_status);
	if (ret < 0) {
		LOG_ERR("Failed to get IRQ status");
		return;
	}

	LOG_DBG("IRQ status: 0x%08x", irq_status);

	/* Clear handled IRQs */
	pan30xx_clear_irq_status(dev, irq_status);

	if (irq_status & PAN30XX_IRQ_TX_DONE) {
		pan30xx_handle_irq_tx_done(dev);
	}

	if (irq_status & PAN30XX_IRQ_RX_DONE) {
		pan30xx_handle_irq_rx_done(dev, irq_status);
	}

	if (irq_status & PAN30XX_IRQ_RX_TX_TIMEOUT) {
		pan30xx_handle_irq_timeout(dev);
	}

	/* Re-enable the interrupt for the next event (unless sleeping) */
	if (atomic_get(&data->state) != PAN30XX_REST_STATE) {
		pan30xx_hal_irq_enable(dev);
	}
}

static int pan30xx_configure_tx_params(const struct device *dev, int8_t power,
				      uint32_t frequency, uint8_t ramp_time)
{
	const struct pan30xx_hal_config *config = dev->config;
	int8_t tx_power;
	uint8_t buf[4];
	int ret;

	if (config->variant == PAN30XX_LR1121) {
		if (frequency > PAN30XX_SUBGHZ_THRESHOLD) {
			tx_power = CLAMP(power, LR1121_MIN_POWER_HF, LR1121_MAX_POWER_HF);
			buf[0] = PAN30XX_PA_HF;
			buf[1] = 0x0;
			buf[2] = LR1121_PA_DUTY_CYCLE_HF;
			buf[3] = 0x4;
		} else {
			tx_power = CLAMP(power, LR1121_MIN_POWER_LF, LR1121_MAX_POWER_LF);
			/* LP PA */
			if (tx_power <= LR1121_LF_PA_THRESHOLD) {
				buf[0] = PAN30XX_PA_LP;
				buf[1] = 0x0;
				buf[2] = LR1121_PA_DUTY_CYCLE_LF;
				buf[3] = 0x4;
			/* HP PA */
			} else {
				buf[0] = PAN30XX_PA_HP;
				buf[1] = 0x1;
				buf[2] = LR1121_PA_DUTY_CYCLE_LF;
				buf[3] = 0x7;
			}
		}
		ret = pan30xx_hal_write_cmd(dev, PAN30XX_CMD_SET_PA_CONFIG, buf, 4);
		if (ret < 0) {
			return ret;
		}
	} else {
		return -ENOTSUP;
	}

	buf[0] = (uint8_t)tx_power;
	buf[1] = ramp_time;

	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_SET_TX_PARAMS, buf, 2);
}

static int pan30xx_set_rx_boosted(const struct device *dev, bool boosted)
{
	uint8_t val = boosted ? 1U : 0U;

	return pan30xx_hal_write_cmd(dev, PAN30XX_CMD_SET_RX_BOOSTED, &val, 1);
}

static int pan30xx_lora_config(const struct device *dev,
			      const struct lora_modem_config *config)
{
	struct pan30xx_data *data = dev->data;
	const struct pan30xx_hal_config *hal_config = dev->config;
	uint8_t bw_reg;
	bool ldro;
	int ret;

	ret = pan30xx_validate_config(dev, config);
	if (ret < 0) {
		return ret;
	}

	if (!atomic_cas(&data->state, PAN30XX_REST_STATE, PAN30XX_STATE_IDLE)) {
		return -EBUSY;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = pan30xx_ensure_ready(dev);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		atomic_set(&data->state, PAN30XX_REST_STATE);
		return ret;
	}

	/* Store configuration */
	memcpy(&data->config, config, sizeof(*config));

	if (config->frequency < PAN30XX_SUBGHZ_THRESHOLD) {
		/* Run image calibration for frequency band when doing sub-GHz */
		ret = pan30xx_calibrate_image(dev, config->frequency);
		if (ret < 0) {
			goto out;
		}
	}

	/* Set RF frequency */
	ret = pan30xx_set_rf_frequency(dev, config->frequency);
	if (ret < 0) {
		goto out;
	}

	/* Configure PA and TX power based on chip variant and frequency */
	ret = pan30xx_configure_tx_params(dev, config->tx_power,
					 config->frequency,
					 PAN30XX_RAMP_64_US);
	if (ret < 0) {
		goto out;
	}

	/* Set modulation parameters */
	ret = bandwidth_to_reg(config->bandwidth, &bw_reg);
	__ASSERT_NO_MSG(ret == 0);
	ldro = should_enable_ldro(config->datarate, config->bandwidth, hal_config);
	ret = pan30xx_set_modulation_params(dev,
					   config->datarate,
					   bw_reg,
					   config->coding_rate,
					   ldro);
	if (ret < 0) {
		goto out;
	}

	/* Set sync word */
	ret = pan30xx_set_sync_word(dev, config);
	if (ret < 0) {
		goto out;
	}

	ret = pan30xx_set_rx_boosted(dev, hal_config->rx_boosted);
	if (ret < 0) {
		goto out;
	}

	data->config_valid = true;
	LOG_DBG("Config: freq=%u, SF=%d, BW=%d, CR=%d, power=%d",
		config->frequency, config->datarate, config->bandwidth,
		config->coding_rate, config->tx_power);

out:
	pan30xx_set_sleep(dev);
	k_mutex_unlock(&data->lock);
	return ret;
}

static int pan30xx_lora_send_async(const struct device *dev,
				  uint8_t *data_buf, uint32_t data_len,
				  struct k_poll_signal *async)
{
	struct pan30xx_data *data = dev->data;
	int ret;

	if (!data->config_valid) {
		LOG_ERR("Not configured");
		return -EINVAL;
	}

	if (data_len > PAN30XX_MAX_PAYLOAD_LEN) {
		LOG_ERR("Payload too long: %u", data_len);
		return -EINVAL;
	}

	if (!atomic_cas(&data->state, PAN30XX_REST_STATE, PAN30XX_STATE_TX)) {
		LOG_ERR("Busy");
		return -EBUSY;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = pan30xx_ensure_ready(dev);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		atomic_set(&data->state, PAN30XX_REST_STATE);
		return ret;
	}

	data->tx_async_signal = async;
	k_msgq_purge(&data->tx_msgq);

	/* Set packet parameters */
	ret = pan30xx_set_packet_params(dev,
				       data->config.preamble_len,
				       PAN30XX_LORA_HEADER_EXPLICIT,
				       data_len,
				       data->config.packet_crc_disable ?
				       PAN30XX_LORA_CRC_OFF : PAN30XX_LORA_CRC_ON,
				       data->config.iq_inverted ?
				       PAN30XX_LORA_IQ_INVERTED : PAN30XX_LORA_IQ_STANDARD);
	if (ret < 0) {
		goto out_error;
	}

	/* Write payload to buffer */
	ret = pan30xx_hal_write_buffer(dev, data_buf, data_len);
	if (ret < 0) {
		goto out_error;
	}

	/* Start transmission with 10 second timeout */
	ret = pan30xx_set_tx(dev, 10000);
	if (ret < 0) {
		goto out_error;
	}

	k_mutex_unlock(&data->lock);
	return 0;

out_error:
	data->tx_async_signal = NULL;
	pan30xx_set_sleep(dev);
	k_mutex_unlock(&data->lock);
	return ret;
}

static int pan30xx_lora_send(const struct device *dev,
			    uint8_t *data_buf, uint32_t data_len)
{
	struct pan30xx_data *data = dev->data;
	struct pan30xx_tx_result result;
	int ret;

	ret = pan30xx_lora_send_async(dev, data_buf, data_len, NULL);
	if (ret < 0) {
		return ret;
	}

	/* Wait for TX completion */
	ret = k_msgq_get(&data->tx_msgq, &result, K_SECONDS(15));
	if (ret < 0) {
		LOG_ERR("TX timeout");
		/* Chip is still transmitting, abort first */
		pan30xx_set_standby(dev, PAN30XX_STANDBY_RC);
		pan30xx_set_sleep(dev);
		return -ETIMEDOUT;
	}

	return result.status;
}

static int pan30xx_lora_recv(const struct device *dev, uint8_t *data_buf,
			    uint8_t size, k_timeout_t timeout,
			    int16_t *rssi, int8_t *snr)
{
	struct pan30xx_data *data = dev->data;
	struct pan30xx_rx_result result;
	uint32_t timeout_ms;
	int ret;

	if (!data->config_valid) {
		LOG_ERR("Not configured");
		return -EINVAL;
	}

	if (!atomic_cas(&data->state, PAN30XX_REST_STATE, PAN30XX_STATE_RX)) {
		LOG_ERR("Busy");
		return -EBUSY;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = pan30xx_ensure_ready(dev);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		atomic_set(&data->state, PAN30XX_REST_STATE);
		return ret;
	}

	data->rx_cb = NULL;
	k_msgq_purge(&data->rx_msgq);

	/* Set packet parameters for variable length reception */
	ret = pan30xx_set_packet_params(dev,
				       data->config.preamble_len,
				       PAN30XX_LORA_HEADER_EXPLICIT,
				       PAN30XX_MAX_PAYLOAD_LEN,
				       data->config.packet_crc_disable ?
				       PAN30XX_LORA_CRC_OFF : PAN30XX_LORA_CRC_ON,
				       data->config.iq_inverted ?
				       PAN30XX_LORA_IQ_INVERTED : PAN30XX_LORA_IQ_STANDARD);
	if (ret < 0) {
		pan30xx_set_sleep(dev);
		k_mutex_unlock(&data->lock);
		return ret;
	}

	/* Start reception (0 = continuous for K_FOREVER) */
	timeout_ms = K_TIMEOUT_EQ(timeout, K_FOREVER)
		     ? 0 : k_ticks_to_ms_ceil32(timeout.ticks);
	ret = pan30xx_set_rx(dev, timeout_ms);
	if (ret < 0) {
		pan30xx_set_sleep(dev);
		k_mutex_unlock(&data->lock);
		return ret;
	}

	k_mutex_unlock(&data->lock);

	/* Wait for RX completion */
	ret = k_msgq_get(&data->rx_msgq, &result, timeout);
	if (ret < 0) {
		LOG_DBG("RX timeout");
		/* Chip is still receiving, abort first */
		pan30xx_set_standby(dev, PAN30XX_STANDBY_RC);
		pan30xx_set_sleep(dev);
		return -EAGAIN;
	}

	/* Copy received data from shared buffer */
	if (result.status > 0) {
		int copy_len = MIN(result.status, size);

		memcpy(data_buf, data->rx_buf, copy_len);
		if (rssi != NULL) {
			*rssi = result.rssi;
		}
		if (snr != NULL) {
			*snr = result.snr;
		}
		return copy_len;
	}

	return result.status;
}

static int pan30xx_lora_recv_async(const struct device *dev,
				  lora_recv_cb cb, void *user_data)
{
	struct pan30xx_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (cb == NULL) {
		/* Stop async reception */
		data->rx_cb = NULL;
		data->rx_cb_user_data = NULL;
		if (atomic_cas(&data->state, PAN30XX_STATE_RX, PAN30XX_STATE_IDLE)) {
			pan30xx_set_standby(dev, PAN30XX_STANDBY_RC);
			pan30xx_set_sleep(dev);
		}
		k_mutex_unlock(&data->lock);
		return 0;
	}

	if (!data->config_valid) {
		LOG_ERR("Not configured");
		k_mutex_unlock(&data->lock);
		return -EINVAL;
	}

	if (!atomic_cas(&data->state, PAN30XX_REST_STATE, PAN30XX_STATE_RX)) {
		LOG_ERR("Busy");
		k_mutex_unlock(&data->lock);
		return -EBUSY;
	}

	ret = pan30xx_ensure_ready(dev);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		atomic_set(&data->state, PAN30XX_REST_STATE);
		return ret;
	}

	data->rx_cb = cb;
	data->rx_cb_user_data = user_data;

	/* Set packet parameters */
	ret = pan30xx_set_packet_params(dev,
				       data->config.preamble_len,
				       PAN30XX_LORA_HEADER_EXPLICIT,
				       PAN30XX_MAX_PAYLOAD_LEN,
				       data->config.packet_crc_disable ?
				       PAN30XX_LORA_CRC_OFF : PAN30XX_LORA_CRC_ON,
				       data->config.iq_inverted ?
				       PAN30XX_LORA_IQ_INVERTED : PAN30XX_LORA_IQ_STANDARD);
	if (ret < 0) {
		data->rx_cb = NULL;
		pan30xx_set_sleep(dev);
		k_mutex_unlock(&data->lock);
		return ret;
	}

	/* Start continuous reception */
	ret = pan30xx_set_rx(dev, 0);
	if (ret < 0) {
		data->rx_cb = NULL;
		pan30xx_set_sleep(dev);
		k_mutex_unlock(&data->lock);
		return ret;
	}

	k_mutex_unlock(&data->lock);
	return 0;
}

static void pan30xx_duty_cycle_stop(const struct device *dev)
{
	/*
	 * During duty cycle the BUSY pin stays asserted even in the
	 * sleep phase. Wake the radio via a raw NSS edge (bypasses
	 * BUSY check) so the standby command can go through
	 * immediately.
	 */
	if (pan30xx_hal_is_busy(dev)) {
		pan30xx_hal_wakeup(dev);
	}
	pan30xx_set_standby(dev, PAN30XX_STANDBY_RC);
	pan30xx_set_sleep(dev);
}

static int pan30xx_duty_cycle_start(const struct device *dev,
				   k_timeout_t rx_period,
				   k_timeout_t sleep_period)
{
	struct pan30xx_data *data = dev->data;
	int ret;

	data->duty_cycle.rx_period = PAN30XX_MS_TO_TIMEOUT(
		k_ticks_to_ms_ceil32(rx_period.ticks));
	data->duty_cycle.sleep_period = PAN30XX_MS_TO_TIMEOUT(
		k_ticks_to_ms_ceil32(sleep_period.ticks));

	ret = pan30xx_set_packet_params(dev,
				       data->config.preamble_len,
				       PAN30XX_LORA_HEADER_EXPLICIT,
				       PAN30XX_MAX_PAYLOAD_LEN,
				       data->config.packet_crc_disable ?
				       PAN30XX_LORA_CRC_OFF : PAN30XX_LORA_CRC_ON,
				       data->config.iq_inverted ?
				       PAN30XX_LORA_IQ_INVERTED :
				       PAN30XX_LORA_IQ_STANDARD);
	if (ret < 0) {
		goto out_error;
	}

	ret = pan30xx_set_rx_duty_cycle(dev, data->duty_cycle.rx_period,
				       data->duty_cycle.sleep_period);
	if (ret < 0) {
		goto out_error;
	}

	k_mutex_unlock(&data->lock);
	return 0;

out_error:
	data->rx_cb = NULL;
	atomic_set(&data->state, PAN30XX_REST_STATE);
	pan30xx_set_sleep(dev);
	k_mutex_unlock(&data->lock);
	return ret;
}

static int pan30xx_duty_cycle_enter(const struct device *dev,
				   lora_recv_cb cb, void *user_data)
{
	struct pan30xx_data *data = dev->data;
	int ret;

	if (!data->config_valid) {
		LOG_ERR("Not configured");
		return -EINVAL;
	}

	if (!atomic_cas(&data->state, PAN30XX_REST_STATE,
			PAN30XX_STATE_RX_DUTY_CYCLE)) {
		LOG_ERR("Busy");
		return -EBUSY;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = pan30xx_ensure_ready(dev);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		atomic_set(&data->state, PAN30XX_REST_STATE);
		return ret;
	}

	data->rx_cb = cb;
	data->rx_cb_user_data = user_data;

	return 0;
}

static int pan30xx_lora_recv_duty_cycle(const struct device *dev,
				       k_timeout_t rx_period,
				       k_timeout_t sleep_period,
				       uint8_t *data_buf, uint8_t size,
				       k_timeout_t timeout,
				       int16_t *rssi, int8_t *snr)
{
	struct pan30xx_data *data = dev->data;
	struct pan30xx_rx_result result;
	int ret;

	ret = pan30xx_duty_cycle_enter(dev, NULL, NULL);
	if (ret < 0) {
		return ret;
	}

	k_msgq_purge(&data->rx_msgq);

	ret = pan30xx_duty_cycle_start(dev, rx_period, sleep_period);
	if (ret < 0) {
		return ret;
	}

	/* Block until a packet arrives or timeout */
	ret = k_msgq_get(&data->rx_msgq, &result, timeout);
	if (ret < 0) {
		LOG_DBG("RX duty cycle timeout");
		pan30xx_duty_cycle_stop(dev);
		return -EAGAIN;
	}

	pan30xx_duty_cycle_stop(dev);

	if (result.status > 0) {
		int copy_len = MIN(result.status, size);

		memcpy(data_buf, data->rx_buf, copy_len);
		if (rssi != NULL) {
			*rssi = result.rssi;
		}
		if (snr != NULL) {
			*snr = result.snr;
		}
		return copy_len;
	}

	return result.status;
}

static int pan30xx_lora_recv_duty_cycle_async(const struct device *dev,
					     k_timeout_t rx_period,
					     k_timeout_t sleep_period,
					     lora_recv_cb cb, void *user_data)
{
	struct pan30xx_data *data = dev->data;
	int ret;

	if (cb == NULL) {
		k_mutex_lock(&data->lock, K_FOREVER);
		data->rx_cb = NULL;
		data->rx_cb_user_data = NULL;
		if (atomic_cas(&data->state, PAN30XX_STATE_RX_DUTY_CYCLE,
			       PAN30XX_STATE_IDLE)) {
			pan30xx_duty_cycle_stop(dev);
		}
		k_mutex_unlock(&data->lock);
		return 0;
	}

	ret = pan30xx_duty_cycle_enter(dev, cb, user_data);
	if (ret < 0) {
		return ret;
	}

	return pan30xx_duty_cycle_start(dev, rx_period, sleep_period);
}

static uint32_t pan30xx_lora_airtime(const struct device *dev, uint32_t data_len)
{
	struct pan30xx_data *data = dev->data;
	uint32_t t_preamble_us, t_payload_us, t_sym_us, n_payload, bw_hz;
	uint8_t sf, cr;
	int32_t tmp;
	bool de, crc;
	int ret;

	if (!data->config_valid) {
		return 0;
	}

	/* Calculate symbol time in microseconds */
	ret = bandwidth_to_hz(data->config.bandwidth, &bw_hz);
	__ASSERT_NO_MSG(ret == 0);
	sf = data->config.datarate;

	/* Symbol time = 2^SF / BW (seconds) */
	/* In microseconds: (2^SF * 1000000) / BW */
	t_sym_us = ((1UL << sf) * 1000000UL) / bw_hz;

	/* Preamble time (4.25 extra symbols) */
	t_preamble_us = (data->config.preamble_len + 4) * t_sym_us +
			(t_sym_us / 4);

	/* Payload symbol count calculation (from LoRa modem designer's guide) */
	de = should_enable_ldro(sf, data->config.bandwidth, dev->config);
	crc = !data->config.packet_crc_disable;
	cr = data->config.coding_rate;

	tmp = 8 * data_len - 4 * sf + 28 + 16 * crc;
	if (tmp < 0) {
		tmp = 0;
	}

	n_payload = 8 + (((tmp + 4 * (sf - 2 * de) - 1) /
			  (4 * (sf - 2 * de))) * (cr + 4));
	t_payload_us = n_payload * t_sym_us;

	/* Total airtime in milliseconds */
	return (t_preamble_us + t_payload_us + 500) / 1000;
}

static int pan30xx_lora_test_cw(const struct device *dev, uint32_t frequency,
			       int8_t tx_power, uint16_t duration)
{
	struct pan30xx_data *data = dev->data;
	int ret;

	if (!atomic_cas(&data->state, PAN30XX_REST_STATE, PAN30XX_STATE_TX)) {
		return -EBUSY;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = pan30xx_ensure_ready(dev);
	if (ret < 0) {
		goto out_unlock;
	}

	/* Set frequency */
	ret = pan30xx_set_rf_frequency(dev, frequency);
	if (ret < 0) {
		goto out_sleep;
	}

	/* Set PA config and TX power */
	ret = pan30xx_configure_tx_params(dev, tx_power, frequency, PAN30XX_RAMP_64_US);
	if (ret < 0) {
		goto out_sleep;
	}

	/* Start CW transmission */
	ret = pan30xx_hal_write_cmd(dev, PAN30XX_CMD_SET_TX_CONTINUOUS_WAVE, NULL, 0);
	if (ret < 0) {
		goto out_sleep;
	}

	k_mutex_unlock(&data->lock);

	/* Wait for duration */
	k_sleep(K_SECONDS(duration));

	/* Stop CW */
	k_mutex_lock(&data->lock, K_FOREVER);
	pan30xx_set_standby(dev, PAN30XX_STANDBY_RC);

out_sleep:
	pan30xx_set_sleep(dev);
out_unlock:
	k_mutex_unlock(&data->lock);
	atomic_set(&data->state, PAN30XX_REST_STATE);

	return ret;
}

static DEVICE_API(lora, pan30xx_lora_api) = {
	.config = pan30xx_lora_config,
	.send = pan30xx_lora_send,
	.send_async = pan30xx_lora_send_async,
	.recv = pan30xx_lora_recv,
	.recv_async = pan30xx_lora_recv_async,
	.recv_duty_cycle = pan30xx_lora_recv_duty_cycle,
	.recv_duty_cycle_async = pan30xx_lora_recv_duty_cycle_async,
	.airtime = pan30xx_lora_airtime,
	.test_cw = pan30xx_lora_test_cw,
};

static int pan30xx_init(const struct device *dev)
{
	struct pan30xx_data *data = dev->data;
	int ret;

	/* Initialize data structures */
	k_mutex_init(&data->lock);
	k_msgq_init(&data->tx_msgq, (char *)&data->tx_result,
		    sizeof(struct pan30xx_tx_result), 1);
	k_msgq_init(&data->rx_msgq, (char *)&data->rx_result,
		    sizeof(struct pan30xx_rx_result), 1);
	k_work_init(&data->irq_work, pan30xx_irq_work_handler);
	data->dev = dev;
	atomic_set(&data->state, PAN30XX_STATE_IDLE);
	data->config_valid = false;

	/* Initialize HAL */
	ret = pan30xx_hal_init(dev);
	if (ret < 0) {
		LOG_ERR("HAL init failed: %d", ret);
		return ret;
	}

	/* Setup interrupt callback */
	ret = pan30xx_hal_set_irq_callback(dev, pan30xx_irq_callback);
	if (ret < 0) {
		LOG_ERR("IRQ callback setup failed: %d", ret);
		return ret;
	}

	/* Initialize chip */
	ret = pan30xx_chip_init(dev);
	if (ret < 0) {
		LOG_ERR("Chip init failed: %d", ret);
		return ret;
	}

	/*
	 * Place the radio into sleep mode upon boot.
	 * The required lora_config call before transmission or reception
	 * will wake the radio. It is automatically placed back into sleep
	 * mode upon TX or RX completion.
	 */
	if (IS_ENABLED(CONFIG_LORA_PAN30XX_NATIVE_SLEEP)) {
		ret = pan30xx_set_sleep(dev);
		if (ret < 0) {
			LOG_ERR("Initial sleep failed: %d", ret);
			return ret;
		}
	}

	return 0;
}

#define PAN30XX_INIT(inst, _variant)								\
	static struct pan30xx_data pan30xx_data_##inst;						\
												\
	static const struct pan30xx_hal_config pan30xx_config_##inst = {				\
		.variant = _variant,								\
		.spi = SPI_DT_SPEC_INST_GET(inst,						\
					    SPI_WORD_SET(8) | SPI_TRANSFER_MSB),		\
		.reset = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),				\
		.busy = GPIO_DT_SPEC_INST_GET(inst, busy_gpios),				\
		.irq = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),					\
		.lf_clk = DT_INST_ENUM_IDX(inst, lf_clk),					\
		.tcxo_voltage = DT_INST_ENUM_IDX(inst, tcxo_voltage),				\
		.tcxo_startup_delay_ms = DT_INST_PROP(inst, tcxo_power_startup_delay_ms),	\
		.rfsw.enable = DT_INST_PROP(inst, rfsw_enable),					\
		.rfsw.standby = DT_INST_PROP(inst, rfsw_standby),				\
		.rfsw.rx = DT_INST_PROP(inst, rfsw_rx),						\
		.rfsw.tx = DT_INST_PROP(inst, rfsw_tx),						\
		.rfsw.tx_hp = DT_INST_PROP(inst, rfsw_tx_hp),					\
		.rfsw.tx_hf = DT_INST_PROP(inst, rfsw_tx_hf),					\
		.rfsw.wifi = DT_INST_PROP(inst, rfsw_wifi),					\
		.rfsw.gnss = DT_INST_PROP(inst, rfsw_gnss),					\
		.use_dcdc = DT_INST_PROP(inst, use_dcdc),					\
		.force_ldro = DT_INST_PROP(inst, force_ldro),					\
		.rx_boosted = DT_INST_PROP(inst, rx_boosted),					\
	};											\
												\
	DEVICE_DT_INST_DEFINE(inst, pan30xx_init, NULL,						\
			      &pan30xx_data_##inst, &pan30xx_config_##inst,			\
			      POST_KERNEL, CONFIG_LORA_INIT_PRIORITY,				\
			      &pan30xx_lora_api);

#define DT_DRV_COMPAT semtech_lr1121
DT_INST_FOREACH_STATUS_OKAY_VARGS(PAN30XX_INIT, PAN30XX_LR1121)
#undef DT_DRV_COMPAT
