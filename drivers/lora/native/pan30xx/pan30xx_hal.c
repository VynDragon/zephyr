/*
 * Copyright (c) 2026 MASSDRIVER EI (massdriver.space)
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>

#include "pan30xx.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pan30xx_hal, CONFIG_LORA_LOG_LEVEL);

#define PAN30XX_RESET_PULSE_MS		1
#define PAN30XX_RESET_WAIT_MS		1

static inline struct pan30xx_hal_data *get_hal_data(const struct device *dev)
{
	struct pan30xx_data *data = dev->data;

	return &data->hal;
}

#if (DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(panchip_pan3029, i2c) || \
	DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(panchip_pan3060, i2c)

static int pan30xx_hal_i2c_ready(const struct device *dev)
{
	const struct pan30xx_hal_config *config = dev->config;

	return i2c_is_ready_dt(&config->transport.bus.i2c);
}

static const char *pan30xx_hal_i2c_name(const struct device *dev)
{
	const struct pan30xx_hal_config *config = dev->config;

	return config->transport.bus.i2c.bus->name;
}

static const char *pan30xx_hal_i2c_transceive(const struct device *dev,
					      const uint8_t *tx_buf, size_t tx_len,
					      uint8_t *rx_buf, size_t rx_len)
{
	const struct pan30xx_hal_config *config = dev->config;

	if (tx_len == 0U && rx_len == 0U) {
		return 0;
	}

	if (rx_len == 0U || rx_buf == 0) {
		return i2c_write_dt(&config->transport.bus.i2c.bus, tx_buf, tx_len);
	}

	if (tx_len == 0U || tx_buf == 0) {
		return i2c_read_dt(&config->transport.bus.i2c.bus, rx_buf, rx_len);
	}

	return i2c_write_read_dt(&config->transport.bus.i2c.bus, tx_buf, tx_len, rx_buf, rx_len);
}

static uint8_t pan30xx_hal_i2c_addr(uint8_t addr, bool writing)
{
	return (addr << 1U) | (writing ? 0x0 : 0x01);
}

#endif

#if (DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(panchip_pan3029, spi) || \
	DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(panchip_pan3060, spi)

static int pan30xx_hal_spi_ready(const struct device *dev)
{
	const struct pan30xx_hal_config *config = dev->config;

	return spi_is_ready_dt(&config->transport.bus.spi);
}

static const char *pan30xx_hal_spi_name(const struct device *dev)
{
	const struct pan30xx_hal_config *config = dev->config;

	return config->transport.bus.spi.bus->name;
}

static const char *pan30xx_hal_spi_transceive(const struct device *dev,
					      const uint8_t *tx_buf, size_t tx_len,
					      uint8_t *rx_buf, size_t rx_len)
{
	const struct pan30xx_hal_config *config = dev->config;
	struct spi_buf tx_bufs[] = {
		{ .buf = (uint8_t *)tx_buf, .len = tx_len },
	};
	struct spi_buf rx_bufs[] = {
		{ .buf = rx_buf, .len = rx_len },
		{ .buf = rx_buf, .len = rx_len },
	};
	struct spi_buf_set tx_set = { .buffers = tx_bufs, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = rx_bufs, .count = 1 };

	if (tx_len == 0U && rx_len == 0U) {
		return 0;
	}

	if (rx_len == 0U || rx_buf == 0) {
		return spi_write_dt(&config->transport.bus.spi.bus, &tx_set);
	}

	if (tx_len == 0U || tx_buf == 0) {
		return spi_read_dt(&config->transport.bus.spi.bus, &rx_set);
	}

	__ASSERT_NO_MSG(tx_len <= rx_len);

	rx_bufs[0].len = tx_len;
	rx_set.count = 2;

	return spi_transceive_dt(&config->transport.bus.spi.bus, &tx_set, &rx_set);
}

static uint8_t pan30xx_hal_spi_addr(uint8_t addr, bool writing)
{
	return (addr << 1U) | (writing ? 0x01 : 0x0);
}

#endif

static int pan30xx_hal_set_page(const struct device *dev, uint8_t page)
{
	const struct pan30xx_hal_config *config = dev->config;
	const uint8_t buf[2] = { config->transport.addr(PAN30XX_SYS, true), page };

	return config->transport.transceive(dev, buf, 2, NULL, 0);
}

int pan30xx_hal_read_register(const struct device *dev, const uint8_t page, const uint8_t addr,
			      uint8_t *value)
{
	const struct pan30xx_hal_config *config = dev->config;
	struct pan30xx_hal_data *data = get_hal_data(dev);
	uint8_t tx_buf = config->transport.addr(addr, false);

	if (data->current_page != page) {
		ret = pan30xx_hal_set_page(dev, page);
		if (ret != 0) {
			LOG_ERR("Failed to set page: %d", ret);
			return ret;
		}
	}

	return config->transport.transceive(dev, &tx_buf, 1, value, 1);
}

int pan30xx_hal_write_register(const struct device *dev, const struct pan30xx_register_value value)
{
	const struct pan30xx_hal_config *config = dev->config;
	struct pan30xx_hal_data *data = get_hal_data(dev);
	uint8_t buf[2] = { config->transport.addr(value.addr, true), value.value };
	int ret;

	if (data->current_page != value.page) {
		ret = pan30xx_hal_set_page(dev, value.page);
		if (ret != 0) {
			LOG_ERR("Failed to set page: %d", ret);
			return ret;
		}
	}

	if (value.mask != 0xff) {
		ret = pan30xx_hal_read_register(dev, value.page, value.addr, &buf[1]);
		if (ret != 0) {
			LOG_ERR("Failed to read before modify: %d", ret);
			return ret;
		}
		buf[1] &= ~value.mask;
		buf[1] |= value.value & value.mask;
	}

	return config->transport.transceive(dev, buf, 2, NULL, 0);
}

int pan30xx_hal_reset(const struct device *dev)
{
	const struct pan30xx_hal_config *config = dev->config;
	int ret;
	uint8_t reset_buf[2] = { config->transport.addr(PAN30XX_POW, true) , PAN30XX_POW_INIT };

	if (config->reset.port != NULL) {
		if (!gpio_is_ready_dt(&config->reset)) {
			LOG_ERR("Reset GPIO not ready");
			return -ENODEV;
		}

		/* Pull reset low */
		ret = gpio_pin_set_dt(&config->reset, 1);
		if (ret < 0) {
			LOG_ERR("Failed to assert reset: %d", ret);
			return ret;
		}

		k_msleep(PAN30XX_RESET_PULSE_MS);

		/* Release reset */
		ret = gpio_pin_set_dt(&config->reset, 0);
		if (ret < 0) {
			LOG_ERR("Failed to release reset: %d", ret);
			return ret;
		}

		k_msleep(PAN30XX_RESET_WAIT_MS);
	}

	ret = config->transport.transceive(dev, reset_buf, 2, NULL, 0);
	if (ret < 0) {
		LOG_ERR("Failed to do soft reset: %d", ret);
		return ret;
	}

	LOG_DBG("Reset complete");
	return 0;
}

static void irq_isr(const struct device *gpio, struct gpio_callback *cb,
		    uint32_t pins)
{
	struct pan30xx_hal_data *data = CONTAINER_OF(cb, struct pan30xx_hal_data, irq_cb);

	if (data->irq_callback != NULL) {
		data->irq_callback(data->dev);
	}
}

int pan30xx_hal_set_irq_callback(const struct device *dev,
				void (*callback)(const struct device *dev))
{
	const struct pan30xx_hal_config *config = dev->config;
	struct pan30xx_hal_data *data = get_hal_data(dev);
	int ret;

	data->irq_callback = callback;

	if (callback != NULL) {
		ret = gpio_pin_interrupt_configure_dt(&config->irq,
						      GPIO_INT_EDGE_TO_ACTIVE);
	} else {
		ret = gpio_pin_interrupt_configure_dt(&config->irq,
						      GPIO_INT_DISABLE);
	}

	return ret;
}

void pan30xx_hal_irq_enable(const struct device *dev)
{
	/* GPIO interrupts auto re-arm after being serviced, no action needed */
	ARG_UNUSED(dev);
}

int pan30xx_hal_init(const struct device *dev)
{
	const struct pan30xx_hal_config *config = dev->config;
	struct pan30xx_hal_data *data = get_hal_data(dev);
	int ret;

	/* Store device reference for callbacks */
	data->dev = dev;
	data->irq_callback = NULL;

	/* Check bus */
	if (!config->transport.ready(dev)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/* Configure reset GPIO */
	if (config->reset.port != NULL) {
		if (!gpio_is_ready_dt(&config->reset)) {
			LOG_ERR("Reset GPIO not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to configure reset GPIO: %d", ret);
			return ret;
		}
	}

	/* Configure IRQ GPIO with interrupt */
	if (!gpio_is_ready_dt(&config->irq)) {
		LOG_ERR("IRQ GPIO not ready");
		return -ENODEV;
	}
	ret = gpio_pin_configure_dt(&config->irq, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure IRQ GPIO: %d", ret);
		return ret;
	}

	/* Setup IRQ GPIO interrupt callback */
	gpio_init_callback(&data->irq_cb, irq_isr, BIT(config->irq.pin));
	ret = gpio_add_callback(config->irq.port, &data->irq_cb);
	if (ret < 0) {
		LOG_ERR("Failed to add IRQ GPIO callback: %d", ret);
		return ret;
	}

	LOG_DBG("HAL initialized");
	return 0;
}

int pan30xx_hal_write_buffer(const struct device *dev, const uint8_t *data, size_t len)
{
	__ASSERT_NO_MSG(len <= PAN30XX_MAX_PAYLOAD_LEN);
	uint8_t buf[PAN30XX_FIFO_SIZE];
	int ret;

	buf[0] = config->transport.addr(PAN30XX_FIFO, true);
	memcpy(&buf[1], data, len);

	return config->transport.transceive(dev, buf, len + 1, NULL, 0);
}

int pan30xx_hal_read_buffer(const struct device *dev, uint8_t offset, uint8_t *data, size_t len)
{
	__ASSERT_NO_MSG(len <= PAN30XX_MAX_PAYLOAD_LEN);
	uint8_t buf = config->transport.addr(PAN30XX_FIFO, false);

	return config->transport.transceive(dev, &buf, 1, data, len);
}
