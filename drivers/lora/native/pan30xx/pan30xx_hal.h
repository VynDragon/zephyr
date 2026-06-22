/*
 * Copyright (c) 2026 MASSDRIVER EI (massdriver.space)
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_LORA_PAN30XX_PAN30XX_HAL_H_
#define ZEPHYR_DRIVERS_LORA_PAN30XX_PAN30XX_HAL_H_

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

enum pan30xx_variants {
	PAN30XX_INVALID = 0,
	PAN30XX_PAN3029 = 0x1,
	PAN30XX_PAN3060 = 0x2,
};

struct pan30xx_hal_gpios_config {
	uint8_t enable;
	uint8_t standby;
	uint8_t rx;
	uint8_t tx;
};

typedef bool (*pan30xx_hal_bus_ready_fn)(const struct device *dev);
typedef int (*pan30xx_hal_bus_transceive_fn)(const struct device *dev,
					     const uint8_t *tx_buf, size_t tx_len,
					     uint8_t *rx_buf, size_t rx_len);
typedef const char *(*pan30xx_hal_bus_name_fn)(const struct device *dev);
typedef uint8_t (*pan30xx_hal_bus_addr_fn)(uint8_t addr, bool writing);

union pan30xx_hal_bus {
	struct i2c_dt_spec i2c;
	struct spi_dt_spec spi;
};

struct pan30xx_hal_transport {
	union pan30xx_hal_bus bus;
	pan30xx_hal_bus_ready_fn ready;
	pan30xx_hal_bus_transceive_fn transceive;
	pan30xx_hal_bus_name_fn name;
	pan30xx_hal_bus_addr_fn addr;
};

struct pan30xx_hal_config {
	enum pan30xx_variants variant;
	struct pan30xx_hal_transport transport;
	struct gpio_dt_spec reset;
	struct gpio_dt_spec irq;
	uint16_t tcxo_startup_delay_ms;
	struct pan30xx_hal_gpios_config gpios;
	bool use_dcdc;
	bool force_ldro;
	bool has_reset;
};

struct pan30xx_hal_data {
	struct gpio_callback irq_cb;
	void (*irq_callback)(const struct device *dev);
	const struct device *dev;
	uint8_t current_page;
};

struct pan30xx_register_value {
	uint8_t page;
	uint8_t addr;
	uint8_t value;
/* If not 0xff, do a read-modify-write */
	uint8_t mask;
};

int pan30xx_hal_init(const struct device *dev);

int pan30xx_hal_reset(const struct device *dev);

int pan30xx_hal_write_register(const struct device *dev, const struct pan30xx_register_value value);

int pan30xx_hal_read_register(const struct device *dev, const uint8_t page, const uint8_t addr,
			      uint8_t *value);

int pan30xx_hal_write_fifo(const struct device *dev, const uint8_t *data, size_t len);

int pan30xx_hal_read_fifo(const struct device *dev, uint8_t offset, uint8_t *data, size_t len);

int pan30xx_hal_set_irq_callback(const struct device *dev,
				 void (*callback)(const struct device *dev));

void pan30xx_hal_irq_enable(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_LORA_PAN30XX_PAN30XX_HAL_H_ */
