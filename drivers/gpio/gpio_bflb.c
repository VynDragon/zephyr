/*
 * Copyright (c) 2024 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_bl_gpio

#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/pinctrl/bflb-common-pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/bl60x-pinctrl.h>
#include <zephyr/irq.h>

#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_bflb);

#include <zephyr/drivers/gpio/gpio_utils.h>

/* this driver supports only 32 GPIO, which happens to be the maximum on BL6 and 7 serie.
 * There is the ability to have multiple sets, a facility to manage this would need added
 * if a bflb mcu comes out with more GPIO
 * but still it will work for the first 32 GPIOs
 */

struct gpio_bflb_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	uint32_t base_reg;
	void (*irq_config_func)(const struct device *dev);
	void (*irq_enable_func)(const struct device *dev);
};

struct gpio_bflb_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	sys_slist_t callbacks;
};


static int gpio_bflb_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_bflb_config * const cfg = dev->config;

	*value = sys_read32(cfg->base_reg + GLB_GPIO_CFGCTL30_OFFSET);

	return 0;
}


static int gpio_bflb_port_set_masked_raw(const struct device *dev,
					uint32_t mask,
					uint32_t value)
{
	const struct gpio_bflb_config * const cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_CFGCTL32_OFFSET);
	tmpVal = (tmpVal & ~mask) | (mask & value);
	sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_CFGCTL32_OFFSET);

	return 0;
}

static int gpio_bflb_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
	const struct gpio_bflb_config * const cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_CFGCTL32_OFFSET);
	tmpVal = tmpVal | mask;
	sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_CFGCTL32_OFFSET);

	return 0;
}

static int gpio_bflb_port_clear_bits_raw(const struct device *dev, uint32_t mask)
{
	const struct gpio_bflb_config * const cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_CFGCTL32_OFFSET);
	tmpVal = tmpVal & ~mask;
	sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_CFGCTL32_OFFSET);

	return 0;
}

static int gpio_bflb_port_toggle_bits(const struct device *dev, uint32_t mask)
{
	const struct gpio_bflb_config * const cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_CFGCTL32_OFFSET);
	tmpVal ^= mask;
	sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_CFGCTL32_OFFSET);

	return 0;
}

static void gpio_bflb_port_interrupt_configure_mode(const struct device *dev, uint32_t pin,
					     enum gpio_int_mode mode,
					     enum gpio_int_trig trig)
{
	const struct gpio_bflb_config * const cfg = dev->config;
	uint32_t tmpVal = 0;
	/* default to 'sync' mode */
	uint8_t trig_mode = 0;

	tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_INT_MODE_SET1_OFFSET + ((pin / 10) << 2));
	/* clear modes */
	tmpVal &= ~(0x07 << ((pin % 10) * 3));

	if ((trig & GPIO_INT_HIGH_1) != 0) {
		trig_mode |= 1;
	}

	if (!(mode & GPIO_INT_EDGE)) {
		trig_mode |= 2;
	}
	tmpVal |= (trig_mode << ((pin % 10) * 3));
	sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_INT_MODE_SET1_OFFSET + ((pin / 10) << 2));
}

static void gpio_bflb_pin_interrupt_clear(const struct device *dev, uint32_t mask)
{
	const struct gpio_bflb_config * const cfg = dev->config;

	sys_write32(mask, cfg->base_reg + GLB_GPIO_INT_CLR1_OFFSET);

	while ((sys_read32(cfg->base_reg + GLB_GPIO_INT_STAT1_OFFSET) & mask) != 0) {
		__asm__ volatile ("nop");
	}
	sys_write32(0x0, cfg->base_reg + GLB_GPIO_INT_CLR1_OFFSET);
}

static int gpio_bflb_pin_interrupt_configure(const struct device *dev,
					     gpio_pin_t pin,
					     enum gpio_int_mode mode,
					     enum gpio_int_trig trig)
{
	const struct gpio_bflb_config * const cfg = dev->config;
	uint32_t tmpVal = 0;

	/* Disable the interrupt. */
	tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_INT_MASK1_OFFSET);
	tmpVal |= BIT(pin);
	sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_INT_MASK1_OFFSET);


	gpio_bflb_port_interrupt_configure_mode(dev, pin, mode, trig);


	if (mode != GPIO_INT_MODE_DISABLED) {
		/* clear */
		gpio_bflb_pin_interrupt_clear(dev, BIT(pin));
		/* unmask */
		tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_INT_MASK1_OFFSET);
		tmpVal &= ~BIT(pin);
		sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_INT_MASK1_OFFSET);
	} else {
		/* mask */
		tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_INT_MASK1_OFFSET);
		tmpVal |= BIT(pin);
		sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_INT_MASK1_OFFSET);
	}
	/* enable clic interrupt for us as it gets cleared in soc init*/
	cfg->irq_enable_func(dev);
	return 0;
}

/* basically pinctrl function of identical job */
static int gpio_bflb_config(const struct device *dev, gpio_pin_t pin,
			   gpio_flags_t flags)
{
	const struct gpio_bflb_config * const cfg = dev->config;
	uint8_t is_odd = 0;
	uint32_t cfg_address;
	uint32_t tmpVal = 0;
	uint32_t tmpVal_a = 0;
	uint32_t tmpVal_b = 0;


	/* Disable output anyway */
	tmpVal_a = sys_read32(cfg->base_reg + GLB_GPIO_CFGCTL34_OFFSET + ((pin >> 5) << 2));
	tmpVal_a &= ~(1 << (pin & 0x1f));
	sys_write32(tmpVal_a, cfg->base_reg + GLB_GPIO_CFGCTL34_OFFSET + ((pin >> 5) << 2));


#ifdef CONFIG_SOC_SERIES_BL70X
	is_odd = pin & 1;
	cfg_address = cfg->base_reg + GLB_GPIO_CFGCTL0_OFFSET + (pin / 2 * 4);
	if (pin >= 23 && pin <= 28) {
		if ((flags & GPIO_INPUT) != 0) {
			LOG_ERR("BL70x pins 23 to 28 are not capable of input");
			return -EINVAL;
		}
		if (sys_read32(GLB_BASE + GLB_GPIO_USE_PSRAM__IO_OFFSET) & (1 << (pin - 23))) {
			cfg_address = cfg->base_reg + GLB_GPIO_CFGCTL0_OFFSET + ((pin + 9) / 2 * 4);
			is_odd = (pin + 9) & 1;
		}
	}
#else
	is_odd = pin & 1;
	cfg_address = cfg->base_reg + GLB_GPIO_CFGCTL0_OFFSET + (pin / 2 * 4);
#endif
	tmpVal_b = sys_read32(cfg_address);
	/*cfg &= ~(0xffff << (16 * is_odd));*/

	tmpVal_a = sys_read32(cfg->base_reg + GLB_GPIO_CFGCTL34_OFFSET + ((pin >> 5) << 2));


	if ((flags & GPIO_INPUT) != 0) {
		tmpVal_b |= (1 << (is_odd * 16 + 0));
		tmpVal_a &= ~(1 << (pin & 0x1f));
	} else {
		tmpVal_b &= ~(1 << (is_odd * 16 + 0));
	}

	if ((flags & GPIO_OUTPUT) != 0) {
		tmpVal_a |= (1 << (pin & 0x1f));
		tmpVal_b &= ~(1 << (is_odd * 16 + 0));
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_CFGCTL32_OFFSET);
			tmpVal = tmpVal | pin;
			sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_CFGCTL32_OFFSET);
		}
		if (flags & GPIO_OUTPUT_INIT_LOW) {
			tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_CFGCTL32_OFFSET);
			tmpVal = tmpVal & ~pin;
			sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_CFGCTL32_OFFSET);
		}
	} else {
		tmpVal_a &= ~(1 << (pin & 0x1f));
	}


	sys_write32(tmpVal_a, cfg->base_reg + GLB_GPIO_CFGCTL34_OFFSET + ((pin >> 5) << 2));

	if ((flags & GPIO_PULL_UP) != 0) {
		tmpVal_b |= (1 << (is_odd * 16 + 4));
		tmpVal_b &= ~(1 << (is_odd * 16 + 5));
	} else if ((flags & GPIO_PULL_DOWN) != 0) {
		tmpVal_b |= (1 << (is_odd * 16 + 5));
		tmpVal_b &= ~(1 << (is_odd * 16 + 4));
	} else {
		tmpVal_b &= ~(1 << (is_odd * 16 + 4));
		tmpVal_b &= ~(1 << (is_odd * 16 + 5));
	}

	/* GPIO mode */
#ifdef CONFIG_SOC_SERIES_BL70X
	/* but function goes in the right place */
	if (pin >= 23 && pin <= 28) {
		tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_CFGCTL0_OFFSET + (pin / 2 * 4));
		tmpVal &= ~(0x1f << ((pin & 1) * 16 + 8));
		tmpVal |= (11 << ((pin & 1) * 16 + 8));
		sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_CFGCTL0_OFFSET + (pin / 2 * 4));
	} else {
		tmpVal_b &= ~(0x1f << (is_odd * 16 + 8));
		tmpVal_b |= (11 << (is_odd * 16 + 8));
	}
#else
	tmpVal_b &= ~(0x1f << (is_odd * 16 + 8));
	tmpVal_b |= (11 << (is_odd * 16 + 8));
#endif
	/* enabled SMT in GPIO mode */
	tmpVal_b |= (1 << (is_odd * 16 + 1));

	sys_write32(tmpVal_b, cfg_address);

	return 0;
}

int gpio_bflb_init(const struct device *dev)
{
	const struct gpio_bflb_config * const cfg = dev->config;

	/* nothing to do beside link irq */

	cfg->irq_config_func(dev);

	return 0;
}

static void gpio_bflb_isr(const struct device *dev)
{
	const struct gpio_bflb_config * const cfg = dev->config;
	struct gpio_bflb_data *data = dev->data;
	uint32_t int_stat;

	/* interrupt data is in format 1 bit = 1 pin */
	int_stat = sys_read32(cfg->base_reg + GLB_GPIO_INT_STAT1_OFFSET);

	gpio_fire_callbacks(&data->callbacks, dev, int_stat);
	/* clear interrupts */
	gpio_bflb_pin_interrupt_clear(dev, int_stat);
}

static int gpio_bflb_manage_callback(const struct device *port,
				    struct gpio_callback *callback,
				    bool set)
{
	struct gpio_bflb_data *data = port->data;

	return gpio_manage_callback(&(data->callbacks), callback, set);
}

static const struct gpio_driver_api gpio_bflb_api = {
	.pin_configure = gpio_bflb_config,
	.port_get_raw = gpio_bflb_port_get_raw,
	.port_set_masked_raw = gpio_bflb_port_set_masked_raw,
	.port_set_bits_raw = gpio_bflb_port_set_bits_raw,
	.port_clear_bits_raw = gpio_bflb_port_clear_bits_raw,
	.port_toggle_bits = gpio_bflb_port_toggle_bits,
	.pin_interrupt_configure = gpio_bflb_pin_interrupt_configure,
	.manage_callback = gpio_bflb_manage_callback,
};

#define GPIO_BFLB_INIT(n)							\
	static void port_##n##_bflb_irq_config_func(const struct device *dev);	\
	static void port_##n##_bflb_irq_enable_func(const struct device *dev);	\
										\
	static const struct gpio_bflb_config port_##n##_bflb_config = {		\
		.common = {							\
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),	\
		},								\
		.base_reg = DT_INST_REG_ADDR(n),				\
		.irq_config_func = port_##n##_bflb_irq_config_func,		\
		.irq_enable_func = port_##n##_bflb_irq_enable_func,		\
	};									\
										\
	static struct gpio_bflb_data port_##n##_bflb_data;			\
										\
	DEVICE_DT_INST_DEFINE(n, gpio_bflb_init, NULL,				\
			    &port_##n##_bflb_data,				\
			    &port_##n##_bflb_config, PRE_KERNEL_1,		\
			    CONFIG_GPIO_INIT_PRIORITY,				\
			    &gpio_bflb_api);					\
										\
	static void port_##n##_bflb_irq_config_func(const struct device *dev)	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),		\
			    gpio_bflb_isr,					\
			    DEVICE_DT_INST_GET(n), 0);				\
	}									\
	static void port_##n##_bflb_irq_enable_func(const struct device *dev)	\
	{									\
		irq_enable(DT_INST_IRQN(n));					\
	}

DT_INST_FOREACH_STATUS_OKAY(GPIO_BFLB_INIT)
