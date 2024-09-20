/*
 * Copyright (c) 2024 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_bl61x_gpio

#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/pinctrl/bflb-common-pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/bl61x-pinctrl.h>
#include <zephyr/irq.h>

#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_bflb_bl61x);

#include <zephyr/drivers/gpio/gpio_utils.h>


/* This driver is limited by zephyr masks and supports 32 pins. BL61x serie supports 35 pins. */

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


static void system_clock_settle(void)
{
	__asm__ volatile (".rept 15 ; nop ; .endr");
}



static int gpio_bflb_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_bflb_config * const cfg = dev->config;

	*value = sys_read32(cfg->base_reg + GLB_GPIO_CFG128_OFFSET);

	return 0;
}


static int gpio_bflb_port_set_masked_raw(const struct device *dev,
					uint32_t mask,
					uint32_t value)
{
	const struct gpio_bflb_config * const cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_CFG136_OFFSET);
	tmpVal = (tmpVal & ~mask) | (mask & value);
	sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_CFG136_OFFSET);

	return 0;
}

static int gpio_bflb_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
	const struct gpio_bflb_config * const cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_CFG136_OFFSET);
	tmpVal = tmpVal | mask;
	sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_CFG136_OFFSET);

	return 0;
}

static int gpio_bflb_port_clear_bits_raw(const struct device *dev, uint32_t mask)
{
	const struct gpio_bflb_config * const cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_CFG136_OFFSET);
	tmpVal = tmpVal & ~mask;
	sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_CFG136_OFFSET);

	return 0;
}

static int gpio_bflb_port_toggle_bits(const struct device *dev, uint32_t mask)
{
	const struct gpio_bflb_config * const cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_CFG136_OFFSET);
	tmpVal ^= mask;
	sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_CFG136_OFFSET);

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

	tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_CFG0_OFFSET + (pin << 2));
	/* clear modes */
	tmpVal &= GLB_REG_GPIO_0_INT_MODE_SET_UMSK;

	if ((trig & GPIO_INT_HIGH_1) != 0
		&& (trig & GPIO_INT_LOW_0) != 0
		&& (mode & GPIO_INT_EDGE)) {
		trig_mode |= 4;
	} else if ((trig & GPIO_INT_HIGH_1) != 0) {
		trig_mode |= 1;
	}

	if (!(mode & GPIO_INT_EDGE)) {
		trig_mode |= 2;
	}
	tmpVal |= (trig_mode << GLB_REG_GPIO_0_INT_MODE_SET_POS);
	sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_CFG0_OFFSET + (pin << 2));
}

static void gpio_bflb_pin_interrupt_clear(const struct device *dev, uint32_t mask)
{
	const struct gpio_bflb_config * const cfg = dev->config;
	uint32_t tmpVal = 0;


	for(int i = 0; i < 32; i++) {
		if (((mask >> i) & 0x1) != 0) {
			tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_CFG0_OFFSET + (i << 2));
			tmpVal |= GLB_REG_GPIO_0_INT_CLR_MSK;
			sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_CFG0_OFFSET + (i << 2));
			system_clock_settle();
			tmpVal &= GLB_REG_GPIO_0_INT_CLR_UMSK;
			sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_CFG0_OFFSET + (i << 2));
		}
	}
}

static int gpio_bflb_pin_interrupt_configure(const struct device *dev,
					     gpio_pin_t pin,
					     enum gpio_int_mode mode,
					     enum gpio_int_trig trig)
{
	const struct gpio_bflb_config * const cfg = dev->config;
	uint32_t tmpVal = 0;

	/* Disable the interrupt. */
	tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_CFG0_OFFSET + (pin << 2));
	tmpVal |= GLB_REG_GPIO_0_INT_MASK_MSK;
	sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_CFG0_OFFSET + (pin << 2));


	gpio_bflb_port_interrupt_configure_mode(dev, pin, mode, trig);


	if (mode != GPIO_INT_MODE_DISABLED) {
		/* clear */
		gpio_bflb_pin_interrupt_clear(dev, BIT(pin));
		/* unmask */
		tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_CFG0_OFFSET + (pin << 2));
		tmpVal &= GLB_REG_GPIO_0_INT_MASK_UMSK;
		sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_CFG0_OFFSET + (pin << 2));
	} else {
		/* mask */
		tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_CFG0_OFFSET + (pin << 2));
		tmpVal |= GLB_REG_GPIO_0_INT_MASK_MSK;
		sys_write32(tmpVal, cfg->base_reg + GLB_GPIO_CFG0_OFFSET + (pin << 2));
	}
	/* enable clic interrupt for us as it gets cleared in soc init*/
	cfg->irq_enable_func(dev);
	return 0;
}

/* basically pinctrl function of identical job */
static int gpio_bflb_config(const struct device *dev, gpio_pin_t pin,
			   gpio_flags_t flags)
{
	const struct gpio_bflb_config * const conf = dev->config;
	uint32_t cfg = 0;
	uint32_t tmpVal = 0;

	/* gpio pad check goes here */

	/* disable RC32K muxing */
	if (pin == 16) {
		*(volatile uint32_t *)(0x2000f000 + 0x38) &= ~(1 << 20);
	} else if (pin == 17) {
		*(volatile uint32_t *)(0x2000f000 + 0x38) &= ~(1 << 21);
	}

	cfg = sys_read32(conf->base_reg + GLB_GPIO_CFG0_OFFSET + (pin << 2));

	if ((flags & GPIO_INPUT) != 0) {
		cfg |= GLB_REG_GPIO_0_IE_MSK;
		cfg &= GLB_REG_GPIO_0_OE_UMSK;
	} else if ((flags & GPIO_OUTPUT) != 0) {
		cfg &= GLB_REG_GPIO_0_IE_UMSK;
		cfg |= GLB_REG_GPIO_0_OE_MSK;
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			tmpVal = sys_read32(conf->base_reg + GLB_GPIO_CFG136_OFFSET);
			tmpVal |= 1 << pin;
			sys_write32(tmpVal, conf->base_reg + GLB_GPIO_CFG136_OFFSET);
		}
		if (flags & GPIO_OUTPUT_INIT_LOW) {
			tmpVal = sys_read32(conf->base_reg + GLB_GPIO_CFG136_OFFSET);
			tmpVal &= ~(1 << pin);
			sys_write32(tmpVal, conf->base_reg + GLB_GPIO_CFG136_OFFSET);
		}
	}

	if ((flags & GPIO_PULL_UP) != 0) {
		cfg &= GLB_REG_GPIO_0_PD_UMSK;
		cfg |= GLB_REG_GPIO_0_PU_MSK;
	} else if ((flags & GPIO_PULL_DOWN) != 0) {
		cfg |= GLB_REG_GPIO_0_PD_MSK;
		cfg &= GLB_REG_GPIO_0_PU_UMSK;
	} else {
		cfg &= GLB_REG_GPIO_0_PD_UMSK;
		cfg &= GLB_REG_GPIO_0_PU_UMSK;
	}

	/* SMT is enable for GPIO */
	cfg |= GLB_REG_GPIO_0_SMT_MSK;

	/* drive is 1 */
	cfg &= GLB_REG_GPIO_0_DRV_UMSK;
	cfg |= (1 << GLB_REG_GPIO_0_DRV_POS);
	/* function is 11 = gpio */
	cfg &= GLB_REG_GPIO_0_FUNC_SEL_UMSK;
	cfg |= (11 << GLB_REG_GPIO_0_FUNC_SEL_POS);

	/* output is controlled by value of _o*/
	cfg &= GLB_REG_GPIO_0_MODE_UMSK;

	sys_write32(cfg, conf->base_reg + GLB_GPIO_CFG0_OFFSET + (pin << 2));
	return 0;
}

int gpio_bflb_init(const struct device *dev)
{
	const struct gpio_bflb_config * const cfg = dev->config;

	cfg->irq_config_func(dev);

	return 0;
}

static void gpio_bflb_isr(const struct device *dev)
{
	const struct gpio_bflb_config * const cfg = dev->config;
	struct gpio_bflb_data *data = dev->data;
	uint32_t int_stat = 0;
	uint32_t tmpVal = 0;

	for(int i = 0; i < 32; i++) {
		tmpVal = sys_read32(cfg->base_reg + GLB_GPIO_CFG0_OFFSET + (i << 2));
		int_stat |= ((tmpVal & GLB_GPIO_0_INT_STAT_MSK) != 0 ? 1 : 0) << i;
	}

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
