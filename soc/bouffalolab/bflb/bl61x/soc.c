/*
 * Copyright (c) 2024, MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Bouffalo Lab RISC-V MCU series initialization code
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/eeprom.h>

#include <soc.h>
#include <clic.h>

/* brown out detection */
void bl61x_BOD_init(void)
{
	uint32_t tmpVal = 0;

	/* disable BOD interrupt */
	tmpVal = sys_read32(HBN_BASE + HBN_IRQ_MODE_OFFSET);
	tmpVal &= ~HBN_IRQ_BOR_EN_MSK;
	sys_write32(tmpVal, HBN_BASE + HBN_IRQ_MODE_OFFSET);

	tmpVal = sys_read32(HBN_BASE + HBN_BOR_CFG_OFFSET);
	/* when brownout thresold, restart*/
	tmpVal |= HBN_BOD_SEL_MSK;
	/* set BOD thresold:
	 * 0:2.05v,1:2.10v,2:2.15v....7:2.4v
	 */
	tmpVal &= ~HBN_BOD_VTH_MSK;
	tmpVal |= (7 << HBN_BOD_VTH_POS);
	/* enable BOD */
	tmpVal |= HBN_PU_BOD_MSK;
	sys_write32(tmpVal, HBN_BASE + HBN_BOR_CFG_OFFSET);
}


/* TODO: should take crystal type, defaults to 40Mhz crystal for BL602 as seems the most common */
static void system_clock_init(void)
{
#if 0
	system_init_root_clock(0, 32);
	system_set_root_clock_dividers(0, 0);
	system_clock_trim_32M();
#else
	system_init_root_clock(5, 4);
	system_set_root_clock_dividers(0, 2);
#endif
	system_set_machine_timer_clock(1, 0, mtimer_get_clk_src_div());
}

static void peripheral_clock_init(void)
{
	uint32_t regval = sys_read32(0x20000584);
	/* enable ADC clock routing */
	regval |= (1 << 2);
	/* enable UART0 clock routing */
	regval |= (1 << 16);
	/* enable I2C0 clock routing */
	regval |= (1 << 19);
	sys_write32(regval, 0x20000584);
	uart_set_clock(1, 1, 0);
}


#ifdef CONFIG_RISCV_GP
ulong_t __soc_get_gp_initial_value(void)
{
	extern uint32_t __global_pointer$;
	return (ulong_t)&__global_pointer$;
}
#endif


static void enable_icache(void)
{
	uint32_t tmpVal = 0;

	__asm__ volatile (
		"fence\n"
		"fence.i\n"
		/* th.icache.iall */
		".insn 0x100000B\n"
	);
	__asm__ volatile(
		"csrr %0, 0x7C1"
		: "=r"(tmpVal));
	tmpVal |= (1 << 0);
	__asm__ volatile(
		"csrw 0x7C1, %0"
		:
		: "r"(tmpVal));
	__asm__ volatile (
		"fence\n"
		"fence.i\n"
	);
}

static void enable_dcache(void)
{
	uint32_t tmpVal = 0;

	__asm__ volatile (
		"fence\n"
		"fence.i\n"
		/* th.dcache.iall */
		".insn 0x20000B\n"
	);
	__asm__ volatile(
		"csrr %0, 0x7C1"
		: "=r"(tmpVal));
	tmpVal |= (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 12);
	__asm__ volatile(
		"csrw 0x7C1, %0"
		:
		: "r"(tmpVal));
	__asm__ volatile (
		"fence\n"
		"fence.i\n"
	);
}

static void enable_thead_isa_ext(void)
{
	uint32_t tmpVal = 0;

	__asm__ volatile(
		"csrr %0, 0x7C0"
		: "=r"(tmpVal));
	tmpVal |= (1 << 22);
	__asm__ volatile(
		"csrw 0x7C0, %0"
		:
		: "r"(tmpVal));
}

static void disable_interrupt_autostacking(void)
{
	uint32_t tmpVal = 0;

	__asm__ volatile(
		"csrr %0, 0x7E1"
		: "=r"(tmpVal));
	tmpVal &= ~(0x3 << 16);
	__asm__ volatile(
		"csrw 0x7E1, %0"
		:
		: "r"(tmpVal));
}

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */

static int bl61x_riscv_init(void)
{
	uint32_t key;
	uint32_t tmpVal = 0;

	key = irq_lock();

	/* turn off USB power */
	sys_write32((1 << 5), PDS_BASE + PDS_USB_CTL_OFFSET);
	sys_write32(0, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	/* here go FPU manage if needed */
	/* enable unaligned access error goes here */
	/* supplementary clic init goes here */

	enable_thead_isa_ext();
	enable_dcache();
	enable_icache();
	disable_interrupt_autostacking();

	/* reset uart signals */
	sys_write32(0xffffffff, GLB_BASE + GLB_UART_CFG1_OFFSET);
	sys_write32(0x0000ffff, GLB_BASE + GLB_UART_CFG2_OFFSET);

	/* TODO: 'em' config for ble goes here */

	bl61x_BOD_init();

	//system_clock_init();
	//peripheral_clock_init();

	irq_unlock(key);

	/* wait 10 ms for peripherals to be ready */
	k_timepoint_t end_timeout = sys_timepoint_calc(K_MSEC(10));

	while (!sys_timepoint_expired(end_timeout)) {
	}

	return 0;
}


SYS_INIT(bl61x_riscv_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
