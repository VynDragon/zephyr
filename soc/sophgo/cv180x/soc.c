#include <zephyr/kernel.h>
#include <stdint.h>

static void clean_dcache(void)
{
	__asm__ volatile (
		"fence\n"
		/* th.dcache.call*/
		".insn 0x10000B\n"
		"fence\n"
	);
}

static void clean_icache(void)
{
	__asm__ volatile (
		"fence\n"
		"fence.i\n"
		/* th.icache.iall */
		".insn 0x100000B\n"
		"fence\n"
		"fence.i\n"
	);
}

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
	tmpVal |= (1 << 1);
	__asm__ volatile(
		"csrw 0x7C1, %0"
		:
		: "r"(tmpVal));
	__asm__ volatile (
		"fence\n"
		"fence.i\n"
	);
}

static void enable_branchpred(bool yes)
{
	uint32_t tmpVal = 0;

	__asm__ volatile (
		"fence\n"
		"fence.i\n"
	);
	__asm__ volatile(
		"csrr %0, 0x7C1"
		: "=r"(tmpVal));
	if (yes) {
		tmpVal |= (1 << 5) | (1 << 6);
	} else {
		tmpVal &= ~((1 << 5) | (1 << 6));
	}
	__asm__ volatile(
		"csrw 0x7C1, %0"
		:
		: "r"(tmpVal));
	__asm__ volatile (
		"fence\n"
		"fence.i\n"
	);
}

void soc_early_init_hook(void)
{
	uint32_t key;
	key = irq_lock();

	enable_icache();
	enable_dcache();
	clean_dcache();
	clean_icache();
	enable_branchpred(true);

	irq_unlock(key);
}
