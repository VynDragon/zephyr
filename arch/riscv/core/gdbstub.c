/*
 * Copyright (c) 2024 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <kernel_internal.h>
#include <zephyr/debug/gdbstub.h>
#include <zephyr/arch/riscv/gdbstub.h>

static const int packet_pos[] = {
	1,
	5,
	6,
	7,
#if !defined(CONFIG_RISCV_ISA_RV32E)
	28,
	29,
	30,
	31,
#endif
	10,
	11,
	12,
	13,
	14,
	15,
#if !defined(CONFIG_RISCV_ISA_RV32E)
	16,
	17,
#endif
	32,
	8,
#ifdef CONFIG_USERSPACE
	2,
#endif
};

/* Required struct */
static struct gdb_ctx ctx;

static struct trigger_ctx trigger_ctx = {
	.enumerating = false,
	.enumeration_no = false,
	.used_cnt = 1,
	.trigger_cnt = 0,
};


/* the CSR values cannot be arguments, they must be constant at compile time */
static void riscv_enumerate_triggers(void)
{
	unsigned long	tmpVal = 0;
	unsigned long	id = 0;

	trigger_ctx.enumerating = true;

	/* write zero to tselect */
	__asm__ volatile(
		"csrw 0x7A0, %0"
		:
		: "r"(tmpVal));

	/* if Illegal Instruction exeption, no trigger support */
	if (trigger_ctx.enumeration_no) {
		trigger_ctx.enumerating = false;
		return;
	}

	while (id < 256) {
		/* write id to tselect */
		__asm__ volatile(
		"csrw 0x7A0, %0"
		:
		: "r"(id));
		/* read back tselect */
		__asm__ volatile(
			"csrr %0, 0x7A0"
			: "=r"(tmpVal));
		if (tmpVal != id) {
			trigger_ctx.enumerating = false;
			return;
		}

		/* read tinfo */
		__asm__ volatile(
			"csrr %0, 0x7A4"
			: "=r"(tmpVal));
		if (trigger_ctx.enumeration_no) {
			trigger_ctx.enumeration_no = false;
		} else if ((tmpVal & RISCV_TINFO_INFO_MASK) != 1) {
			trigger_ctx.triggers[id].supported_types =
				tmpVal & RISCV_TINFO_INFO_MASK;
			trigger_ctx.triggers[id].version =
				(tmpVal & RISCV_TINFO_VERSION_MASK)
				>> RISCV_TINFO_VERSION_SHIFT;
			trigger_ctx.triggers[id].supported = true;
		} else if ((tmpVal & RISCV_TINFO_INFO_MASK) == 1) {
			/* if tinfo info is 1, no trigger at this id */
			trigger_ctx.enumerating = false;
			return;
		}

		/* read tdata1 */
		__asm__ volatile(
			"csrr %0, 0x7A1"
			: "=r"(tmpVal));
		if ((tmpVal & RISCV_TDATA1_TYPE_MASK) == 0
			&& !trigger_ctx.triggers[id].supported) {
			/* if tdata1 type is 0 and tinfo doesnt tell us this
			 * exists, we have no trigger here
			 */
			trigger_ctx.enumerating = false;
			return;
		}
		trigger_ctx.triggers[id].supported = true;
		trigger_ctx.triggers[id].type =
			(tmpVal & RISCV_TDATA1_TYPE_MASK)
			>> RISCV_TDATA1_TYPE_SHIFT;
		trigger_ctx.triggers[id].supported_types |=
			1 << trigger_ctx.triggers[id].type;
		if ((tmpVal & RISCV_TDATA1_DMODE_MASK) != 0) {
			trigger_ctx.triggers[id].debug_only = true;
		}
		trigger_ctx.triggers[id].data =
			tmpVal & RISCV_TDATA1_DATA_MASK;
		trigger_ctx.trigger_cnt++;
		id++;
	}
	trigger_ctx.enumerating = false;
}

static int riscv_trigger_setup_icount(uint8_t trigger_id, unsigned long count)
{
	uint32_t	tmpVal = 0;

	if (trigger_id > trigger_ctx.trigger_cnt) {
		return -1;
	}
	if (!trigger_ctx.triggers[trigger_id].supported) {
		return -1;
	}
	if (trigger_ctx.triggers[trigger_id].supported_types == 0) {
		if (trigger_ctx.triggers[trigger_id].type
			!= RISCV_TDATA1_TYPE_ICOUNT) {
			return -ENOTSUP;
		}
	} else {
		if ((trigger_ctx.triggers[trigger_id].supported_types
			& RISCV_TINFO_STYPE_ICOUNT_MASK) == 0) {
			return -ENOTSUP;
		}
	}

	/* write id to tselect */
	__asm__ volatile(
	"csrw 0x7A0, %0"
		:
		: "r"(trigger_id)
	);

	/* write 0 tdata1 */
	__asm__ volatile(
	"csrw 0x7A1, %0"
		:
		: "r"(0)
	);
	/* write 0 tdata2 */
	__asm__ volatile(
	"csrw 0x7A2, %0"
		:
		: "r"(0)
	);
	/* write 0 tdata3 */
	__asm__ volatile(
	"csrw 0x7A3, %0"
		:
		: "r"(0)
	);

	tmpVal = 0;

	tmpVal |= RISCV_TDATA1_TYPE_ICOUNT << RISCV_TDATA1_TYPE_SHIFT;

	tmpVal |= 1 << RISCV_TDATA1_ICOUNT_M_SHIFT
		| 1 << RISCV_TDATA1_ICOUNT_S_SHIFT
		| 1 << RISCV_TDATA1_ICOUNT_U_SHIFT;

	tmpVal |= RISCV_TDATA1_ACTION_EBREAK;

	tmpVal |= count << RISCV_TDATA1_ICOUNT_COUNT_SHIFT;

	/* write tdata1 */
	__asm__ volatile(
	"csrw 0x7A1, %0"
		:
		: "r"(tmpVal)
	);

	return 0;
}

/* most basic breakpoint type */
static int riscv_trigger_setup_mcontrol_address(uint8_t trigger_id, unsigned long address)
{
	uint32_t	tmpVal = 0;

	if (trigger_id > trigger_ctx.trigger_cnt) {
		return -1;
	}
	if (!trigger_ctx.triggers[trigger_id].supported) {
		return -1;
	}
	if (trigger_ctx.triggers[trigger_id].supported_types == 0) {
		if (trigger_ctx.triggers[trigger_id].type
			!= RISCV_TDATA1_TYPE_MCONTROL) {
			return -ENOTSUP;
		}
	} else {
		if ((trigger_ctx.triggers[trigger_id].supported_types
			& RISCV_TINFO_STYPE_MCONTROL_MASK) == 0) {
			return -ENOTSUP;
		}
	}

	/* write id to tselect */
	__asm__ volatile(
	"csrw 0x7A0, %0"
		:
		: "r"(trigger_id)
	);

	/* write 0 tdata1 */
	__asm__ volatile(
	"csrw 0x7A1, %0"
		:
		: "r"(0)
	);
	/* write 0 tdata2 */
	__asm__ volatile(
	"csrw 0x7A2, %0"
		:
		: "r"(0)
	);
	/* write 0 tdata3 */
	__asm__ volatile(
	"csrw 0x7A3, %0"
		:
		: "r"(0)
	);

	tmpVal = 0;

	tmpVal |= RISCV_TDATA1_TYPE_MCONTROL << RISCV_TDATA1_TYPE_SHIFT;

	tmpVal |= RISCV_TDATA1_MCONTROL_U_MASK
		| RISCV_TDATA1_MCONTROL_S_MASK
		| RISCV_TDATA1_MCONTROL_M_MASK;

	tmpVal |= RISCV_TDATA1_ACTION_EBREAK << RISCV_TDATA1_MCONTROL_ACTION_SHIFT;

	tmpVal |= RISCV_TDATA1_MCONTROL_LOAD_MASK
		| RISCV_TDATA1_MCONTROL_STORE_MASK
		| RISCV_TDATA1_MCONTROL_EXECUTE_MASK;

	/* write tdata1 */
	__asm__ volatile(
	"csrw 0x7A1, %0"
		:
		: "r"(tmpVal)
	);

	/* write tdata2 */
	__asm__ volatile(
	"csrw 0x7A2, %0"
		:
		: "r"(address)
	);

	return 0;
}

static int riscv_trigger_setup_mcontrol_next(uint8_t trigger_id)
{
	uint32_t	tmpVal = 0;
	uint32_t	tmpVal2 = 0;

	if (trigger_id > trigger_ctx.trigger_cnt) {
		return -1;
	}
	if (!trigger_ctx.triggers[trigger_id].supported) {
		return -1;
	}
	if (trigger_ctx.triggers[trigger_id].supported_types == 0) {
		if (trigger_ctx.triggers[trigger_id].type
			!= RISCV_TDATA1_TYPE_MCONTROL) {
			return -ENOTSUP;
		}
	} else {
		if ((trigger_ctx.triggers[trigger_id].supported_types
			& RISCV_TINFO_STYPE_MCONTROL_MASK) == 0) {
			return -ENOTSUP;
		}
	}

	/* write id to tselect */
	__asm__ volatile(
	"csrw 0x7A0, %0"
		:
		: "r"(trigger_id)
	);

	/* write 0 tdata1 */
	__asm__ volatile(
	"csrw 0x7A1, %0"
		:
		: "r"(0)
	);
	/* write 0 tdata2 */
	__asm__ volatile(
	"csrw 0x7A2, %0"
		:
		: "r"(0)
	);
	/* write 0 tdata3 */
	__asm__ volatile(
	"csrw 0x7A3, %0"
		:
		: "r"(0)
	);

	tmpVal = 0;

	tmpVal |= RISCV_TDATA1_TYPE_MCONTROL << RISCV_TDATA1_TYPE_SHIFT;

	tmpVal |= RISCV_TDATA1_MCONTROL_U_MASK
		| RISCV_TDATA1_MCONTROL_S_MASK
		| RISCV_TDATA1_MCONTROL_M_MASK;

	tmpVal |= RISCV_TDATA1_ACTION_EBREAK << RISCV_TDATA1_MCONTROL_ACTION_SHIFT;

	tmpVal |= RISCV_TDATA1_MCONTROL_EXECUTE_MASK;

	/* match any address greater than tdata2 */
	tmpVal |= 2 << RISCV_TDATA1_MCONTROL_MATCH_SHIFT;


	/* write tdata1 */
	__asm__ volatile(
	"csrw 0x7A1, %0"
		:
		: "r"(tmpVal)
	);

	__asm__ volatile(
		"csrr %0, 0x7A1"
		: "=r"(tmpVal2)
	);

	/* something we set is unsupported */
	if ((tmpVal2 & ~RISCV_TDATA1_MCONTROL_MSU_MASK)
		!= (tmpVal & ~RISCV_TDATA1_MCONTROL_MSU_MASK)) {
		return -1;
	}

	return 0;
}

static unsigned int get_exception(unsigned int id)
{
	unsigned int exception = 255;

	switch (id) {
	case 0:
		exception = GDB_EXCEPTION_INVALID_MEMORY;
		break;
	case 1:
		exception = GDB_EXCEPTION_MEMORY_FAULT;
		break;
	case 2:
		exception = GDB_EXCEPTION_INVALID_INSTRUCTION;
		break;
	case 3:
		exception = GDB_EXCEPTION_BREAKPOINT;
		break;
	case 4:
		exception = GDB_EXCEPTION_INVALID_MEMORY;
		break;
	case 5:
		exception = GDB_EXCEPTION_MEMORY_FAULT;
		break;
	case 6:
		exception = GDB_EXCEPTION_INVALID_MEMORY;
		break;
	case 7:
		exception = GDB_EXCEPTION_MEMORY_FAULT;
		break;
	case 12:
		exception = GDB_EXCEPTION_MEMORY_FAULT;
		break;
	case 13:
		exception = GDB_EXCEPTION_MEMORY_FAULT;
		break;
	case 15:
		exception = GDB_EXCEPTION_MEMORY_FAULT;
		break;
	default:
		break;
	}

	return exception;
}

static void advance_ebreak(void)
{
	/* advance mepc so we dont retrigger EBREAK */
	if (sys_read32(ctx.registers[MEPC]) == 0x100073) {
		ctx.registers[MEPC] += 4;
	/* advance mepc so we dont retrigger C.EBREAK */
	} else if (sys_read16(ctx.registers[MEPC]) == 0x9002) {
		ctx.registers[MEPC] += 2;
	}
}

static size_t riscv_instruction_size(void)
{
	if ((sys_read32(ctx.registers[MEPC]) & 0x3) == 3) {
		return 4;
	} else if ((sys_read16(ctx.registers[MEPC]) & 0x3) != 3) {
		return 2;
	}
	return 4;
}

/* Wrapper function to save and restore execution c
 * This is called from _Fault in fatal.c when a exception happens
 * if exception is BREAK we continue running after we are done
 */
int z_gdb_entry(struct arch_esf *esf, unsigned int id)
{
	/* trigger enumeration uses illegal instruction exception */
	if (trigger_ctx.enumerating) {
		if (id == 2) {
			trigger_ctx.enumeration_no = true;
			return 0;
		}
	}
	ctx.exception = get_exception(id);
	if (ctx.exception == 255) {
		return -1;
	}

	/* save the registers */
	ctx.registers[RA] = esf->ra;
	ctx.registers[T0] = esf->t0;
	ctx.registers[T1] = esf->t1;
	ctx.registers[T2] = esf->t2;
#if !defined(CONFIG_RISCV_ISA_RV32E)
	ctx.registers[T3] = esf->t3;
	ctx.registers[T4] = esf->t4;
	ctx.registers[T5] = esf->t5;
	ctx.registers[T6] = esf->t6;
#endif
	ctx.registers[A0] = esf->a0;
	ctx.registers[A1] = esf->a1;
	ctx.registers[A2] = esf->a2;
	ctx.registers[A3] = esf->a3;
	ctx.registers[A4] = esf->a4;
	ctx.registers[A5] = esf->a5;
#if !defined(CONFIG_RISCV_ISA_RV32E)
	ctx.registers[A6] = esf->a6;
	ctx.registers[A7] = esf->a7;
#endif
	ctx.registers[MEPC] = esf->mepc;
	ctx.registers[MSTATUS] = esf->mstatus;
	ctx.registers[S0] = esf->s0;
#ifdef CONFIG_USERSPACE
	ctx.registers[SP] = esf->sp;
#endif

	z_gdb_main_loop(&ctx);

	/* restore the registers */
	esf->ra = ctx.registers[RA];
	esf->t0 = ctx.registers[T0];
	esf->t1 = ctx.registers[T1];
	esf->t2 = ctx.registers[T2];
#if !defined(CONFIG_RISCV_ISA_RV32E)
	esf->t3 = ctx.registers[T3];
	esf->t4 = ctx.registers[T4];
	esf->t5 = ctx.registers[T5];
	esf->t6 = ctx.registers[T6];
#endif
	esf->a0 = ctx.registers[A0];
	esf->a1 = ctx.registers[A1];
	esf->a2 = ctx.registers[A2];
	esf->a3 = ctx.registers[A3];
	esf->a4 = ctx.registers[A4];
	esf->a5 = ctx.registers[A5];
#if !defined(CONFIG_RISCV_ISA_RV32E)
	esf->a6 = ctx.registers[A6];
	esf->a7 = ctx.registers[A7];
#endif
	esf->mepc = ctx.registers[MEPC];
	esf->mstatus = ctx.registers[MSTATUS];
	esf->s0 = ctx.registers[S0];
#ifdef CONFIG_USERSPACE
	esf->sp = ctx.registers[SP];
#endif
	if (ctx.exception != GDB_EXCEPTION_BREAKPOINT) {
		return -1;
	}

	return 0;
}

void arch_gdb_init(void)
{
	uint8_t mode = 0;

	riscv_enumerate_triggers();
	for (int i = 0; i < trigger_ctx.trigger_cnt && mode == 0;  i++) {
		if  ((trigger_ctx.triggers[i].supported_types
			& RISCV_TINFO_STYPE_ICOUNT_MASK) != 0) {
			mode = RISCV_TDATA1_TYPE_ICOUNT;
		} else if ((trigger_ctx.triggers[i].supported_types
			& RISCV_TINFO_STYPE_MCONTROL_MASK) != 0) {
			mode = RISCV_TDATA1_TYPE_MCONTROL;
		} else if (trigger_ctx.triggers[i].type == RISCV_TDATA1_TYPE_ICOUNT) {
			mode = RISCV_TDATA1_TYPE_ICOUNT;
		} else if (trigger_ctx.triggers[i].type == RISCV_TDATA1_TYPE_MCONTROL) {
			mode = RISCV_TDATA1_TYPE_MCONTROL;
		}
	}
	trigger_ctx.mode = mode;
	__asm__ volatile("EBREAK");
}

void arch_gdb_continue(void)
{
	advance_ebreak();
}

void arch_gdb_step(void)
{
	advance_ebreak();

	if (trigger_ctx.mode == RISCV_TDATA1_TYPE_ICOUNT) {
		riscv_trigger_setup_icount(0, 1);
	} else if (trigger_ctx.mode == RISCV_TDATA1_TYPE_MCONTROL) {
		riscv_trigger_setup_mcontrol_next(0);
	}
}

size_t arch_gdb_reg_readall(struct gdb_ctx *c, uint8_t *buf, size_t buflen)
{
	int ret = 0;
	/* All other registers are not supported */
	memset(buf, 'x', buflen);
	for (int i = 0; i < GDB_NUM_REGS; i++) {
		/* offset inside the packet */
		int pos = packet_pos[i] * sizeof(unsigned long) * 2;
		int r = gdb_bin2hex((const uint8_t *)(&c->registers[i]), 4, buf + pos, sizeof(unsigned long) * 2);
		if (r == 0) {
			ret = 0;
			break;
		}
		ret += r;
	}

	if (ret) {
		/* Since we don't support some most registers,
		 * set the packet size manually
		 */
		ret = GDB_READALL_PACKET_SIZE;
	}
	buf[buflen - 1] = 0;
	return ret;
}

size_t arch_gdb_reg_writeall(struct gdb_ctx *c, uint8_t *hex, size_t hexlen)
{
	int ret = 0;

	for (unsigned int i = 0; i < hexlen; i += 8) {
		if (hex[i] != 'x') {
			/* check if the stub supports this register */
			for (unsigned int j = 0; j < GDB_NUM_REGS; j++) {
				if (packet_pos[j] != i) {
					continue;
				}
				int r = hex2bin(hex + i * 8, 8, (uint8_t *)(c->registers + j), 4);

				if (r == 0) {
					return 0;
				}
				ret += r;
			}
		}
	}
	return ret;
}

size_t arch_gdb_reg_readone(struct gdb_ctx *c, uint8_t *buf, size_t buflen, uint32_t regno)
{
	/* Reading four bytes (could be any return value except 0, which would indicate an error) */
	int ret = 4;
	/* Fill the buffer with 'x' in case the stub does not support the required register */
	memset(buf, 'x', 8);
	/* Check which of our registers corresponds to regnum */
	for (int i = 0; i < GDB_NUM_REGS; i++) {
		if (packet_pos[i] == regno) {
			ret = bin2hex((uint8_t *)(c->registers + i), 4, buf, buflen);
			break;
		}
	}
	return ret;
}

size_t arch_gdb_reg_writeone(struct gdb_ctx *c, uint8_t *hex, size_t hexlen, uint32_t regno)
{
	int ret = 0;
	/* Set the value of a register */
	if (hexlen != 8) {
		return ret;
	}

	if (regno < (GDB_NUM_REGS - 1)) {
		/* Again, check the corresponding register index */
		for (int i = 0; i < GDB_NUM_REGS; i++) {
			if (packet_pos[i] == regno) {
				ret = hex2bin(hex, hexlen, (uint8_t *)(c->registers + i), 4);
				break;
			}
		}
	}
	return ret;
}

int arch_gdb_add_breakpoint(struct gdb_ctx *ctx, uint8_t type,
			    uintptr_t addr, uint32_t kind)
{
	if (trigger_ctx.used_cnt < trigger_ctx.trigger_cnt) {
		riscv_trigger_setup_mcontrol_address(trigger_ctx.used_cnt, addr);
		trigger_ctx.used_cnt++;
		return 0;
	}
	return -1;
}

int arch_gdb_remove_breakpoint(struct gdb_ctx *ctx, uint8_t type,
			       uintptr_t addr, uint32_t kind)
{
	return -2;
}
