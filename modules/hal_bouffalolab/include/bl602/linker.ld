#include <zephyr/arch/riscv/common/linker.ld>

__HeapLimit = ORIGIN(RAM) + LENGTH(RAM);
__HeapBase = _end;
__metal_chicken_bit = 0;
PROVIDE(__global_pointer$ = RAM_BASE + 0x800 );
