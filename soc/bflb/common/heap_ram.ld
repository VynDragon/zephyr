/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Heaps and network buffers first used after the POST_KERNEL memory controller init */
#if DT_HAS_CHOSEN(bflb_heap_psram) && defined(CONFIG_MEMC)
SECTION_PROLOGUE(_bflb_heap_psram, (NOLOAD),)
{
	*(".noinit.*subsys/net/ip/net_pkt.c.[0-9]*")
	*(".noinit.*lib/libc/common/source/stdlib/malloc.c*")
	*(.mbedtls_heap)
} GROUP_LINK_IN(LINKER_DT_NODE_REGION_NAME_TOKEN(DT_CHOSEN(bflb_heap_psram)))
#endif

/* Heaps and network packet pools, in a region usable from PRE_KERNEL_1 */
#if DT_HAS_CHOSEN(bflb_heap_ram)
SECTION_PROLOGUE(_bflb_heap_ram, (NOLOAD),)
{
	*(.noinit.*kheap_buf__system_heap*)
	*(".noinit.*lib/libc/common/source/stdlib/malloc.c*")
	*(.mbedtls_heap)
	*(".noinit.*subsys/net/ip/net_pkt.c*")
} GROUP_LINK_IN(LINKER_DT_NODE_REGION_NAME_TOKEN(DT_CHOSEN(bflb_heap_ram)))
#endif
