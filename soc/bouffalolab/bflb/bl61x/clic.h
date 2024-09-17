/*
 * Copyright (c) 2021, ATL Electronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SIFIVE_CLIC_H
#define _SIFIVE_CLIC_H

#define CLIC_CTRL_ADDR  0xE0000000
#define CLIC_HART0_ADDR 0xE0800000

#define CLIC_MSIP          0x0000
#define CLIC_MSIP_size     0x4
#define CLIC_MTIMECMP      0x4000
#define CLIC_MTIMECMP_size 0x8
#define CLIC_MTIME         0xBFF8
#define CLIC_MTIME_size    0x8

#define CLIC_INTIP	0x1000
#define CLIC_INTIE	0x1001
#define CLIC_INTCFG	0x1002
#define CLIC_CFG	0x0
#define CLIC_INFO	0x4
#define CLIC_THRESOLD	0x8

#endif /* _SIFIVE_CLIC_H */
