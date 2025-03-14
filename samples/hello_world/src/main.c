/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <stdio.h>
#include <stdint.h>
#include <zephyr/kernel.h>

struct potato_t {
uint8_t start;
const uint8_t *padding;
uint16_t end;
};

const uint8_t paddock[300000] = {[0 ... 299999] = 0xAD, };

const struct potato_t potato = {
.start = 0xDE,
.padding = paddock,
.end = 0xBEEF,
};

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	printf("POTATO start: %x\n", potato.start);

	for (int i = 0; i < 300000; i+=1000) {
		printf("Potato padding %x: %x\n", &(potato.padding[i]), potato.padding[i]);
		k_msleep(2);
	}
	printf("Potato end: %x\n", potato.end);

	return 0;
}
