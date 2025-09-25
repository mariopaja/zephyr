/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, 4);

#include <zephyr/kernel.h>
#include <stdio.h>

int main(void)
{

	while (1) {
		LOG_INF("Hello World! %s", CONFIG_BOARD_TARGET);
		k_msleep(1000);
	}

	return 0;
}
