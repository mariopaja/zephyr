/*
 * Copyright (c) 2024, ZAL GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

static int usb_init(void)
{

	const struct gpio_dt_spec usb_psw =
		GPIO_DT_SPEC_GET(DT_NODELABEL(board_init), usb_psw_gpios);

	gpio_pin_configure_dt(&usb_psw, GPIO_OUTPUT_ACTIVE);

	return 0;
}

SYS_INIT(usb_init, POST_KERNEL, 41);
