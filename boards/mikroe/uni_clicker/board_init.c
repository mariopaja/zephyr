/*
 * Copyright (c) 2025 Mario Paja
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

	const struct gpio_dt_spec usb_id = GPIO_DT_SPEC_GET(DT_NODELABEL(board_init), usb_id_gpios);

	gpio_pin_configure_dt(&usb_psw, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&usb_id, GPIO_OUTPUT_ACTIVE);

	k_msleep(100); /* wait for the power to stabilize */

	return 0;
}

SYS_INIT(usb_init, POST_KERNEL, 41);
