/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */

#include "ble_comm.h"

void spi_slave_init(void);



int main(void)
{
	int blink_status = 0;
	int err = ble_comm_init();

    if (err) {
        printk("BLE init failed (err %d)\n", err);
        return err;
    }

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}
