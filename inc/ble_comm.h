#ifndef BLE_COMM_H_
#define BLE_COMM_H_
#include <uart_async_adapter.h>

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include <zephyr/drivers/gpio.h>

#include "spi_ble_queue.h"

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME


#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>

#define FEM_PDN_PIN   20   /* P0.20 */
#define FEM_TXEN_PIN  19   /* P0.19 */
#define FEM_RXEN_PIN  31   /* P0.31 */
#define FEM_MODE_PIN  9    /* P1.09 */

/* Initialize BLE stack + NUS service and start advertising */
int ble_comm_init(void);

/* Return true if there is an active BLE connection */
bool ble_comm_is_connected(void);

/* Send data over NUS. Splitting into chunks if needed, if you want. */
int ble_comm_send(const uint8_t *data, size_t len);

bool get_nus_ble_data_available();

uint8_t * get_nus_ble_data();

void reset_nus_ble_data_available();

int get_nus_ble_data_len();

#endif /* BLE_COMM_H_ */