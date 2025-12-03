#include "ble_comm.h"
#include "spi_ble_queue.h"

LOG_MODULE_REGISTER(ble_comm, LOG_LEVEL_INF);

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;
static struct k_work adv_work;

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
static struct k_work_delayable uart_work;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

struct k_msgq spi_packet_msgq;


K_MSGQ_DEFINE(spi_packet_msgq,
              sizeof(struct spi_packet),
              SPI_PACKET_QUEUE_LEN,
              4); // alignment

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

#ifdef CONFIG_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
#define async_adapter NULL
#endif

#define BLE_HANDSHAKE_IN_NODE   DT_NODELABEL(gpio0)
#define BLE_HANDSHAKE_OUT_NODE  DT_NODELABEL(gpio1)

#define BLE_HANDSHAKE_IN_PIN    12   /* P0.12 */
#define BLE_HANDSHAKE_OUT_PIN   0    /* P1.00 */

static const struct device * const gpio0_dev = DEVICE_DT_GET(BLE_HANDSHAKE_IN_NODE);
static const struct device * const gpio1_dev = DEVICE_DT_GET(BLE_HANDSHAKE_OUT_NODE);

static bool ble_connected = false;

bool ble_comm_is_connected(void)
{
    return ble_connected;
}


static int ble_handshake_gpio_init(void)
{
    int ret;

    if (!device_is_ready(gpio0_dev)) {
        printk("gpio0 device not ready\n");
        return -ENODEV;
    }

    if (!device_is_ready(gpio1_dev)) {
        printk("gpio1 device not ready\n");
        return -ENODEV;
    }

    /* P0.12 as input (you can change pull if needed) */
    ret = gpio_pin_configure(gpio0_dev,
                             BLE_HANDSHAKE_IN_PIN,
                             GPIO_INPUT | GPIO_PULL_DOWN);
    if (ret) {
        printk("Failed to config P0.12 as input: %d\n", ret);
        return ret;
    }

    /* P1.00 as output, default low, active-high */
    ret = gpio_pin_configure(gpio1_dev,
                             BLE_HANDSHAKE_OUT_PIN,
                             GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_HIGH);
    if (ret) {
        printk("Failed to config P1.00 as output: %d\n", ret);
        return ret;
    }

    return 0;
}

void ble_handshake_set(bool level)
{
    /* level = true → set P1.00 high (slave has data)
     * level = false → set P1.00 low
     */
    gpio_pin_set(gpio1_dev, BLE_HANDSHAKE_OUT_PIN, level ? 1 : 0);
}

int ble_handshake_read(void)
{
    /* Read P0.12 (master signal) */
    return gpio_pin_get(gpio0_dev, BLE_HANDSHAKE_IN_PIN);
}


void ble_handshake_toggle()
{
    /* level = true → set P1.00 high (slave has data)
     * level = false → set P1.00 low
     */
    int ret = gpio_pin_toggle(gpio1_dev, BLE_HANDSHAKE_OUT_PIN);

	if (ret) {
        printk("Failed to toggle P1.00: %d\n", ret);
    }
}


static size_t ble_get_max_payload(void)
{
    /* Default ATT MTU is 23 bytes (20 bytes payload) */
    uint16_t att_mtu = 23U;

    struct bt_conn *conn = current_conn;

    if (conn != NULL) {
        uint16_t tmp = bt_gatt_get_mtu(conn);

        /* bt_gatt_get_mtu() should be >= 23 when connected, but be safe */
        if (tmp >= 23U) {
            att_mtu = tmp;
        }
    }

    /* ATT header is 3 bytes → payload = MTU - 3 */
    uint16_t max_payload = (att_mtu > 3U) ? (att_mtu - 3U) : 20U;

    /* Do not exceed what fits in one SPI packet */
    if (max_payload > SPI_PACKET_MAX_LEN) {
        max_payload = SPI_PACKET_MAX_LEN;
    }

    return (size_t)max_payload;
}




static struct bt_gatt_exchange_params mtu_exchange_params;

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                            struct bt_gatt_exchange_params *params)
{
    if (!err) {
        uint16_t mtu = bt_gatt_get_mtu(conn);
        printk("MTU exchange done, new MTU = %d\n", mtu);
    } else {
        printk("MTU exchange failed (err %u)\n", err);
    }
}

void ble_send(uint8_t *data, int size)
{
    if (!ble_connected) {
        return;
    }

    struct bt_conn *conn = current_conn; // however you're storing it
    uint16_t att_mtu = bt_gatt_get_mtu(conn);
    uint16_t max_payload = att_mtu - 3;  // safe payload per notification
	//printk("max_payload = %d\r\n", max_payload);

    int offset = 0;
    while (offset < size) {
        int chunk = size - offset;
        if (chunk > max_payload) {
            chunk = max_payload;
        }

        int err = bt_nus_send(conn, &data[offset], chunk);
        if (err) {
            printk("bt_nus_send failed (err %d)\n", err);
            break;
        }

        offset += chunk;
    }
}

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Advertising successfully started");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);
	
	ble_connected = true;

	current_conn = bt_conn_ref(conn);

	dk_set_led_on(CON_STATUS_LED);


	/*  Example: request 30–50 ms connection interval, no latency, 4 s timeout
     *  min_interval = 24 * 1.25 ms = 30 ms
     *  max_interval = 40 * 1.25 ms = 50 ms
     *  timeout      = 400 * 10 ms = 4 s
     */
    struct bt_le_conn_param param = {
        .interval_min = 12,
        .interval_max = 24,
        .latency      = 0,
        .timeout      = 500,
    };

    int rc = bt_conn_le_param_update(conn, &param);
    if (rc) {
        printk("conn param update failed (err %d)\n", rc);
    }

	mtu_exchange_params.func = mtu_exchange_cb;

    int ret = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
    
	if (ret) {
        printk("bt_gatt_exchange_mtu failed (err %d)\n", ret);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	
	ble_connected = false;

	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		dk_set_led_off(CON_STATUS_LED);
	}
}

static void recycled_cb(void)
{
	LOG_INF("Connection object available from previous conn. Disconnect is complete!");
	advertising_start();
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d %s", addr, level, err,
			bt_security_err_to_str(err));
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.recycled         = recycled_cb,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);

	if (IS_ENABLED(CONFIG_SOC_SERIES_NRF54HX) || IS_ENABLED(CONFIG_SOC_SERIES_NRF54LX)) {
		LOG_INF("Press Button 0 to confirm, Button 1 to reject.");
	} else {
		LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
	}
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d %s", addr, reason,
		bt_security_err_to_str(reason));
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

uint8_t nus_ble_data[255];
int nus_ble_data_len = 0;
bool nus_ble_data_available = false;

bool get_nus_ble_data_available()
{
	if(nus_ble_data_available)
		return true;

	return false;
}

void reset_nus_ble_data_available()
{
	nus_ble_data_available = false;
}

int get_nus_ble_data_len()
{
	return nus_ble_data_len;
}

uint8_t * get_nus_ble_data()
{
	return nus_ble_data;
}

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);

	ble_handshake_toggle();

	memcpy(nus_ble_data, data, len);

	nus_ble_data_len = len;

	nus_ble_data_available = true;

	// for (uint16_t pos = 0; pos != len;) {
	// 	struct uart_data_t *tx = k_malloc(sizeof(*tx));

	// 	if (!tx) {
	// 		LOG_WRN("Not able to allocate UART send data buffer");
	// 		return;
	// 	}

	// 	/* Keep the last byte of TX buffer for potential LF char. */
	// 	size_t tx_data_size = sizeof(tx->data) - 1;

	// 	if ((len - pos) > tx_data_size) {
	// 		tx->len = tx_data_size;
	// 	} else {
	// 		tx->len = (len - pos);
	// 	}

	// 	memcpy(tx->data, &data[pos], tx->len);

	// 	pos += tx->len;

	// 	/* Append the LF character when the CR character triggered
	// 	 * transmission from the peer.
	// 	 */
	// 	if ((pos == len) && (data[len - 1] == '\r')) {
	// 		tx->data[tx->len] = '\n';
	// 		tx->len++;
	// 	}

	// 	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	// 	if (err) {
	// 		k_fifo_put(&fifo_uart_tx_data, tx);
	// 	}
	// }
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
	if (accept) {
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	} else {
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	if (auth_conn) {
		if (buttons & KEY_PASSKEY_ACCEPT) {
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT) {
			num_comp_reply(false);
		}
	}
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */


int ble_comm_init(void)
{
	int err = 0;

	err = ble_handshake_gpio_init();

    if (err) {
        printk("Handshake GPIO init failed: %d\n", err);
        return err;
    }

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization callbacks. (err: %d)", err);
			return 0;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization info callbacks. (err: %d)", err);
			return 0;
		}
	}

	err = bt_enable(NULL);
	if (err) {
		error();
	}

	LOG_INF("Bluetooth initialized");

    
	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return 0;
	}

	k_work_init(&adv_work, adv_work_handler);
	advertising_start();

}


void ble_write_thread(void)
{
    struct spi_packet pkt;
    static uint8_t agg_buf[SPI_PACKET_MAX_LEN];
    size_t agg_len = 0U;

    while (1) {
        /* Drop any partial buffer if the link went down */
        if (!ble_connected && agg_len > 0U) {
            agg_len = 0U;
        }

        /* If we already have data, wait up to 10 ms for more.
         * Otherwise, block until the first packet arrives.
         */
        k_timeout_t timeout = (agg_len == 0U) ? K_FOREVER : K_MSEC(10);

        int ret = k_msgq_get(&spi_packet_msgq, &pkt, timeout);

        if (ret == 0) {
            /* New SPI packet available */
            if (!ble_connected) {
                /* Not connected: drop packet and any partial buffer */
                agg_len = 0U;
                continue;
            }

            size_t max_payload = ble_get_max_payload();
            size_t offset = 0U;

            while (offset < pkt.len) {
                size_t space    = max_payload - agg_len;
                size_t copy_len = pkt.len - offset;

                if (space == 0U) {
                    /* Active buffer full, push it out and start again */
                    ble_send(agg_buf, agg_len);
                    agg_len = 0U;

                    max_payload = ble_get_max_payload();
                    space       = max_payload;
                }

                if (copy_len > space) {
                    copy_len = space;
                }

                memcpy(&agg_buf[agg_len], &pkt.data[offset], copy_len);
                agg_len += copy_len;
                offset  += copy_len;

                if (agg_len == max_payload) {
                    /* We reached exactly one MTU payload: send immediately */
                    ble_send(agg_buf, agg_len);
                    agg_len = 0U;

                    max_payload = ble_get_max_payload();
                }
            }
        } else {
            /* Timeout (no new SPI data within 10 ms) or other error:
             * flush any pending data in the active buffer.
             */
            if (agg_len > 0U && ble_connected) {
                ble_send(agg_buf, agg_len);
                agg_len = 0U;
            }
        }
    }
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);

