#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>
#include <bluetooth/services/nus.h>
#include "spi_ble_queue.h"
#include "nRF_commands.h"
#include "ble_comm.h"


#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_slave);

#define SPI_SLAVE_STACK_SIZE 2048 
#define SPI_SLAVE_PRIORITY   5
#define SPI_RX_BUF_SIZE      200
#define MIN_PCK_SIZE         2 // header and size.

#define SPI_TX_BUF_SIZE 200

static uint8_t spi_tx_buf[SPI_TX_BUF_SIZE];
static size_t  spi_tx_len = 0;


#define SPI_SLAVE_NODE DT_NODELABEL(spi2)
static const struct device *const spi_dev = DEVICE_DT_GET(SPI_SLAVE_NODE);

static K_THREAD_STACK_DEFINE(spi_slave_stack, SPI_SLAVE_STACK_SIZE);
static struct k_thread spi_slave_thread_data;

static uint8_t spi_rx_buf[SPI_RX_BUF_SIZE];

static const struct spi_config spi_cfg = {
    .operation = SPI_OP_MODE_SLAVE |
                 SPI_MODE_CPOL |
                 SPI_MODE_CPHA |        /* CPOL_LOW, CPHA_HIGH (mode 1) */
                 SPI_WORD_SET(8) |
                 SPI_TRANSFER_MSB,
    .frequency = 0,
    .slave = 0,
};

void ble_send(uint8_t *data, int size);

int counter = 0;
int16_t tx_size = 0;

extern struct k_msgq spi_packet_msgq;

uint8_t msg[] = "This is the BLE msg";

void set_ble_pck_size()
{
    // Example: payload coming from master in this command
    // is what you want to send back on the next transaction.
    // Adjust to your real protocol.


    spi_tx_buf[0] = 0x55;

    if(get_nus_ble_data_available())
    {
        spi_tx_buf[1] = BLE_UART_MSG;
        spi_tx_buf[2] = (uint8_t) get_nus_ble_data_len();
    }
    else
    {
        spi_tx_buf[1] = BLE_NO_MSG;
        spi_tx_buf[2] = 0;
    }

    LOG_INF("spi_tx_buf[2] = %d, and size = %d", spi_tx_buf[2], sizeof(msg));

    spi_tx_len = 3;

    // Prepare TX buffers
    struct spi_buf tx_buf = {
        .buf = spi_tx_buf,
        .len = spi_tx_len,
    };
    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count   = 1,
    };

    // No RX here, master just reads
    int tret = spi_transceive(spi_dev, &spi_cfg, &tx, NULL);
    if (tret < 0) {
        LOG_ERR("spi_transceive (TX to master) failed: %d", tret);
    } else {
        LOG_DBG("Sent %d frames to master", tret);
    }
}

void set_ble_pck_data()
{
    // Example: payload coming from master in this command
    // is what you want to send back on the next transaction.
    // Adjust to your real protocol.

    spi_tx_buf[0] = 0x55;
    spi_tx_buf[1] = _RETRIEVE_BLE_PCK;

    uint8_t *nus_data = get_nus_ble_data();
    int ble_data_len = get_nus_ble_data_len();

    for(int inx_data = 0; inx_data<ble_data_len; inx_data++)
    {
        spi_tx_buf[inx_data+2] = nus_data[inx_data];
    }

    spi_tx_buf[2+ble_data_len] = 0xAA;
    
    spi_tx_len = 3+ble_data_len;

    // Prepare TX buffers
    struct spi_buf tx_buf = {
        .buf = spi_tx_buf,
        .len = spi_tx_len,
    };
    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count   = 1,
    };

    // No RX here, master just reads
    int tret = spi_transceive(spi_dev, &spi_cfg, &tx, NULL);
    if (tret < 0) {
        LOG_ERR("spi_transceive (TX to master) failed: %d", tret);
    } else {
        LOG_DBG("Sent %d frames to master", tret);
    }

    reset_nus_ble_data_available();
    
}



static void spi_slave_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    if (!device_is_ready(spi_dev)) {
        LOG_INF("SPIS device not ready");
        return;
    }

    LOG_INF("SPI slave thread started");

    while (1) {
        struct spi_buf rx_buf = {
            .buf = spi_rx_buf,
            .len = sizeof(spi_rx_buf),
        };
        struct spi_buf_set rx = {
            .buffers = &rx_buf,
            .count = 1,
        };

        int ret = spi_read(spi_dev, &spi_cfg, &rx);
        
        if (ret <= 0) {
            // you may want to LOG_ERR or handle specific errors here
            LOG_ERR("ERROR receiving SPIS data");
            continue;
        }

        // First two bytes are header + packet size → payload starts at index 2

        int payload_len = ret - MIN_PCK_SIZE;

        if(payload_len >= 0)
        {
            LOG_INF("Master-Request: 0x%x", spi_rx_buf[0]);
            switch (spi_rx_buf[0])
            {               
            
                /* Set the buffers so it can be read by the master */
            case _SET_BLE_BUFFER_NORDIC:
                LOG_INF("SPIS: Setting PACKET size!");
                set_ble_pck_size();
                break;

            case _RETRIEVE_BLE_PCK:
                LOG_INF("SPIS: Sending PCK!");
                set_ble_pck_data();
                break;
                
            case _SEND_OVER_NUS:
                
                /* Send the data over NUS/BLE */
                if (payload_len <= 0) {
                    continue;
                }
                if (payload_len > SPI_PACKET_MAX_LEN) {
                    payload_len = SPI_PACKET_MAX_LEN;  // clamp, or drop if you prefer
                }

                struct spi_packet pkt;
                pkt.len = (uint16_t)payload_len;
                memcpy(pkt.data, &spi_rx_buf[2], pkt.len);

                int qret = k_msgq_put(&spi_packet_msgq, &pkt, K_NO_WAIT);
                if (qret != 0) {
                    // queue full → decide policy: drop oldest, drop new, or block
                    // simple approach: drop and optionally count/log
                    // LOG_WRN("SPI packet queue full, dropping packet");
                }
                break;
                
            default:
                /* Do nothing with packets that are unknown */
                break;
                
            }
        }

    }
}

/* Thread starts automatically at boot */
K_THREAD_DEFINE(spi_slave_thread_id,
                SPI_SLAVE_STACK_SIZE,
                spi_slave_thread,
                NULL, NULL, NULL,
                SPI_SLAVE_PRIORITY,
                0,
                0);
