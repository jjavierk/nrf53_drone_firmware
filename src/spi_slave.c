#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>
#include <bluetooth/services/nus.h>

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_slave);

#define SPI_SLAVE_STACK_SIZE 2048 
#define SPI_SLAVE_PRIORITY   5
#define SPI_RX_BUF_SIZE      500

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
uint8_t msg[100];
int16_t tx_size = 0;

static void spi_slave_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    if (!device_is_ready(spi_dev)) {
        printk("SPIS device not ready");
        return;
    }

    printk("SPI slave thread started");

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


        if (ret>0) {
            printk("spi_read ret=%d, len=%u\r\n", ret, rx_buf.len);
            ble_send(&spi_rx_buf[2], ret-2); // First two bytes are header + PCK size. TODO: add checks
        }

    }
}

void spi_slave_init(void)
{
    printk("spi_slave_init() via printk\n");
    LOG_INF("spi_slave_init() via LOG_INF");

    k_thread_create(&spi_slave_thread_data,
                spi_slave_stack,
                K_THREAD_STACK_SIZEOF(spi_slave_stack),
                spi_slave_thread,
                NULL, NULL, NULL,
                SPI_SLAVE_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&spi_slave_thread_data, "spi_slave");
}
