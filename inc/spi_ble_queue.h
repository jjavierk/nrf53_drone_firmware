#ifndef _SPI_BLE_QUEUE_H_
#define _SPI_BLE_QUEUE_H_

#include <zephyr/kernel.h>


#define SPI_PACKET_MAX_LEN  256      // or whatever max you expect (ret-2)
#define SPI_PACKET_QUEUE_LEN 8       // number of packets buffered

struct spi_packet {
    uint16_t len;
    uint8_t  data[SPI_PACKET_MAX_LEN];
};

#endif