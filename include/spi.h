#ifndef SPI_H
#define SPI_H

#include <stdint.h>

/* Define a callback type for when SPI data arrives.
 * We pass the buffer and its length to main.c 
 */
typedef void (*spi_rx_callback_t)(const uint8_t *data, uint32_t len);

void spi_init(void);
void spi_set_callback(spi_rx_callback_t cb);

#endif /* SPI_H */