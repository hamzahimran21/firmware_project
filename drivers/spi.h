/*
 * SPI driver for ATSAMS70Q21
 * SPI master mode for Avian radar sensor communication
 */

#ifndef SPI_H
#define SPI_H

#include <stdint.h>

/*
 * Initialize SPI0 in master mode
 * Clock speed: ~10 MHz (suitable for Avian sensor)
 * Mode: CPOL=0, CPHA=0 (SPI Mode 0)
 */
void spi_init(void);

/*
 * Transfer a single byte (read/write)
 */
uint8_t spi_transfer(uint8_t data);

/*
 * Transfer multiple bytes
 */
void spi_transfer_buffer(const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len);

/*
 * Manual chip select control for burst transfers
 */
void spi_select(void);
void spi_deselect(void);

#endif /* SPI_H */
