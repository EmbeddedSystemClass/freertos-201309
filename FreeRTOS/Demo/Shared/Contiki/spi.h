/*
 * spi.h - General SPI interface
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

#ifndef SPI_H
#define	SPI_H

#include <stdint.h>


void spi_begin(void);
void spi_end(void);
void spi_send(uint8_t v);
void spi_begin_rx(void);
uint8_t spi_recv(void);
void spi_init(void);

#endif /* !SPI_H */
