/*
 * spi-hw.h - Hardware-assisted SPI interface
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

#ifndef SPI_HW_H
#define	SPI_HW_H

#include <stdint.h>

#include STM32_CONF_H


void spi_begin(SPI_TypeDef *spi);
void spi_end(SPI_TypeDef *spi);
void spi_send(SPI_TypeDef *spi, uint8_t v);
void spi_begin_rx(SPI_TypeDef *spi);
uint8_t spi_recv(SPI_TypeDef *spi);
void spi_init(SPI_TypeDef *spi);

#endif /* !SPI_HW_H */
