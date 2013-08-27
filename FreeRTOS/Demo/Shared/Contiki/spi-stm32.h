/*
 * spi-stm32.h - Hardware-assisted SPI interface (STM32)
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

#ifndef SPI_STM32_H
#define	SPI_STM32_H

#include <stdint.h>

#include STM32_CONF_H


void spi_begin(SPI_TypeDef *spi);
void spi_end(SPI_TypeDef *spi);
void spi_send(SPI_TypeDef *spi, uint8_t v);
void spi_begin_rx(SPI_TypeDef *spi);
uint8_t spi_recv(SPI_TypeDef *spi);
void spi_init(SPI_TypeDef *spi);

#endif /* !SPI_STM32_H */
