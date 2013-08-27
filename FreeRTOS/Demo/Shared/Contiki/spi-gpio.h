/*
 * spi-gpio.h - Bit-banging SPI interface
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

#ifndef SPI_GPIO_H
#define	SPI_GPIO_H

#include <stdint.h>


#define	spi_begin(spi)		spi_gpio_begin()
#define	spi_end(spi)		spi_gpio_end()
#define	spi_send(spi, v)	spi_gpio_send(v)
#define	spi_begin_rx(spi)	spi_gpio_begin_rx()
#define	spi_recv(spi)		spi_gpio_recv()
#define	spi_init(spi)		spi_gpio_init()

void spi_gpio_begin(void);
void spi_gpio_end(void);
void spi_gpio_send(uint8_t v);
void spi_gpio_begin_rx(void);
uint8_t spi_gpio_recv(void);
void spi_gpio_init(void);

#endif /* !SPI_GPIO_H */
