/*
 * spi-gpio.c - Bit-banging SPI interface
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */


#include <stdint.h>

#include "gpio.h"
#include "spi-gpio.h"

#include "platform.h"


void spi_gpio_begin(struct spi *spi)
{
	CLR(nSEL);
}


void spi_gpio_end(struct spi *spi)
{
	SET(nSEL);
}


void spi_gpio_send(struct spi *spi, uint8_t v)
{
	uint8_t mask;

	for (mask = 0x80; mask; mask >>= 1) {
		if (v & mask)
			SET(MOSI);
		else
			CLR(MOSI);
		SET(SCLK);
		SET(SCLK);
		SET(SCLK);
		CLR(SCLK);
	}
}


void spi_gpio_begin_rx(struct spi *spi)
{
}


uint8_t spi_gpio_recv(struct spi *spi)
{
	uint8_t res = 0;
	uint8_t mask;

	for (mask = 0x80; mask; mask >>= 1) {
		if (PIN(MISO))
			res |= mask;
		SET(SCLK);
		SET(SCLK);
		SET(SCLK);
		CLR(SCLK);
	}
	return res;
}


void spi_gpio_init(struct spi *spi)
{
	GPIO_ENABLE(MOSI);
	GPIO_ENABLE(MISO);
	GPIO_ENABLE(SCLK);
	GPIO_ENABLE(nSEL);

	CLR(SCLK);
	SET(nSEL);

	OUT(MOSI);
	IN(MISO);
	OUT(SCLK);
	OUT(nSEL);
}
