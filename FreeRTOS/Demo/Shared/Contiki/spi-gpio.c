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


static unsigned mosi, miso, sclk, nsel;


void spi_gpio_begin(struct spi *spi)
{
	CLR(nsel);
}


void spi_gpio_end(struct spi *spi)
{
	SET(nsel);
}


void spi_gpio_send(struct spi *spi, uint8_t v)
{
	uint8_t mask;

	for (mask = 0x80; mask; mask >>= 1) {
		if (v & mask)
			SET(mosi);
		else
			CLR(mosi);
		SET(sclk);
		SET(sclk);
		SET(sclk);
		CLR(sclk);
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
		if (PIN(miso))
			res |= mask;
		SET(sclk);
		SET(sclk);
		SET(sclk);
		CLR(sclk);
	}
	return res;
}


void spi_gpio_init(struct spi *spi)
{
	mosi = GPIO_ENABLE(MOSI);
	miso = GPIO_ENABLE(MISO);
	sclk = GPIO_ENABLE(SCLK);
	nsel = GPIO_ENABLE(nSEL);

	CLR(sclk);
	SET(nsel);

	OUT(mosi);
	IN(miso);
	OUT(sclk);
	OUT(nsel);
}
