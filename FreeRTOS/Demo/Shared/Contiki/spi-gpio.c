/*
 * spi-gpio.c - Bit-banging SPI interface
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */


#include <stdint.h>

#include "gpio.h"
#include "spi-gpio.h"


void spi_begin(const struct spi *spi)
{
	CLR(spi->nsel);
}


void spi_end(const struct spi *spi)
{
	SET(spi->nsel);
}


void spi_send(const struct spi *spi, uint8_t v)
{
	uint8_t mask;

	for (mask = 0x80; mask; mask >>= 1) {
		if (v & mask)
			SET(spi->mosi);
		else
			CLR(spi->mosi);
		SET(spi->sclk);
		SET(spi->sclk);
		SET(spi->sclk);
		CLR(spi->sclk);
	}
}


void spi_begin_rx(const struct spi *spi)
{
}


uint8_t spi_recv(const struct spi *spi)
{
	uint8_t res = 0;
	uint8_t mask;

	for (mask = 0x80; mask; mask >>= 1) {
		if (PIN(spi->miso))
			res |= mask;
		SET(spi->sclk);
		SET(spi->sclk);
		SET(spi->sclk);
		CLR(spi->sclk);
	}
	return res;
}


void spi_init(const struct spi *spi)
{
	CLR(spi->sclk);
	SET(spi->nsel);

	OUT(spi->mosi);
	IN(spi->miso);
	OUT(spi->sclk);
	OUT(spi->nsel);
}
