/*
 * spi-gpio.c - Bit-banging SPI interface
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */


#include <stdint.h>

#include "gpio.h"
#include "spi.h"

#include "platform.h"


void spi_begin(void)
{
	CLR(nSEL);
}


void spi_end(void)
{
	SET(nSEL);
}


void spi_send(uint8_t v)
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


void spi_begin_rx(void)
{
}


uint8_t spi_recv(void)
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


void spi_init(void)
{
	CLR(SCLK);
	SET(nSEL);

	OUT(MOSI);
	IN(MISO);
	OUT(SCLK);
	OUT(nSEL);
}
