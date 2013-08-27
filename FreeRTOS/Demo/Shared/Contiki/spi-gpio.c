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


void spi_gpio_begin(void)
{
	CLR(nsel);
}


void spi_gpio_end(void)
{
	SET(nsel);
}


void spi_gpio_send(uint8_t v)
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


void spi_gpio_begin_rx(void)
{
}


uint8_t spi_gpio_recv(void)
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


void spi_gpio_init(void)
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
