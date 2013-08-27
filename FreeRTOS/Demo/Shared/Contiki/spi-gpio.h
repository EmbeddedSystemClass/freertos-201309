/*
 * spi-gpio.h - Bit-banging SPI interface
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

#ifndef SPI_GPIO_H
#define	SPI_GPIO_H

struct spi {
	unsigned mosi, miso, sclk, nsel;
};

#endif /* !SPI_GPIO_H */
