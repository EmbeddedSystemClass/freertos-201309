/*
 * spi-stm32.h - Hardware-assisted SPI interface (STM32)
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

#ifndef SPI_STM32_H
#define	SPI_STM32_H

#include STM32_CONF_H


struct spi {
	SPI_TypeDef *dev;
	unsigned  mosi, miso, sclk, nsel;
};


#define	SPI_STM32_DEV(_dev, _mosi, _miso, _sclk, _nsel) \
	(struct spi) {				\
		.dev = _dev,			\
		.mosi = GPIO_ENABLE(_mosi),	\
		.miso = GPIO_ENABLE(_miso),	\
		.sclk = GPIO_ENABLE(_sclk),	\
		.nsel = GPIO_ENABLE(_nsel)	\
	}

#endif /* !SPI_STM32_H */
