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
	unsigned nsel;
};

#endif /* !SPI_STM32_H */
