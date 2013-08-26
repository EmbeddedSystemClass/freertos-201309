/*
 * gpio.c - GPIO interface
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */


#include <stdbool.h>
#include <stdint.h>

#include STM32_CONF_H

#include "platform.h"
#include "gpio.h"



void gpio_inout(GPIO_TypeDef *GPIOx, uint16_t pins, bool out)
{
	GPIO_InitTypeDef gpio_init = {
		.GPIO_Pin       = pins,
		.GPIO_Mode      = out ? GPIO_Mode_OUT : GPIO_Mode_IN,
		.GPIO_Speed     = GPIO_Speed_25MHz,
		.GPIO_OType     = GPIO_OType_PP,
		.GPIO_PuPd      = GPIO_PuPd_NOPULL,
	};

	GPIO_Init(GPIOx, &gpio_init);
}


#ifdef SPI_AF

void gpio_af_spi(GPIO_TypeDef *gpio, int bit)
{
	GPIO_InitTypeDef gpio_init = {
		.GPIO_Pin	= 1 << bit,
		.GPIO_Mode	= GPIO_Mode_AF,
		.GPIO_Speed	= GPIO_Speed_25MHz,
		.GPIO_OType	= GPIO_OType_PP,
		.GPIO_PuPd	= GPIO_PuPd_DOWN,
	};

	GPIO_PinAFConfig(gpio, bit, SPI_AF);
	GPIO_Init(gpio, &gpio_init);
}

#endif /* SPI_AF */
