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


#define	GPIO_BANKS	9	/* A through I */


static const uint32_t gpio_rcc[] = {
	RCC_AHB1Periph_GPIOA,	RCC_AHB1Periph_GPIOB,	RCC_AHB1Periph_GPIOC,
	RCC_AHB1Periph_GPIOD,	RCC_AHB1Periph_GPIOE,	RCC_AHB1Periph_GPIOF,
	RCC_AHB1Periph_GPIOG,	RCC_AHB1Periph_GPIOH,	RCC_AHB1Periph_GPIOI,
};

/* Number of uses of a GPIO bank. 0 if disabled.  */

static int gpio_enabled[GPIO_BANKS];


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


static int gpio_num(GPIO_TypeDef *gpio)
{
	if (gpio == GPIOA)
		return 0;
	if (gpio == GPIOB)
		return 1;
	if (gpio == GPIOC)
		return 2;
	if (gpio == GPIOD)
		return 3;
	if (gpio == GPIOE)
		return 4;
	if (gpio == GPIOF)
		return 5;
	if (gpio == GPIOG)
		return 6;
	if (gpio == GPIOH)
		return 7;
	if (gpio == GPIOI)
		return 8;
	return -1;
}


void gpio_enable(GPIO_TypeDef *gpio, int bit)
{
	int n;

	n = gpio_num(gpio);
	if (!gpio_enabled[n]++)
		RCC_AHB1PeriphClockCmd(gpio_rcc[n], ENABLE);
}


void gpio_disable(GPIO_TypeDef *gpio, int bit)
{
	int n;

	n = gpio_num(gpio);
	if (!--gpio_enabled[n])
		RCC_AHB1PeriphClockCmd(gpio_rcc[n], DISABLE);
}
