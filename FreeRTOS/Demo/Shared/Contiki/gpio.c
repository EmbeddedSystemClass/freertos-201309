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


/* ----- RCC_AHB1Periph_GPIOx value (by GPIO bank index) ------------------- */


#define	GPIO_BANKS	(sizeof(gpio_rcc) / sizeof(*gpio_rcc))


static const uint32_t gpio_rcc[] = {
#ifdef GPIOA
	RCC_AHB1Periph_GPIOA,
#endif
#ifdef GPIOB
	RCC_AHB1Periph_GPIOB,
#endif
#ifdef GPIOC
	RCC_AHB1Periph_GPIOC,
#endif
#ifdef GPIOD
	RCC_AHB1Periph_GPIOD,
#endif
#ifdef GPIOE
	RCC_AHB1Periph_GPIOE,
#endif
#ifdef GPIOF
	RCC_AHB1Periph_GPIOF,
#endif
#ifdef GPIOG
	RCC_AHB1Periph_GPIOG,
#endif
#ifdef GPIOH
	RCC_AHB1Periph_GPIOH,
#endif
#ifdef GPIOI
	RCC_AHB1Periph_GPIOI,
#endif
};


/* ----- In/out/function configuration ------------------------------------- */


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


/* ----- GPIO enable/disable ----------------------------------------------- */


/* Number of uses of a GPIO bank. 0 if disabled.  */

static int gpio_enabled[GPIO_BANKS];


int gpio_num(GPIO_TypeDef *gpio)
{
#ifdef GPIOA
	if (gpio == GPIOA)
		return 0;
#endif
#ifdef GPIOB
	if (gpio == GPIOB)
		return 1;
#endif
#ifdef GPIOC
	if (gpio == GPIOC)
		return 2;
#endif
#ifdef GPIOD
	if (gpio == GPIOD)
		return 3;
#endif
#ifdef GPIOE
	if (gpio == GPIOE)
		return 4;
#endif
#ifdef GPIOF
	if (gpio == GPIOF)
		return 5;
#endif
#ifdef GPIOG
	if (gpio == GPIOG)
		return 6;
#endif
#ifdef GPIOH
	if (gpio == GPIOH)
		return 7;
#endif
#ifdef GPIOI
	if (gpio == GPIOI)
		return 8;
#endif
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
