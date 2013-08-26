/*
 * gpio.h - GPIO interface
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

/*
 * @@@ WIP: very STM32 specific
 */

#ifndef GPIO_H
#define	GPIO_H

#include <stdbool.h>
#include <stdint.h>

#include STM32_CONF_H


#define	OUT(pin)	gpio_inout(PORT_##pin, 1 << BIT_##pin, 1)
#define	IN(pin)		gpio_inout(PORT_##pin, 1 << BIT_##pin, 0)
#define	SET(pin)	GPIO_SetBits(PORT_##pin, 1 << BIT_##pin)
#define	CLR(pin)	GPIO_ResetBits(PORT_##pin, 1 << BIT_##pin)

#define	PIN(pin)	(GPIO_ReadInputDataBit(PORT_##pin, 1 << BIT_##pin) \
			    == Bit_SET)

#define	EXTI_PinSource_CONCAT(n)	EXTI_PinSource##n
#define	EXTI_PinSource(n)		EXTI_PinSource_CONCAT(n)

#define	EXTI_Line_CONCAT(n)		EXTI_Line##n
#define	EXTI_Line(n)			EXTI_Line_CONCAT(n)

#define	GPIO_AF_SPI(pin)		gpio_af_spi(PORT_##pin, BIT_##pin)


void gpio_inout(GPIO_TypeDef *GPIOx, uint16_t pins, bool out);
void gpio_af_spi(GPIO_TypeDef *gpio, int bit);

#endif /* !GPIO_H */
