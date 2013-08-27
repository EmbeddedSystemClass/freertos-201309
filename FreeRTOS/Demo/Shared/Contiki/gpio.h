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


/* ----- GPIO ID (single number to identify pin) --------------------------- */


static inline unsigned gpio_id(int port, int bit)
{
	return port << 5 | bit;
}


static inline unsigned gpio_id_port(unsigned id)
{
	return id >> 5;
}


static inline unsigned gpio_id_bit(unsigned id)
{
	return id & 31;
}


/* ----- Pin operations ---------------------------------------------------- */


extern GPIO_TypeDef *const gpiox[];


#define	OUT(id)	gpio_inout(id, 1)
#define	IN(id)	gpio_inout(id, 0)
#define	SET(id)	GPIO_SetBits(gpiox[gpio_id_port(id)], 1 << gpio_id_bit(id))
#define	CLR(id)	GPIO_ResetBits(gpiox[gpio_id_port(id)], 1 << gpio_id_bit(id))

#define	PIN(id)	(GPIO_ReadInputDataBit(gpiox[gpio_id_port(id)], \
		    1 << gpio_id_bit(id)) == Bit_SET)


/* ----- Setup ------------------------------------------------------------- */


#define	GPIO_AF_SPI(id)			gpio_af_spi(id)
#define	GPIO_ENABLE(pin)		gpio_enable(PORT_##pin, BIT_##pin)
#define	GPIO_DISABLE(id)		gpio_disable(id)


int gpio_num(GPIO_TypeDef *gpio);

void gpio_inout(unsigned id, bool out);
void gpio_af_spi(unsigned id);

unsigned gpio_enable(GPIO_TypeDef *gpio, int bit);
void gpio_disable(unsigned id);

void gpio_enable(GPIO_TypeDef *gpio, int bit);
void gpio_disable(GPIO_TypeDef *gpio, int bit);

#endif /* !GPIO_H */
