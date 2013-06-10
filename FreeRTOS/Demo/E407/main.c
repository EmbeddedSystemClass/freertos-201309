/*
 * main.c -
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 *
 * Based on ../CORTEX_R4_RM48_TMS570_CCS5/main.c and
 * ../CORTEX_M4F_STM32F407ZG/main.c
 */


#include <stdint.h>

#include "stm32f2xx_conf.h"


int main(void)
{
	uint32_t i;

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;	/* enable clock to GPIOC */
	GPIOC->MODER = 1 << 26;			/* make PC13 an output */
	while (1) {
		GPIOC->ODR ^= 1 << 13;		/* toggle PC13 */
		for (i = 0; i != 1000000; i++);
	}
	return 0;
}
