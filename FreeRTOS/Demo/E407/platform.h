/*
 * platform.h - Platform-specific definitions
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */


#ifndef PLATFORM_H
#define	PLATFORM_H

#include STM32_CONF_H


/* ----- ODEV settings ----------------------------------------------------- */

/*
 * SD/MMC pin	atben signal	GPIO (STM32-E407+odev)
 * ----------	------------	----------------------
 * DAT2		IRQ		PG10
 * DAT3		nSEL		PB9
 * CMD		MOSI		PC3 / SPI2_MOSI
 * CLK		SLP_TR		PB8
 * DAT0		MISO		PC2 / SPI2_MISO
 * DAT1		SCLK		PB10 / SPI2_SCK
 */


#define	PORT_IRQ	GPIOG
#define	BIT_IRQ		10
#define	PORT_nSEL	GPIOB
#define	BIT_nSEL	9
#define	PORT_MOSI	GPIOC
#define	BIT_MOSI	3
#define	PORT_SLP_TR	GPIOB
#define	BIT_SLP_TR	8
#define	PORT_MISO	GPIOC
#define	BIT_MISO	2
#define	PORT_SCLK	GPIOB
#define	BIT_SCLK	10

#define	SPI_DEV		SPI2
#define	SPI_AF		GPIO_AF_SPI2

#define	SPI_PRESCALER	SPI_BaudRatePrescaler_8
			/* APB1 = 42 MHz; 42 MHz / 8 = 5.25 MHz */

#define	EXTI_PortSource	EXTI_PortSourceGPIOG

#define	IRQn		EXTI15_10_IRQn
#define	IRQ_HANDLER	EXTI15_10_IRQHandler


static inline void enable_spi_clocks(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
}

#endif /* !PLATFORM_H */
