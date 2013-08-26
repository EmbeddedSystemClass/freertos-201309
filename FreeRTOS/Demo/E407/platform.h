/*
 * platform.h - Platform-specific definitions
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */


#ifndef PLATFORM_H
#define	PLATFORM_H

#include STM32_CONF_H


/* ----- SPI settings ------------------------------------------------------ */

/*
 * SD/MMC pin	atben signal	GPIO
 *				uSD		STM32-E407+odev
 * ----------	------------	-------------------------------
 * DAT2		IRQ		PC10		PG10
 * DAT3		nSEL		PC11		PB9
 * CMD		MOSI		PD2		PC3 / SPI2_MOSI
 * CLK		SLP_TR		PC12		PB8
 * DAT0		MISO		PC8		PC2 / SPI2_MISO
 * DAT1		SCLK		PC9		PB10 / SPI2_SCK
 */


/* ----- uSD slot ---------------------------------------------------------- */


#if !defined(ODEV_GPIO) && !defined(ODEV_SPI)

#define	PORT_IRQ	GPIOC
#define	BIT_IRQ		10
#define	PORT_nSEL	GPIOC
#define	BIT_nSEL	11
#define	PORT_MOSI	GPIOD
#define	BIT_MOSI	2
#define	PORT_SLP_TR	GPIOC
#define	BIT_SLP_TR	12
#define	PORT_MISO	GPIOC
#define	BIT_MISO	8
#define	PORT_SCLK	GPIOC
#define	BIT_SCLK	9


static inline void enable_spi_clocks(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
}

#endif /* !ODEV_GPIO && !ODEV_SPI */


/* ----- ODEV (GPIO or SPI) ------------------------------------------------ */


#if defined(ODEV_GPIO) || defined(ODEV_SPI)

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


static inline void enable_spi_clocks(void)
{
#ifdef ODEV_SPI
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
#endif
}

#endif /* ODEV_GPIO || ODEV_SPI */


/* ----- Common for all forms of SPI attachment ---------------------------- */


#define	IRQn		EXTI15_10_IRQn
#define	IRQ_HANDLER	EXTI15_10_IRQHandler

#define	EXTI_PortSource	EXTI_PortSourceGPIOG

#endif /* !PLATFORM_H */
