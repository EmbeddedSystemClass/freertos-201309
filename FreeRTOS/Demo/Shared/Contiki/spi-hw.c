/*
 * spi-hw.c - Hardware-assisted SPI interface
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */


#include <stdint.h>

#include STM32_CONF_H

#include "gpio.h"
#include "spi.h"

#include "platform.h"


/* ----- RCC_APBxPeriphClockCmd function (by SPI device index) ------------- */


typedef void (*spi_rcc_fnp)(uint32_t RCC_APB2Periph, FunctionalState NewState);


static spi_rcc_fnp spi_rcc_fn[] = {
#ifdef SPI1
	RCC_APB2PeriphClockCmd,
#endif
#ifdef SPI2
	RCC_APB1PeriphClockCmd,
#endif
#ifdef SPI3
	RCC_APB1PeriphClockCmd,
#endif
#ifdef SPI4
	RCC_APB2PeriphClockCmd,
#endif
#ifdef SPI5
	RCC_APB2PeriphClockCmd,
#endif
#ifdef SPI6
	RCC_APB2PeriphClockCmd,
#endif
};


/* ----- RCC_APB2Periph_SPIx value (by SPI device index) ------------------- */


static const uint32_t spi_rcc[] = {
#ifdef SPI1
	RCC_APB2Periph_SPI1,
#endif
#ifdef SPI2
	RCC_APB1Periph_SPI2,
#endif
#ifdef SPI3
	RCC_APB1Periph_SPI3,
#endif
#ifdef SPI4
	RCC_APB2Periph_SPI4,
#endif
#ifdef SPI5
	RCC_APB2Periph_SPI5,
#endif
#ifdef SPI6
	RCC_APB2Periph_SPI6,
#endif
};


/* ----- SPI communication ------------------------------------------------- */


static void inline delay_1us(void)
{
	IN(MISO);
}


void spi_begin(void)
{
	CLR(nSEL);
}


void spi_end(void)
{
	/*
	 * RM0090 Reference manual, 27.3.9: "[...] As a consequence, it is
	 * mandatory to wait first until TXE=1 and then until BSY=0 after
	 * writing the last data."
	 */

	while (SPI_I2S_GetFlagStatus(SPI_DEV, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI_DEV, SPI_I2S_FLAG_BSY) == SET);
	SET(nSEL);
}


void spi_send(uint8_t v)
{
	SPI_I2S_SendData(SPI_DEV, v);
	while (SPI_I2S_GetFlagStatus(SPI_DEV, SPI_I2S_FLAG_TXE) == RESET);
}


void spi_begin_rx(void)
{
	/*
	 * If we did a series of writes, we'll have overrun the receiver.
	 * RM0090 section 27.3.10, "Overrun condition", claims that we need to
	 * clear OVR in order to proceed. (Experiments suggest that it's not
	 * necessary, but better safe than sorry.)
	 *
	 * We also have to discard any stale data sitting in the receiver.
	 */

	while (SPI_I2S_GetFlagStatus(SPI_DEV, SPI_I2S_FLAG_BSY) == SET);
	(void) SPI_I2S_GetFlagStatus(SPI_DEV, SPI_I2S_FLAG_OVR);
	(void) SPI_I2S_ReceiveData(SPI_DEV);
}


uint8_t spi_recv(void)
{
	/*
	 * The AT86RF231 requires a delay of 250 ns between the LSB of the
	 * preceding byte and the MSB of a byte being received. We use 1 us
	 * here because that's the delay a port read produces.
	 */

	delay_1us();

	SPI_I2S_SendData(SPI_DEV, 0);
	while (SPI_I2S_GetFlagStatus(SPI_DEV, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(SPI_DEV);
}


/* ----- Initialization ---------------------------------------------------- */


static int spi_to_num(SPI_TypeDef *spi)
{
#ifdef SPI1
	if (spi == SPI1)
		return 0;
#endif
#ifdef SPI2
	if (spi == SPI2)
		return 1;
#endif
#ifdef SPI3
	if (spi == SPI3)
		return 2;
#endif
#ifdef SPI4
	if (spi == SPI4)
		return 3;
#endif
#ifdef SPI5
	if (spi == SPI5)
		return 4;
#endif
#ifdef SPI6
	if (spi == SPI6)
		return 5;
#endif
	return -1;
}


void spi_init(void)
{
	static SPI_InitTypeDef spi_init = {
		.SPI_Direction	= SPI_Direction_2Lines_FullDuplex,
		.SPI_Mode	= SPI_Mode_Master,
		.SPI_DataSize	= SPI_DataSize_8b,
		.SPI_CPOL	= SPI_CPOL_Low,
		.SPI_CPHA	= SPI_CPHA_1Edge,
		.SPI_NSS	= SPI_NSS_Soft,
		.SPI_BaudRatePrescaler = SPI_PRESCALER,
		.SPI_FirstBit	= SPI_FirstBit_MSB,
	};
	int n;

	GPIO_ENABLE(MOSI);
	GPIO_ENABLE(MISO);
	GPIO_ENABLE(SCLK);
	GPIO_ENABLE(nSEL);

	GPIO_AF_SPI(MOSI);
	GPIO_AF_SPI(MISO);
	GPIO_AF_SPI(SCLK);

	SET(nSEL);

	OUT(nSEL);

	n = spi_to_num(SPI_DEV);
	spi_rcc_fn[n](spi_rcc[n], ENABLE);

	SPI_Init(SPI_DEV, &spi_init);
	SPI_Cmd(SPI_DEV, ENABLE);
}
