/*
 * hal-spi.c - SPI-based HAL
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include STM32_CONF_H

/* @@@ contiki/contiki/ because we're on top of contiki-outoftree */
#include "contiki/contiki/cpu/avr/radio/rf230bb/at86rf230_registermap.h"
#include "contiki/hal.h"

#include "contiki.h"


#define	AT86RF230_REG_READ	0x80
#define	AT86RF230_REG_WRITE	0xc0
#define	AT86RF230_BUF_READ	0x20
#define	AT86RF230_BUF_WRITE	0x60

#define	IRQ_TRX_END		0x08


/* ----- I/O pin definitions ----------------------------------------------- */


/*
 * SD/MMC pin	atben signal	GPIO
 *				STM32-E407+odev	WM09+dev9
 * ----------	------------	---------------	---------------
 * DAT2		IRQ		PG10		PA0
 * DAT3		nSEL		PB9		PC5
 * CMD		MOSI		PC3 / SPI2_MOSI	PA7 / SPI1_MOSI
 * CLK		SLP_TR		PB8		PA3
 * DAT0		MISO		PC2 / SPI2_MISO	PA6 / SPI1_MISO
 * DAT1		SCLK		PB10 / SPI2_SCK	PA5 / SPI1_SCK
 */


/* ----- ODEV settings ----------------------------------------------------- */


#ifdef ODEV

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


static void enable_clocks(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
}

#endif /* ODEV */


/* ----- DEV9 settings ----------------------------------------------------- */


#ifdef DEV9

#define	PORT_IRQ	GPIOA
#define	BIT_IRQ		0
#define	PORT_nSEL	GPIOC
#define	BIT_nSEL	5
#define	PORT_MOSI	GPIOA
#define	BIT_MOSI	7
#define	PORT_SLP_TR	GPIOA
#define	BIT_SLP_TR	3
#define	PORT_MISO	GPIOA
#define	BIT_MISO	6
#define	PORT_SCLK	GPIOA
#define	BIT_SCLK	5

#define	SPI_DEV		SPI1
#define	SPI_AF		GPIO_AF_SPI1

#define	SPI_PRESCALER	SPI_BaudRatePrescaler_8
			/* APB2 = 60 MHz; 60 MHz / 8 = 7.5 MHz */

#define	EXTI_PortSource	EXTI_PortSourceGPIOA

#define	IRQn		EXTI0_IRQn
#define	IRQ_HANDLER	EXTI0_IRQHandler


static void enable_clocks(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
}

#endif /* DEV9 */


/* ----- Helper macros ----------------------------------------------------- */


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


/* ----- GPIO helper functions --------------------------------------------- */


static void gpio_inout(GPIO_TypeDef *GPIOx, uint16_t pins, bool out)
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


static void gpio_af_spi(GPIO_TypeDef *gpio, int bit)
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


/* ----- Items shared with rf230bb ----------------------------------------- */


extern hal_rx_frame_t rxframe[RF230_CONF_RX_BUFFERS];
extern uint8_t rxframe_head, rxframe_tail;


/* ----- Control signals --------------------------------------------------- */


void hal_set_rst_low(void)
{
	/* not supported by hardware */
	hal_register_read(RG_IRQ_STATUS);
}


void hal_set_rst_high(void)
{
	/* not supported by hardware */
}


void hal_set_slptr_high(void)
{
	SET(SLP_TR);
}


void hal_set_slptr_low(void)
{
	CLR(SLP_TR);
}


bool hal_get_slptr(void)
{
	return PIN(SLP_TR);
}


/* ----- SPI bit-banging --------------------------------------------------- */


static void inline delay_1us(void)
{
	IN(IRQ);
}


static void spi_begin(void)
{
	CLR(nSEL);
}


static void spi_end(void)
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


static void spi_send(uint8_t v)
{
	SPI_I2S_SendData(SPI_DEV, v);
	while (SPI_I2S_GetFlagStatus(SPI_DEV, SPI_I2S_FLAG_TXE) == RESET);
}


static void spi_begin_rx(void)
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


static uint8_t spi_recv(void)
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


/* ----- Register access --------------------------------------------------- */


static uint8_t register_read_unsafe(uint8_t address)
{
	uint8_t res;

	spi_begin();
	spi_send(AT86RF230_REG_READ | address);
	spi_begin_rx();
	res = spi_recv();
	spi_end();
	return res;
}


uint8_t hal_register_read(uint8_t address)
{
	uint8_t res;

	HAL_ENTER_CRITICAL_REGION();
	res = register_read_unsafe(address);
	HAL_LEAVE_CRITICAL_REGION();
	return res;
}


static void register_write_unsafe(uint8_t address, uint8_t value)
{
	spi_begin();
	spi_send(AT86RF230_REG_WRITE | address);
	spi_send(value);
	spi_end();
}


void hal_register_write(uint8_t address, uint8_t value)
{
	HAL_ENTER_CRITICAL_REGION();
	register_write_unsafe(address, value);
	HAL_LEAVE_CRITICAL_REGION();
}


static uint8_t subregister_read_unsafe(uint8_t address, uint8_t mask,
    uint8_t position)
{
	return (register_read_unsafe(address) & mask) >> position;
}


uint8_t hal_subregister_read(uint8_t address, uint8_t mask, uint8_t position)
{
	return (hal_register_read(address) & mask) >> position;
}


void hal_subregister_write(uint8_t address, uint8_t mask, uint8_t position,
    uint8_t value)
{
	uint8_t reg;

	HAL_ENTER_CRITICAL_REGION();
	reg = register_read_unsafe(address);
	reg = (reg & ~mask) | (value << position);
	hal_register_write(address, reg);
	HAL_LEAVE_CRITICAL_REGION();
}


/* ----- Buffer access ----------------------------------------------------- */


/*
 * Note that frame_read_unsafe can block for up to about 500 us.
 *
 * 131 Bytes * 3.6 us/Byte = 472 us
 *
 * The 3.6 us/Byte rate was obtained by measurement, see
 * frtos-wpan/lab/atben-spi/spi-buf-read.png
 */

static void frame_read_unsafe(hal_rx_frame_t *rx_frame)
{
	uint8_t *buf = rx_frame->data;
	uint8_t i;

	spi_begin();
	spi_send(AT86RF230_BUF_READ);
	spi_begin_rx();
	rx_frame->length = spi_recv();
	if (rx_frame->length > HAL_MAX_FRAME_LENGTH)
		rx_frame->length = HAL_MAX_FRAME_LENGTH;
	for (i = 0; i != rx_frame->length; i++)
		*(uint8_t *) buf++ = spi_recv();
	rx_frame->lqi = spi_recv();
        spi_end();
	rx_frame->crc = true;	/* checked by hardware */
}


void hal_frame_read(hal_rx_frame_t *rx_frame)
{
	HAL_ENTER_CRITICAL_REGION();
	frame_read_unsafe(rx_frame);
	HAL_LEAVE_CRITICAL_REGION();
}


/*
 * Note that hal_frame_write can block for up to about 200 us:
 *
 * 130 Bytes * 8 / 5.25 Mbps = 198 us
 */

void hal_frame_write(uint8_t *write_buffer, uint8_t length)
{
	HAL_ENTER_CRITICAL_REGION();
	spi_begin();
	spi_send(AT86RF230_BUF_WRITE);
	spi_send(length);
	while (length--)
		spi_send(*(uint8_t *) write_buffer++);
	spi_end();
	HAL_LEAVE_CRITICAL_REGION();
}


void hal_sram_read(uint8_t address, uint8_t length, uint8_t *data)
{
	/* not used */
}


void hal_sram_write(uint8_t address, uint8_t length, uint8_t *data)
{
	/* not used */
}


/* ----- Interrupts -------------------------------------------------------- */


static void exti_setup(bool enable)
{
	EXTI_InitTypeDef exti_init = {
		.EXTI_Line	= EXTI_Line(BIT_IRQ),
		.EXTI_Mode	= EXTI_Mode_Interrupt,
		.EXTI_Trigger	= EXTI_Trigger_Rising,
		.EXTI_LineCmd	= enable ? ENABLE : DISABLE,
	};

	EXTI_Init(&exti_init);
}


void hal_enable_trx_interrupt(void)
{
	exti_setup(1);
}


void hal_disable_trx_interrupt(void)
{
	exti_setup(0);
}


void IRQ_HANDLER(void)
{
	uint8_t irq, state;

	EXTI_ClearITPendingBit(EXTI_Line(BIT_IRQ));
	irq = register_read_unsafe(RG_IRQ_STATUS);

	if (!(irq & IRQ_TRX_END))
		return;

	/* @@@ record RSSI ? */
	/* @@@ check power level ? */
	/* @@@ make BAT_LOW one-shot ? */

	state = subregister_read_unsafe(SR_TRX_STATUS);
	if (state == BUSY_RX_AACK || state == RX_ON || state == BUSY_RX ||
	     state == RX_AACK_ON) {
		frame_read_unsafe(&rxframe[rxframe_tail]);
		rxframe_tail++;
		if (rxframe_tail >= RF230_CONF_RX_BUFFERS)
			rxframe_tail = 0;
		rf230_interrupt();
	}
}


/* ----- Critical sections (general) --------------------------------------- */



void HAL_ENTER_CRITICAL_REGION(void)
{
	taskENTER_CRITICAL();
}


void HAL_LEAVE_CRITICAL_REGION(void)
{
	taskEXIT_CRITICAL();
}


/* ----- Initialization ---------------------------------------------------- */


void hal_init(void)
{
	static NVIC_InitTypeDef nvic_init = {
		.NVIC_IRQChannel	= IRQn,
		.NVIC_IRQChannelPreemptionPriority = 8,	/* 0-15; @@@ ? */
		.NVIC_IRQChannelSubPriority = 8,	/* 0-15; @@@ ? */
		.NVIC_IRQChannelCmd	= ENABLE,
	};
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

	enable_clocks();

	GPIO_AF_SPI(MOSI);
	GPIO_AF_SPI(MISO);
	GPIO_AF_SPI(SCLK);

	SET(nSEL);
	CLR(SLP_TR);

	OUT(nSEL);
	OUT(SLP_TR);
	IN(IRQ);

	SPI_Init(SPI_DEV, &spi_init);
	SPI_Cmd(SPI_DEV, ENABLE);

	hal_register_read(RG_IRQ_STATUS);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	NVIC_Init(&nvic_init);
	SYSCFG_EXTILineConfig(EXTI_PortSource, EXTI_PinSource(BIT_IRQ));
	hal_enable_trx_interrupt();

	/*
	 * rf230bb.c will force the transceiver into TRX_OFF, so we're probably
	 * good despite not being able to do a proper hardware reset.
	 */
}


void hal_test(void)
{
	uint8_t pn, vn;
	uint8_t m0, m1;

	hal_init();
	pn = hal_register_read(RG_PART_NUM);
	vn = hal_register_read(RG_VERSION_NUM);
	m0 = hal_register_read(RG_MAN_ID_0);
	m1 = hal_register_read(RG_MAN_ID_1);
	printf("part 0x%02x revision 0x%02x manufacturer xxxx%02x%02x\n",
	    pn, vn, m1, m0);
}
