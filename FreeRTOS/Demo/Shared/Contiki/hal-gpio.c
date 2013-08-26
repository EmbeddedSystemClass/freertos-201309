/*
 * hal-gpio.c - Substitute for cpu/avr/radio/rf230bb/halbb.c (atben on STM32-E407)
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

#include "platform.h"


#define	AT86RF230_REG_READ	0x80
#define	AT86RF230_REG_WRITE	0xc0
#define	AT86RF230_BUF_READ	0x20
#define	AT86RF230_BUF_WRITE	0x60

#define	IRQ_TRX_END		0x08


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


static void spi_begin(void)
{
	CLR(nSEL);
}


static void spi_end(void)
{
	SET(nSEL);
}


static void spi_send(uint8_t v)
{
	uint8_t mask;

	for (mask = 0x80; mask; mask >>= 1) {
		if (v & mask)
			SET(MOSI);
		else
			CLR(MOSI);
		SET(SCLK);
		SET(SCLK);
		SET(SCLK);
		CLR(SCLK);
	}
}


static uint8_t spi_recv(void)
{
	uint8_t res = 0;
	uint8_t mask;

	for (mask = 0x80; mask; mask >>= 1) {
		if (PIN(MISO))
			res |= mask;
		SET(SCLK);
		SET(SCLK);
		SET(SCLK);
		CLR(SCLK);
	}
	return res;
}


/* ----- Register access --------------------------------------------------- */


static uint8_t register_read_unsafe(uint8_t address)
{
	uint8_t res;

	spi_begin();
	spi_send(AT86RF230_REG_READ | address);
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


static void frame_read_unsafe(hal_rx_frame_t *rx_frame)
{
	uint8_t *buf = rx_frame->data;
	uint8_t i;

	spi_begin();
	spi_send(AT86RF230_BUF_READ);
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

	enable_spi_clocks();

	CLR(SCLK);
	SET(nSEL);
	CLR(SLP_TR);

	OUT(MOSI);
	IN(MISO);
	OUT(SCLK);
	OUT(nSEL);
	OUT(SLP_TR);
	IN(IRQ);

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
