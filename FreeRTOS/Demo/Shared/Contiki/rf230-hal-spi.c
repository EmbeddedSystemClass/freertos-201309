/*
 * rf230-hal-spi.c - SPI-based HAL
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
#include "gpio.h"


#define	AT86RF230_REG_READ	0x80
#define	AT86RF230_REG_WRITE	0xc0
#define	AT86RF230_BUF_READ	0x20
#define	AT86RF230_BUF_WRITE	0x60

#define	IRQ_TRX_END		0x08


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
		.NVIC_IRQChannelSubPriority = 0,	/* not on FreeRTOS */
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

	enable_spi_clocks();

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
