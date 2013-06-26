/*
 * serial.c - Serial console
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */


#include "FreeRTOS.h"

#include "task.h"
//#include "partest.h"
#include "flash.h"

#include "stm32f4xx_conf.h"

#include "serial.h"


extern void (*serial_recv)(const char *buf, unsigned n);


void serial_send_isr(const char *buf, unsigned len)
{
	while (len--) {
		while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
		USART_SendData(USART6, *buf++);
	}
}


void serial_send(const char *buf, unsigned len)
{
	serial_send_isr(buf, len);
}


void serial_init(void)
{
	static USART_InitTypeDef uart_init = {
		.USART_BaudRate		= 38400,
		.USART_WordLength	= USART_WordLength_8b,
		.USART_StopBits		= USART_StopBits_1,
		.USART_Parity		= USART_Parity_No,
		.USART_Mode		= USART_Mode_Tx,
		.USART_HardwareFlowControl = USART_HardwareFlowControl_None,
	};
	static GPIO_InitTypeDef gpio_init = {
		.GPIO_Pin	= GPIO_Pin_6,
		.GPIO_Mode	= GPIO_Mode_AF,
		.GPIO_Speed	= GPIO_Speed_2MHz,
		.GPIO_OType	= GPIO_OType_PP,
		.GPIO_PuPd	= GPIO_PuPd_NOPULL,
	};


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_Init(GPIOC, &gpio_init);

	USART_Init(USART6, &uart_init);
	USART_Cmd(USART6, ENABLE);
}
