/*
 * demo.c -
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 *
 * Based on ../CORTEX_R4_RM48_TMS570_CCS5/main.c and
 * ../CORTEX_M4F_STM32F407ZG-SK/main.c
 */


#include "FreeRTOS.h"

#include "task.h"
//#include "partest.h"
#include "flash.h"

#include "stm32f4xx_conf.h"

#include <stdio.h>


#define mainFLASH_TASK_PRIORITY		(tskIDLE_PRIORITY + 1UL)


extern void contiki_main(void);


static void init_led(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIOC->MODER = 1 << 26;			/* make PC13 an output */
}


static void init_uart(void)
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


int main(void)
{
	/*
	 * Ensure all priority bits are assigned as preemption priority bits.
	 * (Taken verbatim from ../CORTEX_M4F_STM32F407ZG-SK/main.c)
	 */

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

//	vParTestInitialise();

	init_led();
	init_uart();

	printf("FreeRTOS Build: %s\n", FREERTOS_BUILD_DATE);

	vStartLEDFlashTasks(mainFLASH_TASK_PRIORITY);

	xTaskCreate((pdTASK_CODE) contiki_main, (const signed char *) "CoMa",
	  2000, NULL, tskIDLE_PRIORITY, NULL);

	vTaskStartScheduler();

	return 0;
}


void vParTestToggleLED(unsigned long ulLED)
{
	static int n = 0;

	if (ulLED == 0) {
		GPIOC->ODR ^= 1 << 13;		/* toggle PC13 */
		printf("hello %d\n", n++);
	}
}


void vApplicationTickHook(void)
{
}


void vApplicationIdleHook(void)
{
}


void vApplicationMallocFailedHook(void)
{
	/* @@@ */
}


void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
	/* @@@ */
}
