/*
 * demo.c -
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 *
 * Based on ../E407/demo.c
 */


#include "FreeRTOS.h"

#include "task.h"
//#include "partest.h"

#include STM32_CONF_H

#include "../E407/serial.h"

#include <stdio.h>


#define mainFLASH_TASK_PRIORITY		(tskIDLE_PRIORITY + 1UL)


static volatile uint16_t mask = 1;
static volatile GPIO_TypeDef *port = GPIOA;


static void serial_receive(const char *buf, unsigned len)
{
	while (len--) {
		switch (*buf++) {
		case '+':
			if (mask != 0x8000)
				mask <<= 1;
			break;
		case '-':
			if (mask != 0x0001)
				mask >>= 1;
			break;
		case 'a':
			port = GPIOA;
			break;
		case 'b':
			port = GPIOB;
			break;
		case 'c':
			port = GPIOC;
			break;
		}
		printf("0x%x\n", mask);
//		serial_send_isr(".", 1);
	}
//		serial_line_input_byte(*buf++);
}


static void toggle(void)
{
	while (1) {
		port->MODER |= mask*mask;
		GPIO_ToggleBits(port, mask);
		taskYIELD();
	}

}


int main(void)
{
	RCC_ClocksTypeDef clocks;

	/*
	 * Ensure all priority bits are assigned as preemption priority bits.
	 * (Taken verbatim from ../CORTEX_M4F_STM32F407ZG-SK/main.c)
	 */

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

//	vParTestInitialise();

	serial_recv = serial_receive;
	serial_init();

	printf("FreeRTOS Build: %s\n", FREERTOS_BUILD_DATE);

	RCC_GetClocksFreq(&clocks);
	printf("Clocks (MHz): "
	    "Crystal %.3f SYS %.3f MHz AHB %.3f APB1 %.3f APB2 %.3f\n",
	    HSE_VALUE/1e6,
	    clocks.SYSCLK_Frequency/1e6, clocks.HCLK_Frequency/1e6,
	    clocks.PCLK1_Frequency/1e6, clocks.PCLK2_Frequency/1e6);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	xTaskCreate((pdTASK_CODE) toggle, (const signed char *) "CoMa",
	  2000, NULL, tskIDLE_PRIORITY, NULL);

	vTaskStartScheduler();

	return 0;
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
