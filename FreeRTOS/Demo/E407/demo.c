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

#include STM32_CONF_H

#include "serial.h"
#include "eui64.h"
#include "contiki.h"

#include <stdio.h>


#define mainFLASH_TASK_PRIORITY		(tskIDLE_PRIORITY + 1UL)


static void serial_receive(const char *buf, unsigned len)
{
	while (len--)
		serial_line_input_byte(*buf++);
}


static void init_led(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIOC->MODER = 1 << 26;			/* make PC13 an output */
}


int main(void)
{
	RCC_ClocksTypeDef clocks;

	/*
	 * Ensure all priority bits are assigned as preemption priority bits.
	 *
	 * See http://www.freertos.org/RTOS-Cortex-M3-M4.html
	 * section "Preempt Priority and Subpriority".
	 */

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

//	vParTestInitialise();

	init_led();
	serial_recv = serial_receive;
	serial_init();

	printf("FreeRTOS Build: %s\n", FREERTOS_BUILD_DATE);

	RCC_GetClocksFreq(&clocks);
	printf("Clocks (MHz): "
	    "Crystal %.3f SYS %.3f MHz AHB %.3f APB1 %.3f APB2 %.3f\n",
	    HSE_VALUE/1e6,
	    clocks.SYSCLK_Frequency/1e6, clocks.HCLK_Frequency/1e6,
	    clocks.PCLK1_Frequency/1e6, clocks.PCLK2_Frequency/1e6);

	init_eui64();

	vStartLEDFlashTasks(mainFLASH_TASK_PRIORITY);

	xTaskCreate((pdTASK_CODE) contiki_main, (const signed char *) "CoMa",
	  2000, NULL, tskIDLE_PRIORITY, NULL);

	vTaskStartScheduler();

	return 0;
}


void vParTestToggleLED(unsigned long ulLED)
{
	if (ulLED == 0)
		GPIOC->ODR ^= 1 << 13;		/* toggle PC13 */
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
