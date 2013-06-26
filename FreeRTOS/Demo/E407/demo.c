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

#include "serial.h"

#include <stdio.h>


#define mainFLASH_TASK_PRIORITY		(tskIDLE_PRIORITY + 1UL)


extern void contiki_main(void);


static void init_led(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIOC->MODER = 1 << 26;			/* make PC13 an output */
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
	serial_init();

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
