/*
 * demo.c -
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 *
 * Based on ../CORTEX_R4_RM48_TMS570_CCS5/main.c and
 * ../CORTEX_M4F_STM32F407ZG/main.c
 */


#include "FreeRTOS.h"

#include "task.h"
//#include "partest.h"
#include "flash.h"

#include "stm32f4xx_conf.h"


#define mainFLASH_TASK_PRIORITY		(tskIDLE_PRIORITY + 1UL)


int main(void)
{
	/* Ensure all priority bits are assigned as preemption priority bits. */

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

//	vParTestInitialise();

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;	/* enable clock to GPIOC */
	GPIOC->MODER = 1 << 26;			/* make PC13 an output */

	vStartLEDFlashTasks(mainFLASH_TASK_PRIORITY);
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
