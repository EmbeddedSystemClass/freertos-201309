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

/*
 * hack around toolchain issues
 *
 * If we let the toolchain pick libc functions freely, ld has an assertion
 * failure. If we add -static, we get a 0.5 MB monster. -nostdlib brings out
 * these nasties.
 */

void *memcpy(void *dest, const void *src, size_t n)
{
	void *ret = dest;

	while (n--)
		*(uint8_t *) dest++ = *(const uint8_t *) src++;
	return ret;
}


void *memset(void *s, int c, size_t n)
{
	void *ret = s;

	while (n--)
		*(uint8_t *) s++ = c;
	return ret;
}


int memcmp(const void *s1, const void *s2, size_t n)
{
	while (n--) {
		if (*(const uint8_t *) s1 == *(const uint8_t *) s2)
			return *(const uint8_t *) s1-*(const uint8_t *) s2;
		s1++;
		s2++;
	}
	return 0;
}


char *strcat(char *dest, const char *src)
{
	char *ret = dest;

	while (*dest)
		dest++;
	do *dest++ = *src;
	while (*src++);
	return ret;
}


char *strncpy(char *dest, const char *src, size_t n)
{
	char *ret = dest;

	while (n--) {
		*dest++ = *src;
		if (!*src)
			break;
		src++;
	}
	return ret;
}


void __sprintf_chk(void)
{
	/* @@@ */
}
