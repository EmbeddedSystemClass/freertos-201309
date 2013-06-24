/*
 * newlib.c - Helper functions for newlib
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

#include <sys/types.h>

#include "FreeRTOS.h"


caddr_t _sbrk(int incr)
{
	caddr_t tmp = (caddr_t) pvPortMalloc(incr);
	return tmp;
}
