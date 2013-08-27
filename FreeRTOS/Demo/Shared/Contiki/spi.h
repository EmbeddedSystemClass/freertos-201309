/*
 * spi.h - General SPI interface
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

#ifndef SPI_H
#define	SPI_H

#include "platform.h"


/* include $(SPI).h */

#define	__HDR_2(file)	#file
#define	__HDR_1(name)	__HDR_2(name.h)
#define	__HDR		__HDR_1(SPI)

#include __HDR

#undef	__HDR
#undef	__HDR_1
#undef	__HDR_2

#endif /* !SPI_H */
