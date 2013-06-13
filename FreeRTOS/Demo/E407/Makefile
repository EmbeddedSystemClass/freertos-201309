#
# E407/Makefile - Build FreeRTOS for Olimex STM32-E407
#
# Developed by Werner Almesberger for Actility S.A., and
# licensed under LGPLv2 by Actility S.A.
#

NAME = demo
CHIP_FAMILY = STM32F4xx

#
# -DUSE_STDPERIPH_DRIVER: make stm32f4xx_misc.c include stm32f4xx_conf.h
#
# -mfpu=fpv4-sp-d16 -mfloat-abi=softfp: work around vstmdbeq/vldmiaeq issue
# http://www.freertos.org/FreeRTOS_Support_Forum_Archive/October_2011/freertos_STM32F4_with_FPU_4761747.html
#
# -nostdlib: prevent the toolchain from picking up things from libc (which
# could trigger an assertion failure in ld)
#

CFLAGS = -Wall -Os \
	-DUSE_STDPERIPH_DRIVER \
	-mfpu=fpv4-sp-d16 -mfloat-abi=softfp \
	-nostdlib \
	-I. \
	-I../../Source/include \
	-I../../Source/portable/GCC/ARM_CM4F \
	-I$(DEMO_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/inc  \
	-I../Common/include

SRC = $(NAME).c libc.c \
	$(DEMO_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_misc.c \
	../Common/Minimal/flash.c \
	../../Source/list.c \
	../../Source/queue.c \
	../../Source/tasks.c \
	../../Source/timers.c \
	../../Source/portable/GCC/ARM_CM4F/port.c \
	../../Source/portable/MemMang/heap_1.c

include Makefile.stm32
