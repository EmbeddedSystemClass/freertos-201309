#
# E407/Makefile - Build FreeRTOS for Olimex STM32-E407
#
# Developed by Werner Almesberger for Actility S.A., and
# licensed under LGPLv2 by Actility S.A.
#

NAME = demo
CHIP_FAMILY = STM32F4xx
CRYSTAL_Hz = 12000000

#
# -DUSE_STDPERIPH_DRIVER: make stm32f4xx_misc.c include stm32f4xx_conf.h
#

CFLAGS = -Wall -Os -g \
	-DUSE_STDPERIPH_DRIVER \
	-I. \
	-I../../Source/include \
	-I../Common/include

CFLAGS += -DFREERTOS_BUILD_DATE="\"`date`\""

DRIVERS = exti gpio misc rcc spi syscfg usart

SRC = $(NAME).c newlib.c serial.c \
	$(DRIVERS_C) \
	../Common/Minimal/flash.c \
	../../Source/list.c \
	../../Source/queue.c \
	../../Source/tasks.c \
	../../Source/timers.c \
	../../Source/portable/MemMang/heap_1.c

include Makefile.stm32

$(DRIVERS_C):	$(DEMOBUILDER)

#
# Adapt compiler name and increase configMINIMAL_STACK_SIZE from 130 to
# 1300 bytes.
#

FreeRTOSConfig.h: ../CORTEX_M4F_STM32F407ZG-SK/FreeRTOSConfig.h mkconfig
		$(BUILD) ./mkconfig $< >$@ || { rm -f $@; exit 1; }

$(SYSTEM_C):	$(PATH_SYSTEM_C) mksystem
		$(BUILD) ./mksystem $< >$@ || { rm -f $@; exit 1; }

clean::
		rm -f FreeRTOSConfig.h $(SYSTEM_C)

include Makefile.contiki


#####  Final dependencies  ####################################################

#
# Since things get added to OBJS until the end of the Makefile, we need to put
# these dependencies last.
#

$(OBJS):	FreeRTOSConfig.h
