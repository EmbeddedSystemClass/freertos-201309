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
# -mfloat-abi=hard -mfpu=fpv4-sp-d16: generate code for FPU and use
# corresponding version of (multi-)newlib. This way, we're ABI-compatible
# with Contiki.
#

CFLAGS = -Wall -Os -g \
	-DUSE_STDPERIPH_DRIVER \
	-mfloat-abi=hard -mfpu=fpv4-sp-d16 \
	-I. \
	-I../../Source/include \
	-I../../Source/portable/GCC/ARM_CM4F \
	-I$(DEMO_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/inc  \
	-I../Common/include

DRV_DIR = $(DEMO_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src

DRIVERS = gpio misc rcc usart
DRIVERS_C = $(DRIVERS:%=$(DRV_DIR)/stm32f4xx_%.c) \

SRC = $(NAME).c newlib.c \
	$(DRIVERS_C) \
	../Common/Minimal/flash.c \
	../../Source/list.c \
	../../Source/queue.c \
	../../Source/tasks.c \
	../../Source/timers.c \
	../../Source/portable/GCC/ARM_CM4F/port.c \
	../../Source/portable/MemMang/heap_1.c

include Makefile.stm32

$(DRIVERS_C):	$(DEMOBUILDER)

$(NAME).o:	FreeRTOSConfig.h

#
# Adapt compiler name and increase configMINIMAL_STACK_SIZE from 130 to
# 1300 bytes.
#

FreeRTOSConfig.h: ../CORTEX_M4F_STM32F407ZG-SK/FreeRTOSConfig.h
		$(BUILD) sed 's/__ICCARM__/__GNUC__/;s/130/1300/' $< >$@ || \
		    { rm -f $@; exit 1; }

CONTIKI = $(shell pwd)/../../../../contiki-outoftree
OBJS += $(CONTIKI)/foo.co
LDFLAGS += $(CONTIKI)/contiki-freertos.a

clean::
		rm -f FreeRTOSConfig.h
