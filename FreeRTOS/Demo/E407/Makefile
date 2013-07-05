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

CFLAGS += -DFREERTOS_BUILD_DATE="\"`date`\""

CONTIKI_FLAGS = UIP_CONF_IPV6=1
CFLAGS += -DODEV
HAL = hal-spi

DRV_DIR = $(DEMO_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src

DRIVERS = exti gpio misc rcc spi syscfg usart
DRIVERS_C = $(DRIVERS:%=$(DRV_DIR)/stm32f4xx_%.c) \

SRC = $(NAME).c newlib.c serial.c $(HAL).c \
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

#####  Items for subordinate Contiki  #########################################

#
# Translate FreeRTOS chip family to Contiki-freertos target CPU selection.
# The target CPU is something that only exists in the hybrid. In regular
# ports, the target CPU is the same as the platform CPU. In "native" and
# similar, the target CPU is implicitly whatever the host has.
#

CONTIKI_TARGET_CPU = $(CONTIKI_TARGET_CPU_$(CHIP_FAMILY))
CONTIKI_TARGET_CPU_STM32F4xx = cm4

# Directory where contiki resides. Can be explicit path, subdirectory, symlink.
CONTIKI = contiki
OBJS += $(CONTIKI)/foo.co
OBJS += $(CONTIKI)/contiki-freertos.a

MAKE_CONTIKI = $(MAKE) -C $(CONTIKI) $(CONTIKI_FLAGS) \
    TARGET=freertos CONTIKI_TARGET_CPU=$(CONTIKI_TARGET_CPU) \
    FREERTOS=$(shell pwd)/../../.. \
    FREERTOS_TARGET=$(shell pwd) \
    FREERTOS_PORTABLE=$(shell pwd)/../../Source/portable/GCC/ARM_CM4F

.PHONY:		$(CONTIKI)/foo.co $(CONTIKI)/contiki-freertos.a

$(CONTIKI)/foo.co:
		$(MAKE_CONTIKI) foo.co

$(CONTIKI)/contiki-freertos.a:
		$(MAKE_CONTIKI) contiki-freertos.a

clean::
		$(MAKE_CONTIKI) clean

#####  Final dependencies  ####################################################

#
# Since things get added to OBJS until the end of the Makefile, we need to put
# these dependencies last.
#

$(OBJS):	FreeRTOSConfig.h

$(HAL).o:	$(CONTIKI)/contiki-freertos.a
