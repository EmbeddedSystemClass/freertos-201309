#
# WM09/Makefile - Build FreeRTOS for USI WM-N-BM-09 EVB
#
# Developed by Werner Almesberger for Actility S.A., and
# licensed under LGPLv2 by Actility S.A.
#

NAME = demo
CHIP_FAMILY = STM32F2xx
CRYSTAL_Hz = 26000000

CFLAGS = -Wall -Os -g \
	-DUSE_STDPERIPH_DRIVER \
	-I. \
	-I../../Source/include \
	-I../Common/include

CFLAGS += -DFREERTOS_BUILD_DATE="\"`date`\""

DRIVERS = gpio misc rcc usart
DRIVERS_C = $(DRIVERS:%=$(DRV_DIR)/stm32f2xx_%.c) \

SRC = $(NAME).c ../E407/newlib.c serial.c \
	$(DRIVERS_C) \
	../../Source/list.c \
	../../Source/queue.c \
	../../Source/tasks.c \
	../../Source/timers.c \
	../../Source/portable/MemMang/heap_1.c

include ../E407/Makefile.stm32

$(DRIVERS_C):	$(DEMOBUILDER)

#
# Adapt compiler name and increase configMINIMAL_STACK_SIZE from 130 to
# 1300 bytes.
#

FreeRTOSConfig.h: ../CORTEX_M4F_STM32F407ZG-SK/FreeRTOSConfig.h ../E407/mkconfig
		$(BUILD) ../E407/mkconfig $< >$@ || { rm -f $@; exit 1; }

$(SYSTEM_C):	$(PATH_SYSTEM_C) mksystem
		$(BUILD) ./mksystem $< >$@ || { rm -f $@; exit 1; }

clean::
		rm -f FreeRTOSConfig.h $(SYSTEM_C)

#####  Final dependencies  ####################################################

#
# Since things get added to OBJS until the end of the Makefile, we need to put
# these dependencies last.
#

$(OBJS):	FreeRTOSConfig.h


#####  Flash via OpenOCD #####

#
# Use BRCM's OpenOCD binary which has the BCM9WCD1EVAL1 layout and some other
# enhancements.
#
WICED = ../../../../wiced
OPENOCD = $(WICED)/Tools/OpenOCD/Linux64/openocd-all-brcm-libftdi
WICED_OPENOCD = $(WICED)/Tools/OpenOCD

flash:		$(NAME).elf
		$(OPENOCD) \
		    -f $(WICED_OPENOCD)/BCM9WCD1EVAL1.cfg \
		    -f $(WICED_OPENOCD)/stm32f2x.cfg \
		    -f $(WICED_OPENOCD)/stm32f2x-flash-app.cfg \
		    -c "flash write_image erase $<" \
		    -c shutdown

run:
		$(OPENOCD) \
		    -f $(WICED_OPENOCD)/BCM9WCD1EVAL1.cfg \
		    -f $(WICED_OPENOCD)/stm32f2x.cfg \
		    -c init \
		    -c "reset run" \
		    -c shutdown
