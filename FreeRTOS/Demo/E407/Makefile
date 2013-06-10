#
# Makefile - 
#
# Developed by Werner Almesberger for Actility S.A., and
# licensed under LGPLv2 by Actility S.A.
#

#
# Note: we treat the M4F largely as if it was an M3, to avoid running into
# problems with unsupported floating-point instructions. @@@ revisit later
#

NAME = demo

ARCH = arm-linux-gnueabi
CC = $(ARCH)-gcc
OBJCOPY = $(ARCH)-objcopy

LD_SCRIPT = stm32_flash.ld

CFLAGS = -Wall -mcpu=cortex-m3 -march=armv7-m -mthumb \
	-I$(DEMO_DIR)/Project/Core/Devices/STM32F2xx \
	-I$(DEMO_DIR)/Libraries/STM32F2xx_StdPeriph_Driver/inc \
	-I$(DEMO_DIR)/Libraries/CMSIS/Device/ST/STM32F2xx/Include \
	-I$(DEMO_DIR)/Libraries/CMSIS/Include

LDFLAGS = -nostartfiles -T$(LD_SCRIPT)

SRC = main.c system_stm32f2xx.c startup_stm32f2xx.s

# .../ARM_CM3/port.c (and not ARM_CM4F/port.c) because the latter uses
# unsupported instructions

OBJS = $(SRC:.c=.o)

.PHONY:	all dfu clean spotless

#----- Compilation ------------------------------------------------------------

all:		$(NAME).bin

$(NAME).bin:	$(NAME).elf
		$(OBJCOPY) -j .text -j .data -O binary $< $@ || \
		    { rm -f $@; exit 1; }

$(NAME).elf:	$(OBJS) $(LD_SCRIPT) Makefile
		$(CC) $(CFLAGS) -o $@ $(OBJS) $(LDFLAGS)

#----- Download the ST demo builder and cherry-pick the files we need ---------

#
# STSW-STM32106STM32F2 and STM32F4 demonstration builder platform
# http://www.st.com/web/catalog/tools/FM147/CL1794/SC961/SS1743/PF258142
#

#
# Note that ST's ZIP contains files we can't legally redistribute, even though
# few enough people seem to care about such details. Also note that ST only
# provide a file of the latest version, thus not allowing us to pin a known to
# be good version.
#

DEMO_DIR=STM32F2-F4_Demonstration_Builder_V1.3.0
DEMO_ZIP=stm32f2-f4_demobuild.zip
TEMPL=$(DEMO_DIR)/Libraries/CMSIS/Device/ST/STM32F2xx/Source/Templates/
STM_DEMO=http://www.st.com/st-web-ui/static/active/en/st_prod_software_internet/resource/technical/software/firmware/$(DEMO_ZIP)

$(DEMO_ZIP):
		wget $(STM_DEMO) || { rm -f $@; exit 1; }

$(BUILD_DIR):	$(DEMO_ZIP)
		unzip $< || { rm -rf $@; exit 1; }

startup_stm32f2xx.s: $(TEMPL)/gcc_ride7/startup_stm32f2xx.s
		cp $< $@ || { rm -f $@; exit 1; }

system_stm32f2xx.c: $(TEMPL)/system_stm32f2xx.c
		cp $< $@ || { rm -f $@; exit 1; }

$(LD_SCRIPT):	$(DEMO_DIR)/Project/TrueSTUDIO/STM324xG-EVAL/STM32F407IG_FLASH.ld
		cp $< $@ || { rm -f $@; exit 1; }

#----- Download firmware with DFU ---------------------------------------------

dfu:		$(NAME).bin
		dfu-util -d 0483:df11 -a 0 -s 0x8000000 -D $<

#----- Cleanup ----------------------------------------------------------------

clean:
		rm -f $(OBJS)
		rm -f startup_stm32f2xx.s system_stm32f2xx.c
		rm -f stm32_flash.ld
		rm -f $(NAME).elf $(NAME).bin

spotless:	clean
		rm -f STM32F2-F4_Demonstration_Builder_V1.3.0
		rm -f $(DEMO_ZIP)
